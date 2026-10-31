#include <ArduinoOTA.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h>
#include "main.hpp"
#include "enhanced.hpp"
#include "bus.hpp"
#include <Preferences.h>

#ifdef ESP32
  #include <esp_efuse.h>
#endif

Preferences preferences;

#ifdef ESP32
  #include <esp_task_wdt.h>
  #include <Update.h>
  #include <ESPmDNS.h>
  #include "esp32c3/rom/rtc.h"
  #include <IotWebConfESP32HTTPUpdateServer.h>

  HTTPUpdateServer httpUpdater;
#else
  #include <ESP8266mDNS.h>
  #include <ESP8266TrueRandom.h>
  #include <ESP8266HTTPUpdateServer.h>

  ESP8266HTTPUpdateServer httpUpdater;
#endif

#define ALPHA 0.3

#define PWM_CHANNEL 0
#define PWM_FREQ 10000
#define PWM_RESOLUTION 8

#define DEFAULT_AP "ebus-test"
#define DEFAULT_PASS "lectronz"
#define DEFAULT_APMODE_PASS "ebusebus"

#ifdef ESP32
TaskHandle_t Task1;
#endif

#define CONFIG_VERSION "eea"
DNSServer dnsServer;
WebServer configServer(80);
char pwm_value_string[8];
IotWebConf iotWebConf(HOSTNAME, &dnsServer, &configServer, "", CONFIG_VERSION);
IotWebConfNumberParameter pwm_value_param = IotWebConfNumberParameter("PWM value", "pwm_value", pwm_value_string, 8, "130", "1..255", "min='1' max='255' step='1'");

WiFiServer wifiServer(3333);
WiFiServer wifiServerRO(3334);
WiFiServer wifiServerEnh(3335);
WiFiServer statusServer(5555);
WiFiClient serverClients[MAX_SRV_CLIENTS];
WiFiClient serverClientsRO[MAX_SRV_CLIENTS];
WiFiClient enhClients[MAX_SRV_CLIENTS];

// Last time bus data reached a connected client (or no client was connected).
// Written by the data loop, read by the main loop.
volatile unsigned long last_comms = 0;
// Last time the main loop ran. Written by the main loop, read by the data loop.
volatile unsigned long last_main_loop = 0;
int last_reset_code = -1;

unsigned long loopDuration = 0;
unsigned long maxLoopDuration = 0;
unsigned long lastConnectTime = 0;
int reconnectCount = 0;

uint8_t adapterHwVersionRaw = 0xEE;
String adapterHwVersion = "unread";
int statusLedPin = -1;

#ifdef ESP32
enum class AdapterHwVersionEfuse : uint8_t {
  PRE_7_0 = 0x00,
  V7_0 = 0x70,
};

static constexpr size_t ADAPTER_HW_VERSION_EFUSE_BITS = 8;
static constexpr size_t ADAPTER_HW_VERSION_EFUSE_OFFSET = 248;  // BLOCK3 bit 248..255

static const esp_efuse_desc_t ADAPTER_HW_VERSION_EFUSE_DESC = {
    EFUSE_BLK3, ADAPTER_HW_VERSION_EFUSE_OFFSET, ADAPTER_HW_VERSION_EFUSE_BITS};
static const esp_efuse_desc_t* ADAPTER_HW_VERSION_EFUSE_FIELD[] = {
    &ADAPTER_HW_VERSION_EFUSE_DESC, nullptr};
#endif

int random_ch(){
#ifdef ESP32
  return esp_random() % 13 + 1 ;
#elif defined(ESP8266)
  return ESP8266TrueRandom.random(1, 13);
#endif
}

#ifdef ESP32
void on_connected(WiFiEvent_t event, WiFiEventInfo_t info){
  lastConnectTime = millis();
  reconnectCount++;
}
#endif

// Watchdogs on ESP32:
// - The data loop is registered with the hardware task watchdog (TWDT) in
//   data_loop() and feeds it every iteration. It must never block for long.
// - The main loop is NOT registered with the TWDT, because OTA and HTTP
//   firmware uploads legitimately block it for tens of seconds. Instead it
//   updates last_main_loop, and the data loop resets the adapter when the
//   main loop has not run for LOOP_STALL_RESET_MS (see check_main_loop()).
// In v7.1.1 no task was registered, so the TWDT watched nothing.
void wdt_start() {
#ifdef ESP32
  // The Arduino core already initialised the TWDT; this reconfigures it
  esp_task_wdt_init(TASK_WDT_TIMEOUT_S, true);
#elif defined(ESP8266)
  ESP.wdtDisable();
#endif
}

void wdt_feed() {
#ifdef ESP32
  last_main_loop = millis();
#elif defined(ESP8266)
  ESP.wdtFeed();
#else
#error UNKNOWN PLATFORM
#endif
}

inline void disableTX() {
#ifdef TX_DISABLE_PIN
  pinMode(TX_DISABLE_PIN, OUTPUT);
  digitalWrite(TX_DISABLE_PIN, HIGH);
#endif
}

inline void enableTX() {
#ifdef TX_DISABLE_PIN
  digitalWrite(TX_DISABLE_PIN, LOW);
#endif
}

void set_pwm(uint8_t value){
#ifdef PWM_PIN
  ledcWrite(PWM_CHANNEL, value);
#endif
}

uint32_t get_pwm(){
#ifdef PWM_PIN
  return ledcRead(PWM_CHANNEL);
#endif
  return 0;
}

String formatAdapterHwVersion(const uint8_t raw) {
#ifdef ESP32
  if (static_cast<AdapterHwVersionEfuse>(raw) == AdapterHwVersionEfuse::PRE_7_0) {
    return "pre-7.0";
  }

  const uint8_t major = (raw >> 4) & 0x0F;
  const uint8_t minor = raw & 0x0F;
  if (major <= 9 && minor <= 9) {
    return String(major) + "." + String(minor);
  }

  char tmp[8]{};
  snprintf(tmp, sizeof(tmp), "0x%02X", raw);
  return String(tmp);
#else
  (void)raw;
  return "n/a";
#endif
}

void loadAdapterHwVersionFromEfuse() {
#ifdef ESP32
  uint8_t raw;
  const esp_err_t err =
      esp_efuse_read_field_blob(ADAPTER_HW_VERSION_EFUSE_FIELD, &raw,
                                ADAPTER_HW_VERSION_EFUSE_BITS);
  if (err != ESP_OK) {
    adapterHwVersionRaw = 0xEE;
    adapterHwVersion = "reading error";
    return;
  }

  adapterHwVersionRaw = raw;
  adapterHwVersion = formatAdapterHwVersion(raw);
  if (static_cast<AdapterHwVersionEfuse>(raw) == AdapterHwVersionEfuse::V7_0) {
    statusLedPin = 5;
  } else {
    statusLedPin = 3;
  }
#else
  adapterHwVersionRaw = 0xFF;
  adapterHwVersion = "n/a";
  statusLedPin = -1;
#endif
}

void reset(){
  disableTX();
  ESP.restart();
}

void reset_config() {
  preferences.clear();
  reset();
}

void check_reset() {
  // check if RESET_PIN being hold low and reset
  pinMode(RESET_PIN, INPUT_PULLUP);
  unsigned long resetStart = millis();
  while(digitalRead(RESET_PIN) == 0){
    if (millis() > resetStart + RESET_MS){
      reset_config();
    }
  }
}

void loop_duration() {
  static unsigned long lastTime = 0;
  unsigned long now = micros();
  unsigned long delta = now - lastTime;

  lastTime = now;

  loopDuration = ((1 - ALPHA) * loopDuration + (ALPHA * delta));

  if (delta > maxLoopDuration) {
    maxLoopDuration = delta;
  }
}

bool any_client_connected(){
  for (int i = 0; i < MAX_SRV_CLIENTS; i++){
#ifdef ESP32
    // fd() is cheap; a socket closed by the peer is stopped on the next write
    if (serverClients[i].fd() >= 0 || serverClientsRO[i].fd() >= 0 || enhClients[i].fd() >= 0)
#else
    if (serverClients[i] || serverClientsRO[i] || enhClients[i])
#endif
      return true;
  }
  return false;
}

void data_process(){
  loop_duration();

  //check clients for data
  for (int i = 0; i < MAX_SRV_CLIENTS; i++){
    handleClient(&serverClients[i]);
    handleEnhClient(&enhClients[i]);
  }

  // Nobody to deliver to is not a communication failure
  if (!any_client_connected()) {
    last_comms = millis();
  }

  //check queue for data
  BusType::data d;
#ifdef ESP32
  // Wait up to 1 tick for bus data instead of busy polling, so the idle task
  // and other low priority tasks get CPU time
  if (Bus.read(d, 1)) {
#else
  if (Bus.read(d)) {
#endif
    for (int i = 0; i < MAX_SRV_CLIENTS; i++){
      if (d._enhanced) {
        if (d._client == &enhClients[i]) {
          if (pushEnhClient(&enhClients[i], d._c, d._d, true)) {
            last_comms = millis();
          }
        }
      }
      else {
        if (pushClient(&serverClients[i], d._d)){
          last_comms = millis();
        }
        if (pushClient(&serverClientsRO[i], d._d)){
          last_comms = millis();
        }
        if (d._client != &enhClients[i]) {
          if (pushEnhClient(&enhClients[i], d._c, d._d, d._logtoclient == &enhClients[i])){
            last_comms = millis();
          }
        }
      }
    }
  }
}

#ifdef ESP32
// Reset when the main loop (web server, Wi-Fi checks, client accept) is stuck.
// Firmware updates block the main loop on purpose, so they get extra time.
void check_main_loop(){
  static unsigned long update_since = 0;
  unsigned long now = millis();

  if (Update.isRunning()) {
    if (update_since == 0) {
      update_since = now;
    }
    if (now - update_since < UPDATE_MAX_MS) {
      return;
    }
  } else {
    update_since = 0;
  }

  // Signed difference: last_main_loop may be updated after "now" was read
  long stalled = (long)(now - last_main_loop);
  if (stalled > (long)LOOP_STALL_RESET_MS) {
    reset();
  }
}

void data_loop(void * pvParameters){
  esp_task_wdt_add(NULL);
  while(1){
    esp_task_wdt_reset();
    data_process();
    check_main_loop();
  }
}
#endif


void saveParamsCallback () {

  uint8_t new_pwm_value = atoi(pwm_value_string);
  if (new_pwm_value > 0){
    set_pwm(new_pwm_value);
    preferences.putUInt("pwm_value", new_pwm_value);
  }
  DebugSer.printf("pwm_value set: %s %d\n", pwm_value_string, new_pwm_value);

}

char* status_string(){
  static char status[1024];

  int pos = 0;

  pos += sprintf(status + pos, "async mode: %s\n", USE_ASYNCHRONOUS ? "true" : "false");
  pos += sprintf(status + pos, "software serial mode: %s\n", USE_SOFTWARE_SERIAL ? "true" : "false");
  pos += sprintf(status + pos, "uptime: %ld ms\n", millis());
  pos += sprintf(status + pos, "last_connect_time: %ld ms\n", lastConnectTime);
  pos += sprintf(status + pos, "reconnect_count: %d \n", reconnectCount);
  pos += sprintf(status + pos, "rssi: %d dBm\n", WiFi.RSSI());
  pos += sprintf(status + pos, "free_heap: %d B\n", ESP.getFreeHeap());
  pos += sprintf(status + pos, "reset_code: %d\n", last_reset_code);
  pos += sprintf(status + pos, "loop_duration: %ld us\r\n", loopDuration);
  pos += sprintf(status + pos, "max_loop_duration: %ld us\r\n", maxLoopDuration);
  pos += sprintf(status + pos, "version: %s\r\n", AUTO_VERSION);
  pos += sprintf(status + pos, "adapter_hw_version: %s\r\n", adapterHwVersion.c_str());
  pos += sprintf(status + pos, "adapter_hw_version_raw: 0x%02X\r\n", adapterHwVersionRaw);
  pos += sprintf(status + pos, "status_led_pin: %d\r\n", statusLedPin);
  pos += sprintf(status + pos, "nbr arbitrations: %i\r\n", (int)Bus._nbrArbitrations);
  pos += sprintf(status + pos, "nbr restarts1: %i\r\n", (int)Bus._nbrRestarts1);
  pos += sprintf(status + pos, "nbr restarts2: %i\r\n", (int)Bus._nbrRestarts2);
  pos += sprintf(status + pos, "nbr lost1: %i\r\n", (int)Bus._nbrLost1);
  pos += sprintf(status + pos, "nbr lost2: %i\r\n", (int)Bus._nbrLost2);
  pos += sprintf(status + pos, "nbr won1: %i\r\n", (int)Bus._nbrWon1);
  pos += sprintf(status + pos, "nbr won2: %i\r\n", (int)Bus._nbrWon2);
  pos += sprintf(status + pos, "nbr late: %i\r\n", (int)Bus._nbrLate);
  pos += sprintf(status + pos, "nbr errors: %i\r\n", (int)Bus._nbrErrors);
  pos += sprintf(status + pos, "nbr queue overflows: %i\r\n", (int)Bus._nbrOverflows);
  pos += sprintf(status + pos, "nbr client drops: %lu\r\n", nbrClientDrops);
  pos += sprintf(status + pos, "nbr client stops: %lu\r\n", nbrClientStops);
  pos += sprintf(status + pos, "pwm_value: %i\r\n", get_pwm());

  return status;
}

void handleStatus()
{
  configServer.send(200, "text/plain", status_string());
}


void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<html><head><title>esp-eBus adapter</title>";
  s += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/>";
  s += "</head><body>";
  s += "<a href='/status'>Adapter status</a><br>";
  s += "<a href='/config'>Configuration</a> - user: admin password: your configured AP mode password or default: ";
  s += DEFAULT_APMODE_PASS;
  s += "<br>";
  s += "<a href='/firmware'>Firmware update</a><br>";
  s += "<br>";
  s += "For more info see project page: <a href='https://github.com/danielkucera/esp-arduino-ebus'>https://github.com/danielkucera/esp-arduino-ebus</a>";
  s += "</body></html>";

  configServer.send(200, "text/html", s);
}

void setup() {
  preferences.begin("esp-ebus", false);

  check_reset();

#ifdef ESP32
  last_reset_code = rtc_get_reset_reason(0);
#elif defined(ESP8266)
  last_reset_code = (int) ESP.getResetInfoPtr();
#endif

  loadAdapterHwVersionFromEfuse();

  if (statusLedPin >= 0) {
    pinMode(statusLedPin, OUTPUT);
    digitalWrite(statusLedPin, HIGH);
  }

  Bus.begin();

  DebugSer.begin(115200);
  DebugSer.setDebugOutput(true);

  disableTX();

#ifdef PWM_PIN
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
#endif

  set_pwm(preferences.getUInt("pwm_value", 130));

  if (preferences.getBool("firstboot", true)){
    preferences.putBool("firstboot", false);
    iotWebConf.init();
    strncpy(iotWebConf.getApPasswordParameter()->valueBuffer, DEFAULT_APMODE_PASS, IOTWEBCONF_WORD_LEN);
    strncpy(iotWebConf.getWifiSsidParameter()->valueBuffer, "ebus-test", IOTWEBCONF_WORD_LEN);
    strncpy(iotWebConf.getWifiPasswordParameter()->valueBuffer, "lectronz", IOTWEBCONF_WORD_LEN);
    iotWebConf.saveConfig();
  } else {
    iotWebConf.skipApStartup();
  }

#ifdef ESP32
  WiFi.onEvent(on_connected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
#endif

  int wifi_ch = random_ch();
  DebugSer.printf("Channel for AP mode: %d\n", wifi_ch);
  WiFi.channel(wifi_ch); // doesn't work, https://github.com/prampec/IotWebConf/issues/286

  iotWebConf.addSystemParameter(&pwm_value_param);
  iotWebConf.setConfigSavedCallback(&saveParamsCallback);
  iotWebConf.getApTimeoutParameter()->visible = true;
  iotWebConf.setWifiConnectionTimeoutMs(7000);

  if (statusLedPin >= 0) {
    iotWebConf.setStatusPin(statusLedPin);
  }

  // -- Initializing the configuration.
  iotWebConf.init();

  // -- Set up required URL handlers on the web server.
  configServer.on("/", []{ handleRoot(); });
  configServer.on("/config", []{ iotWebConf.handleConfig(); });
  configServer.on("/param", []{ iotWebConf.handleConfig(); });
  configServer.on("/status", []{ handleStatus(); });
  configServer.onNotFound([](){ iotWebConf.handleNotFound(); });

  iotWebConf.setupUpdateServer(
    [](const char* updatePath) { httpUpdater.setup(&configServer, updatePath); },
    [](const char* userName, char* password) { httpUpdater.updateCredentials(userName, password); });

  while (iotWebConf.getState() != iotwebconf::NetworkState::OnLine){
      iotWebConf.doLoop();
  }

  wifiServer.begin();
  wifiServerRO.begin();
  wifiServerEnh.begin();
  statusServer.begin();

  ArduinoOTA.begin();

  MDNS.end();
  MDNS.begin(HOSTNAME);

  wdt_start();

  last_comms = millis();
  // Setup can take a while (Wi-Fi connect), don't count it as a stall
  last_main_loop = millis();

#ifdef ESP32
  xTaskCreate(data_loop, "data_loop", 10000, NULL, 1, &Task1);
#endif
}

bool handleStatusServerRequests() {
  if (!statusServer.hasClient())
    return false;

  WiFiClient client = statusServer.accept();

  if (client.availableForWrite() >= AVAILABLE_THRESHOLD) {
    client.print(status_string());
    client.flush();
    client.stop();
  }
  return true;
}

void loop() {
  ArduinoOTA.handle();

#ifdef ESP8266
  MDNS.update();

  data_process();
#endif

  wdt_feed();

#ifdef ESP32
  iotWebConf.doLoop();
#endif

  if (WiFi.status() != WL_CONNECTED) {
    reset();
  }

  // Clients are connected but nothing reached any of them for too long.
  // Signed difference: last_comms is updated by the data loop concurrently,
  // and this form also survives the millis() overflow after 49 days.
  if ((long)(millis() - last_comms) > (long)NO_COMMS_RESET_MS) {
    reset();
  }

  // Check if new client on the status server
  if (handleStatusServerRequests()) {
    // exclude handleStatusServerRequests from maxLoopDuration calculation
    // as it skews the typical loop duration and set maxLoopDuration to 0
    loop_duration();
    maxLoopDuration = 0;
  }

  // Check if there are any new clients on the eBUS servers
  if (handleNewClient(wifiServer, serverClients)){
    enableTX();
  }
  if (handleNewClient(wifiServerEnh, enhClients)){
    enableTX();
  }

  handleNewClient(wifiServerRO, serverClientsRO);

#ifdef ESP32
  // The Arduino loop task never yields by itself. Give the data loop, the
  // idle task and lower priority tasks a chance to run.
  vTaskDelay(1);
#endif
}
