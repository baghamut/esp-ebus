#ifndef _MAIN_HPP_
#define _MAIN_HPP_

#include <WiFiClient.h>
#include <WiFiServer.h>

#define MAX_SRV_CLIENTS 4
#define RXBUFFERSIZE 512 // On ESP8266, maximum 512 icw SoftwareSerial, otherwise you run out of heap
#define QUEUE_SIZE 480 
#define STACK_PROTECTOR  512 // bytes
#define HOSTNAME "esp-eBus"
#define RESET_MS 1000

// Reset the adapter when clients are connected but nothing could be
// delivered to any of them for this long
#define NO_COMMS_RESET_MS (200UL * 1000)
// Reset the adapter when the main loop (web server, Wi-Fi checks) has not
// run for this long. Checked from the data loop, skipped during OTA.
#define LOOP_STALL_RESET_MS (60UL * 1000)
// Longest firmware update during which the main loop stall check is paused
#define UPDATE_MAX_MS (10UL * 60 * 1000)
// Task watchdog timeout for the main loop and the data loop
#define TASK_WDT_TIMEOUT_S 6
// How long to retry the rest of a partially written enhanced frame
#define CLIENT_PARTIAL_WRITE_MS 50

#ifdef ESP32
#define UART_TX 20
#define UART_RX 21
#define USE_SOFTWARE_SERIAL 1
#define USE_ASYNCHRONOUS 1    // requires USE_SOFTWARE_SERIAL
#define AVAILABLE_THRESHOLD 0 // https://esp32.com/viewtopic.php?t=19788
#else
#define UART_TX 1
#define UART_RX 3
#define USE_SOFTWARE_SERIAL 0
#define USE_ASYNCHRONOUS 0    // requires USE_SOFTWARE_SERIAL
#define AVAILABLE_THRESHOLD 1
#endif

inline int DEBUG_LOG(const char *format, ...) { return 0;}
int DEBUG_LOG_IMPL(const char *format, ...);
//#define DEBUG_LOG DEBUG_LOG_IMPL

bool   handleNewClient(WiFiServer &server, WiFiClient clients[]);
size_t clientWrite(WiFiClient* client, const uint8_t* buf, size_t size);
int    pushClient(WiFiClient* client, uint8_t B);
void   handleClient(WiFiClient* client);

// Data that could not be delivered because a client socket was busy
extern volatile unsigned long nbrClientDrops;
// Clients closed because of a socket error or a broken enhanced frame
extern volatile unsigned long nbrClientStops;


#endif
