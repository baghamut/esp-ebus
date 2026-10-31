#include "main.hpp"
#include "bus.hpp"

#ifdef ESP32
#include <lwip/sockets.h>
#include <errno.h>
#endif

volatile unsigned long nbrClientDrops = 0;
volatile unsigned long nbrClientStops = 0;

bool handleNewClient(WiFiServer &server, WiFiClient clients[]) {
  if (!server.hasClient())
    return false;

  // Find free/disconnected slot
  int i;
  for (i = 0; i < MAX_SRV_CLIENTS; i++) {
    if (!clients[i]) { // equivalent to !serverClients[i].connected()
      clients[i] = server.accept();
      clients[i].setNoDelay(true);
      break;
    }
  }

  // No free/disconnected slot so reject
  if (i == MAX_SRV_CLIENTS) {
    server.accept().println("busy");
    // hints: server.available() is a WiFiClient with short-term scope
    // when out of scope, a WiFiClient will
    // - flush() - all data will be sent
    // - stop() - automatically too
  }

  return true;
}

// Write to a client without ever blocking the data loop.
//
// On ESP32, WiFiClient::write() waits in select() up to 10 x 1 s when the
// socket send queue is full, and availableForWrite() always returns 0, so the
// old threshold check never helped. One slow client could stall delivery to
// everybody and overflow the bus queue. Here we send with MSG_DONTWAIT:
// - all bytes sent           -> return size
// - socket busy, nothing sent -> drop the data for this client, return 0
// - socket error             -> stop the client, return 0
// A partial write (only possible for multi-byte enhanced frames) would break
// the protocol stream, so the remainder gets a short bounded retry and the
// client is stopped if it still does not fit.
size_t clientWrite(WiFiClient* client, const uint8_t* buf, size_t size) {
#ifdef ESP32
  int fd = client->fd();
  if (fd < 0)
    return 0;

  size_t sent = 0;
  unsigned long start = millis();
  while (sent < size) {
    int res = send(fd, buf + sent, size - sent, MSG_DONTWAIT);
    if (res > 0) {
      sent += res;
      continue;
    }
    if (res < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      nbrClientStops++;
      client->stop();
      return 0;
    }
    // Socket busy
    if (sent == 0) {
      nbrClientDrops++;
      return 0;
    }
    if (millis() - start > CLIENT_PARTIAL_WRITE_MS) {
      nbrClientStops++;
      client->stop();
      return 0;
    }
    vTaskDelay(1);
  }
  return sent;
#else
  if (client->availableForWrite() >= (int)size) {
    return client->write(buf, size);
  }
  nbrClientDrops++;
  return 0;
#endif
}

int pushClient(WiFiClient* client, uint8_t B){
    return clientWrite(client, &B, 1) == 1 ? 1 : 0;
}

void handleClient(WiFiClient* client){
    while (client->available() && Bus.availableForWrite() > 0) {
      // working char by char is not very efficient
      Bus.write(client->read());
    }
}
