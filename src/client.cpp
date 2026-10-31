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

#ifdef ESP32
// Non-blocking client output with a small per-client buffer.
//
// On ESP32, WiFiClient::write() waits in select() up to 10 x 1 s when the
// socket send queue is full, and availableForWrite() always returns 0, so
// the old threshold check never helped: one slow client stalled the data
// loop for seconds and the bus queue overflowed.
//
// With TCP_NODELAY every bus byte is its own segment, and lwIP queues only
// a few unacknowledged segments per socket, so "send queue full" happens
// whenever ACKs are a bit late on Wi-Fi. Instead of blocking or dropping,
// bytes are kept in a per-client buffer and sent with MSG_DONTWAIT on the
// next call, which also merges them into one segment. Data is only dropped
// when a client's buffer is full (it stopped reading for ~2 s of bus data).

struct ClientBuffer {
  WiFiClient* client;
  int         fd;
  uint16_t    len;
  uint8_t     data[CLIENT_BUFFER_SIZE];
};

static ClientBuffer buffers[CLIENT_BUFFER_SLOTS];

static ClientBuffer* bufferFor(WiFiClient* client, int fd) {
  ClientBuffer* free_slot = nullptr;
  for (int i = 0; i < CLIENT_BUFFER_SLOTS; i++) {
    if (buffers[i].client == client) {
      if (buffers[i].fd != fd) {
        // A new connection in the same client slot: forget old data
        buffers[i].fd = fd;
        buffers[i].len = 0;
      }
      return &buffers[i];
    }
    if (free_slot == nullptr && buffers[i].client == nullptr) {
      free_slot = &buffers[i];
    }
  }
  if (free_slot != nullptr) {
    free_slot->client = client;
    free_slot->fd = fd;
    free_slot->len = 0;
  }
  return free_slot;
}

static void stopClient(WiFiClient* client, ClientBuffer* buffer) {
  nbrClientStops++;
  if (buffer != nullptr) {
    buffer->len = 0;
  }
  client->stop();
}

// Send as much of the buffered data as the socket takes right now.
// Returns false if the client was stopped because of a socket error.
static bool flushBuffer(WiFiClient* client, ClientBuffer* buffer) {
  if (buffer->len == 0)
    return true;

  int res = send(buffer->fd, buffer->data, buffer->len, MSG_DONTWAIT);
  if (res > 0) {
    buffer->len -= res;
    if (buffer->len > 0) {
      memmove(buffer->data, buffer->data + res, buffer->len);
    }
    return true;
  }
  if (res < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    stopClient(client, buffer);
    return false;
  }
  return true;  // socket busy, try again later
}
#endif

// Queue data for a client without ever blocking. Multi-byte data (enhanced
// frames) is taken whole or not at all, so the protocol stream stays intact.
// Returns size if the data was sent or buffered, 0 if it was dropped.
size_t clientWrite(WiFiClient* client, const uint8_t* buf, size_t size) {
#ifdef ESP32
  int fd = client->fd();
  if (fd < 0)
    return 0;

  ClientBuffer* buffer = bufferFor(client, fd);
  if (buffer == nullptr) {
    // No buffer slot (should not happen): fall back to a direct attempt
    int res = send(fd, buf, size, MSG_DONTWAIT);
    if (res == (int)size)
      return size;
    nbrClientDrops++;
    return 0;
  }

  if (!flushBuffer(client, buffer))
    return 0;

  if (buffer->len == 0) {
    // Fast path: nothing pending, send directly and keep any remainder
    int res = send(fd, buf, size, MSG_DONTWAIT);
    if (res < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        stopClient(client, buffer);
        return 0;
      }
      res = 0;
    }
    if ((size_t)res == size)
      return size;
    buf += res;
    size -= res;
  }

  if (buffer->len + size > CLIENT_BUFFER_SIZE) {
    nbrClientDrops++;
    return 0;
  }
  memcpy(buffer->data + buffer->len, buf, size);
  buffer->len += size;
  return size;
#else
  if (client->availableForWrite() >= (int)size) {
    return client->write(buf, size);
  }
  nbrClientDrops++;
  return 0;
#endif
}

// Push out buffered data for a client, called every data loop pass
void clientFlush(WiFiClient* client) {
#ifdef ESP32
  int fd = client->fd();
  for (int i = 0; i < CLIENT_BUFFER_SLOTS; i++) {
    if (buffers[i].client == client) {
      if (fd < 0 || buffers[i].fd != fd) {
        buffers[i].len = 0;
        return;
      }
      flushBuffer(client, &buffers[i]);
      return;
    }
  }
#else
  (void)client;
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
