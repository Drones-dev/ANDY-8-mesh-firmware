#pragma once

#include <stddef.h>
#include <stdint.h>

#include "PacketProtocol.h"

namespace AndyMesh {

// PacketTransport is the boundary between mesh behavior and the communication
// medium. The ESP32 implementation uses RadioLib/SX1262; the PC implementation
// uses an in-memory simulated radio. Neither transport interprets mesh packets.
struct TransportPacket {
  uint8_t data[kMaxPacketSize];
  size_t length;
  float rssiDbm;
  float snrDb;
};

class PacketTransport {
 public:
  virtual ~PacketTransport() {}
  virtual bool poll(TransportPacket& packet) = 0;
  virtual bool send(const uint8_t* data, size_t length) = 0;
  virtual bool readyToSend() const = 0;
};

}  // namespace AndyMesh
