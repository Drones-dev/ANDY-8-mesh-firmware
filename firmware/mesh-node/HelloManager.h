#pragma once

#include <stdint.h>

#include "NodeConfig.h"
#include "PacketProtocol.h"

/*
  HELLO beacon scheduler

  A HELLO beacon is a small broadcast packet sent at a regular interval. It lets
  nearby nodes discover one another automatically and acts as a heartbeat: if
  several expected HELLOs stop arriving, the neighbor can be marked LOST.
*/
namespace AndyMesh {

class HelloManager {
 public:
  HelloManager();

  // begin copies the identity/timing settings for this node. due is a quick
  // nonblocking timer check, makePayload builds the introduction, and markSent
  // schedules the next beacon only after the radio accepted this one.
  void begin(const NodeConfig& config, uint32_t nowMs);
  bool due(uint32_t nowMs) const;
  HelloPayload makePayload(uint32_t nowMs) const;
  void markSent(uint32_t nowMs);

 private:
  NodeId nodeId_;             // Identity placed in each HELLO payload.
  NodeRole role_;             // GROUND, GATEWAY, RELAY, or DRONE.
  uint8_t protocolVersion_;   // Allows incompatible nodes to reject the beacon.
  uint32_t intervalMs_;       // Time between successful HELLO transmissions.
  uint32_t nextSendMs_;       // millis() deadline for the next beacon.
};

}  // namespace AndyMesh
