#pragma once

#include <stdint.h>

#include "PacketProtocol.h"

/*
  Delivers one mesh command to this drone's flight controller.

  The flight controller is sent the command up to kMaxAttempts times, every
  kRetryIntervalMs, until it answers with COMMAND_ACK. The outcome is then held
  as a CommandAckPayload for the mesh to route back to the Ground Station.
  Transport-free so the firmware and the simulator share the same behavior.
*/
namespace AndyMesh {

class CommandExecutor {
 public:
  static const uint8_t kMaxAttempts = 5;
  static const uint32_t kRetryIntervalMs = 500;

  void begin(NodeId myId);

  // A newer command replaces one still in progress.
  void start(MeshCommand command, uint16_t sequence, uint32_t nowMs);
  bool active() const;

  // True when the command should be written to the flight controller now.
  // attempt is the zero-based try number (the MAVLink confirmation field).
  bool attemptDue(uint32_t nowMs, uint8_t& attempt);

  void finish(uint8_t result);

  bool takeReport(CommandAckPayload& report);

 private:
  NodeId myId_ = 0;
  bool active_ = false;
  bool reportReady_ = false;
  MeshCommand command_ = MeshCommand::Land;
  uint16_t sequence_ = 0;
  uint8_t attempts_ = 0;
  uint32_t nextAttemptMs_ = 0;
  CommandAckPayload report_{0, 0, MeshCommand::Land, 0};
};

}  // namespace AndyMesh
