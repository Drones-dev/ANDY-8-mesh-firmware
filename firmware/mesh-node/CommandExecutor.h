#pragma once

#include <stdint.h>

#include "PacketProtocol.h"

/*
  Delivers one mesh command to this drone's flight controller.

  Each mesh command is one or more MAVLink COMMAND_LONG steps. A step is sent
  up to kMaxAttempts times, every kRetryIntervalMs, until the flight
  controller answers with COMMAND_ACK. An accepted step moves on to the next;
  any refusal ends the command, so START never arms a drone whose Mission
  mode was refused. The outcome is held as a CommandAckPayload for the mesh
  to route back to the Ground Station. Transport-free so the firmware and the
  simulator share the same behavior.
*/
namespace AndyMesh {

// MAVLink command IDs used by the steps (MAV_CMD in the common dialect).
constexpr uint16_t kMavCmdNavLand = 21;
constexpr uint16_t kMavCmdDoSetMode = 176;
constexpr uint16_t kMavCmdComponentArmDisarm = 400;
// param2 value that makes PX4 disarm even while flying.
constexpr float kForceDisarmMagic = 21196.0F;
constexpr uint8_t kMavResultInProgress = 5;

struct FlightControllerStep {
  uint16_t command;
  float params[7];
};

class CommandExecutor {
 public:
  static const uint8_t kMaxAttempts = 5;
  static const uint32_t kRetryIntervalMs = 500;

  // Steps for a command; returns how many (0 for an unknown command).
  static uint8_t stepsFor(MeshCommand command,
                          const FlightControllerStep*& steps);

  void begin(NodeId myId);

  // A newer command replaces one still in progress.
  void start(MeshCommand command, uint16_t sequence, uint32_t nowMs);
  bool active() const;

  // True when step should be written to the flight controller now. attempt
  // is the zero-based try number (the MAVLink confirmation field).
  bool attemptDue(uint32_t nowMs, const FlightControllerStep*& step,
                  uint8_t& attempt);

  // The flight controller's COMMAND_ACK. Answers for other commands are
  // ignored.
  void onFlightControllerAck(uint16_t command, uint8_t result, uint32_t nowMs);

  void finish(uint8_t result);

  bool takeReport(CommandAckPayload& report);

 private:
  NodeId myId_ = 0;
  bool active_ = false;
  bool reportReady_ = false;
  MeshCommand command_ = MeshCommand::Land;
  uint16_t sequence_ = 0;
  const FlightControllerStep* steps_ = nullptr;
  uint8_t stepCount_ = 0;
  uint8_t step_ = 0;
  uint8_t attempts_ = 0;
  uint32_t nextAttemptMs_ = 0;
  CommandAckPayload report_{0, 0, MeshCommand::Land, 0};
};

}  // namespace AndyMesh
