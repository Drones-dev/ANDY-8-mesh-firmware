#include "CommandExecutor.h"

#include <math.h>

namespace AndyMesh {

namespace {
// NaN yaw and position mean "keep current heading, land where you are".
const FlightControllerStep kLandSteps[] = {
    {kMavCmdNavLand, {0, 0, 0, NAN, NAN, NAN, NAN}},
};
// Mission mode first: PX4 refuses it without a valid uploaded mission and a
// position fix, and then the drone is never armed. PX4 custom mode 4/4 is
// AUTO / MISSION; base mode 1 means "custom mode fields are used".
const FlightControllerStep kStartSteps[] = {
    {kMavCmdDoSetMode, {1, 4, 4, 0, 0, 0, 0}},
    {kMavCmdComponentArmDisarm, {1, 0, 0, 0, 0, 0, 0}},
};
const FlightControllerStep kKillSteps[] = {
    {kMavCmdComponentArmDisarm, {0, kForceDisarmMagic, 0, 0, 0, 0, 0}},
};
}  // namespace

uint8_t CommandExecutor::stepsFor(MeshCommand command,
                                  const FlightControllerStep*& steps) {
  switch (command) {
    case MeshCommand::Land:
      steps = kLandSteps;
      return sizeof(kLandSteps) / sizeof(kLandSteps[0]);
    case MeshCommand::Start:
      steps = kStartSteps;
      return sizeof(kStartSteps) / sizeof(kStartSteps[0]);
    case MeshCommand::Kill:
      steps = kKillSteps;
      return sizeof(kKillSteps) / sizeof(kKillSteps[0]);
  }
  steps = nullptr;
  return 0;
}

void CommandExecutor::begin(NodeId myId) {
  myId_ = myId;
  active_ = false;
  reportReady_ = false;
}

void CommandExecutor::start(MeshCommand command, uint16_t sequence,
                            uint32_t nowMs) {
  command_ = command;
  sequence_ = sequence;
  stepCount_ = stepsFor(command, steps_);
  step_ = 0;
  attempts_ = 0;
  nextAttemptMs_ = nowMs;
  reportReady_ = false;
  active_ = stepCount_ > 0;
}

bool CommandExecutor::active() const { return active_; }

bool CommandExecutor::attemptDue(uint32_t nowMs,
                                 const FlightControllerStep*& step,
                                 uint8_t& attempt) {
  if (!active_ || static_cast<int32_t>(nowMs - nextAttemptMs_) < 0) {
    return false;
  }
  if (attempts_ >= kMaxAttempts) {
    finish(kCommandResultNoResponse);
    return false;
  }
  step = &steps_[step_];
  attempt = attempts_++;
  nextAttemptMs_ = nowMs + kRetryIntervalMs;
  return true;
}

void CommandExecutor::onFlightControllerAck(uint16_t command, uint8_t result,
                                            uint32_t nowMs) {
  if (!active_ || command != steps_[step_].command) return;
  if (result == kMavResultInProgress) return;
  if (result != kCommandResultAccepted) {
    finish(result);
    return;
  }
  if (step_ + 1 < stepCount_) {
    ++step_;
    attempts_ = 0;
    nextAttemptMs_ = nowMs;
    return;
  }
  finish(kCommandResultAccepted);
}

void CommandExecutor::finish(uint8_t result) {
  if (!active_) return;
  active_ = false;
  report_ = CommandAckPayload{myId_, sequence_, command_, result};
  reportReady_ = true;
}

bool CommandExecutor::takeReport(CommandAckPayload& report) {
  if (!reportReady_) return false;
  report = report_;
  reportReady_ = false;
  return true;
}

}  // namespace AndyMesh
