#include "CommandExecutor.h"

namespace AndyMesh {

void CommandExecutor::begin(NodeId myId) {
  myId_ = myId;
  active_ = false;
  reportReady_ = false;
}

void CommandExecutor::start(MeshCommand command, uint16_t sequence,
                            uint32_t nowMs) {
  command_ = command;
  sequence_ = sequence;
  attempts_ = 0;
  nextAttemptMs_ = nowMs;
  active_ = true;
  reportReady_ = false;
}

bool CommandExecutor::active() const { return active_; }

bool CommandExecutor::attemptDue(uint32_t nowMs, uint8_t& attempt) {
  if (!active_ || static_cast<int32_t>(nowMs - nextAttemptMs_) < 0) {
    return false;
  }
  if (attempts_ >= kMaxAttempts) {
    finish(kCommandResultNoResponse);
    return false;
  }
  attempt = attempts_++;
  nextAttemptMs_ = nowMs + kRetryIntervalMs;
  return true;
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
