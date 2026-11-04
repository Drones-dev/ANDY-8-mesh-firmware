#include "HelloManager.h"

namespace AndyMesh {

namespace {
// Subtracting two unsigned millis() values makes this comparison continue to
// work when the 32-bit counter wraps around after roughly 49.7 days.
bool timeReached(uint32_t nowMs, uint32_t deadlineMs) {
  return static_cast<int32_t>(nowMs - deadlineMs) >= 0;
}
}  // namespace

HelloManager::HelloManager()
    : nodeId_(0),
      role_(NodeRole::GroundStation),
      protocolVersion_(kProtocolVersion),
      intervalMs_(1000),
      nextSendMs_(0) {}

void HelloManager::begin(const NodeConfig& config, uint32_t nowMs) {
  nodeId_ = config.nodeId;
  role_ = config.role;
  protocolVersion_ = config.protocolVersion;
  intervalMs_ = config.helloIntervalMs;
  // A small deterministic offset reduces startup collisions while retaining the
  // configured one-second beacon interval. Without it, four boards powered at
  // the same moment could repeatedly begin their first HELLO at nearly the same
  // time. Each ID adds 75 ms, so the initial attempts are naturally separated.
  nextSendMs_ = nowMs + 100U + static_cast<uint32_t>(nodeId_) * 75U;
}

bool HelloManager::due(uint32_t nowMs) const {
  return timeReached(nowMs, nextSendMs_);
}

HelloPayload HelloManager::makePayload(uint32_t nowMs) const {
  // nowMs is the sender's uptime, not wall-clock time. All Arduino nodes have a
  // local millis() counter even when no GPS or real-time clock is available.
  // Routing fields default to "no route"; filled from Router before transmit.
  return HelloPayload{nodeId_, role_, protocolVersion_, nowMs, 255, 255};
}

void HelloManager::markSent(uint32_t nowMs) {
  // Schedule relative to the actual successful send time. If the radio was busy,
  // the next interval does not become artificially short.
  nextSendMs_ = nowMs + intervalMs_;
}

}  // namespace AndyMesh
