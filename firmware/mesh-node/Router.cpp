#include "Router.h"

namespace AndyMesh {

void Router::begin(NodeId myId, bool isGroundStation) {
  myId_ = myId;
  isGroundStation_ = isGroundStation;

  if (isGroundStation_) {
    hops_ = 0;
    nextHop_ = myId_;
  } else {
    // No route until neighbors are heard.
    hops_ = kNoRoute;
    nextHop_ = kNoNextHop;
  }
}

bool Router::update(const NeighborTable& neighbors) {
  // The Ground Station is the destination; its route never changes.
  if (isGroundStation_) {
    return false;
  }

  NodeId oldNextHop = nextHop_;
  uint8_t oldHops = hops_;

  // Best-neighbor search.
  const NeighborEntry* best = nullptr;

  for (size_t i = 0; i < kMaxNeighbors; i++) {
    const NeighborEntry* n = neighbors.entryAt(i);

    if (n == nullptr) {
      continue;  // Empty slot.
    }
    if (n->status != NeighborStatus::Active) {
      continue;  // LOST neighbor.
    }
    if (n->hopsToGround >= kMaxHops) {
      continue;  // Neighbor has no route, or the route is too long.
    }
    if (n->nextHop == myId_) {
      continue;  // Neighbor relays through this node; selecting it loops.
    }

    if (best == nullptr) {
      best = n;
    } else if (n->hopsToGround < best->hopsToGround) {
      best = n;  // Fewer hops.
    } else if (n->hopsToGround == best->hopsToGround &&
               n->rssiDbm > best->rssiDbm) {
      best = n;  // Same hops, stronger signal.
    }
  }

  if (best != nullptr) {
    nextHop_ = best->nodeId;
    hops_ = best->hopsToGround + 1;
  } else {
    nextHop_ = kNoNextHop;
    hops_ = kNoRoute;
  }

  return nextHop_ != oldNextHop || hops_ != oldHops;
}

bool Router::hasRoute() const {
  return hops_ != kNoRoute;
}

NodeId Router::nextHop() const {
  return nextHop_;
}

uint8_t Router::hopsToGround() const {
  return hops_;
}

bool Router::alreadySeen(NodeId origin, uint16_t sequence,
                         uint32_t timestampMs) {
  for (int i = 0; i < kSeenSize; i++) {
    if (seenUsed_[i] && seenOrigin_[i] == origin &&
        seenSequence_[i] == sequence && seenTimestamp_[i] == timestampMs) {
      return true;
    }
  }

  // New packet stored in the next slot (wraps to 0).
  seenOrigin_[seenNext_] = origin;
  seenSequence_[seenNext_] = sequence;
  seenTimestamp_[seenNext_] = timestampMs;
  seenUsed_[seenNext_] = true;
  seenNext_ = (seenNext_ + 1) % kSeenSize;

  return false;
}

}  // namespace AndyMesh
