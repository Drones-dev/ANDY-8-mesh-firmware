#pragma once

#include <stdint.h>

#include "NeighborTable.h"

/*
  Router - next-hop selection for multi-hop telemetry.

  Each node tracks its hop count to the Ground Station (Ground Station = 0)
  and advertises it, along with its current next hop, in every HELLO.

  Route selection, run against the neighbor table:
    - Only ACTIVE neighbors with a hop count below kMaxHops are considered.
    - Neighbors whose next hop is this node are skipped (prevents loops).
    - Fewest hops wins; ties go to the stronger RSSI.
    - This node's hop count = selected neighbor's hop count + 1.

  When the selected neighbor times out (LOST), the next update() selects a
  replacement from the remaining ACTIVE neighbors.

  Also provides the duplicate filter used when relaying telemetry.
*/
namespace AndyMesh {

const uint8_t kNoRoute = 255;    // Hop count meaning "no route".
const NodeId kNoNextHop = 255;   // Next hop meaning "none".
const uint8_t kMaxHops = 5;      // Longest route accepted.

class Router {
 public:
  // isGroundStation fixes this node's hop count at 0.
  void begin(NodeId myId, bool isGroundStation);

  // Selects the next hop from the neighbor table. Returns true if the route
  // changed.
  bool update(const NeighborTable& neighbors);

  bool hasRoute() const;
  NodeId nextHop() const;         // kNoNextHop when there is no route.
  uint8_t hopsToGround() const;   // kNoRoute when there is no route.

  // Returns true if this telemetry packet was already handled; otherwise
  // records it and returns false. Timestamp is included so packets from a
  // rebooted drone (sequence restarted) are not mistaken for duplicates.
  bool alreadySeen(NodeId origin, uint16_t sequence, uint32_t timestampMs);

 private:
  NodeId myId_ = 0;
  bool isGroundStation_ = false;
  NodeId nextHop_ = kNoNextHop;
  uint8_t hops_ = kNoRoute;

  // Last 32 telemetry packets handled (ring buffer, oldest overwritten).
  static const int kSeenSize = 32;
  NodeId seenOrigin_[kSeenSize] = {};
  uint16_t seenSequence_[kSeenSize] = {};
  uint32_t seenTimestamp_[kSeenSize] = {};
  bool seenUsed_[kSeenSize] = {};
  int seenNext_ = 0;
};

}  // namespace AndyMesh
