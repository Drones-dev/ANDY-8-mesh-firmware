// Router unit tests: next-hop selection, reroute on neighbor loss, loop
// prevention, and duplicate filtering. Uses the neighbor table directly
// (no radio).

#include <assert.h>
#include <stdio.h>

#include "NeighborTable.h"
#include "Router.h"

using namespace AndyMesh;

// Records a HELLO from `from` in the table, advertising `hops` to the
// Ground Station and `via` as its next hop.
static void hearHello(NeighborTable& table, NodeId from, uint8_t hops,
                      float rssi, uint32_t nowMs, NodeId via = 0) {
  HelloPayload hello{from, NodeRole::Drone, kProtocolVersion, nowMs, hops, via};
  table.observeHello(hello, nowMs, rssi, 5.0F);
}

int main() {
  // Node under test: Drone 3, 3 s neighbor timeout.
  NeighborTable table;
  table.begin(3, 3000);
  Router router;
  router.begin(3, false);

  assert(!router.hasRoute());

  // Best next hop: equal hops, stronger RSSI wins (Drone 2).
  hearHello(table, 1, 1, -78.0F, 1000);
  hearHello(table, 2, 1, -62.0F, 1000);
  assert(router.update(table));
  assert(router.nextHop() == 2);
  assert(router.hopsToGround() == 2);

  // Fewer hops takes priority over RSSI.
  hearHello(table, 2, 3, -62.0F, 1500);
  router.update(table);
  assert(router.nextHop() == 1);
  hearHello(table, 2, 1, -62.0F, 1600);
  router.update(table);
  assert(router.nextHop() == 2);

  // Reroute: Drone 2 times out, Drone 1 is still heard.
  hearHello(table, 1, 1, -78.0F, 3500);
  hearHello(table, 1, 1, -78.0F, 4500);
  NodeId lost[kMaxNeighbors];
  size_t lostCount = table.updateTimeouts(4700, lost, kMaxNeighbors);
  assert(lostCount == 1 && lost[0] == 2);
  assert(router.update(table));
  assert(router.nextHop() == 1);
  assert(router.hopsToGround() == 2);

  // All neighbors lost: no route.
  table.updateTimeouts(9000, lost, kMaxNeighbors);
  router.update(table);
  assert(!router.hasRoute());

  // Loop prevention: a neighbor relaying through this node is not selected.
  hearHello(table, 4, 2, -50.0F, 9100, 3);
  router.update(table);
  assert(!router.hasRoute());

  // Duplicate filter.
  assert(!router.alreadySeen(3, 10, 5000));  // first copy
  assert(router.alreadySeen(3, 10, 5000));   // repeat
  assert(!router.alreadySeen(3, 11, 5000));  // next sequence
  assert(!router.alreadySeen(3, 10, 9999));  // same sequence after reboot

  // Ground Station: always 0 hops, never changes.
  Router ground;
  ground.begin(0, true);
  assert(ground.hasRoute());
  assert(ground.hopsToGround() == 0);
  assert(!ground.update(table));

  puts("routing_tests: PASS");
  return 0;
}
