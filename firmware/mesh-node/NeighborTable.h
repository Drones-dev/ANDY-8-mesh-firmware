#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "PacketProtocol.h"

/*
  Direct-neighbor tracking

  A neighbor is another node whose HELLO packet this radio received directly.
  The table does not assume every drone can hear every other drone, and it does
  not add nodes merely because another node mentioned them. That distinction is
  important for future route selection because RSSI/SNR describe only a direct
  radio link.
*/
namespace AndyMesh {

// Eight fixed entries use predictable memory and avoid heap allocation. The
// Phase 2 network has only four total nodes, leaving room for expansion.
constexpr size_t kMaxNeighbors = 8;

struct NeighborEntry {
  NodeId nodeId;                    // Address announced by the HELLO sender.
  NodeRole role;                    // Sender's configured network job.
  uint32_t lastSeenMs;              // Local receive time of its newest HELLO.
  float rssiDbm;                    // Signal strength of that newest HELLO.
  float snrDb;                      // Signal quality of that newest HELLO.
  NeighborStatus status;            // ACTIVE while heartbeat is recent.
  uint32_t helloPacketsReceived;    // Running diagnostic count.
  bool occupied;                    // False means this array slot is unused.
};

class NeighborTable {
 public:
  NeighborTable();

  // observeHello adds or refreshes one direct neighbor. updateTimeouts compares
  // lastSeenMs with the configured heartbeat limit. print produces a readable
  // table without exposing internal storage to the main sketch.
  void begin(NodeId localNodeId, uint32_t timeoutMs);
  void observeHello(const HelloPayload& hello,
                    uint32_t receivedAtMs,
                    float rssiDbm,
                    float snrDb,
                    Print& log);
  void updateTimeouts(uint32_t nowMs, Print& log);
  void print(Print& log) const;
  size_t count() const;
  const NeighborEntry* find(NodeId nodeId) const;

 private:
  // Mutable lookup is used when an entry will be updated; the const public
  // lookup allows safe inspection without permitting accidental changes.
  NeighborEntry* findMutable(NodeId nodeId);
  NeighborEntry* allocate(NodeId nodeId);

  NodeId localNodeId_;                  // Used in diagnostic message prefixes.
  uint32_t timeoutMs_;                  // Maximum age of an ACTIVE heartbeat.
  NeighborEntry entries_[kMaxNeighbors];
};

}  // namespace AndyMesh
