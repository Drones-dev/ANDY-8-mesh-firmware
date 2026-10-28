#include "NeighborTable.h"

namespace AndyMesh {

NeighborTable::NeighborTable() : localNodeId_(0), timeoutMs_(5000) {
  // Clear every fixed slot at startup. LOST is a harmless initial status because
  // occupied=false means the slot is not yet a real neighbor entry.
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    entries_[i] = NeighborEntry{0,
                                NodeRole::Drone,
                                0,
                                0.0F,
                                0.0F,
                                NeighborStatus::Lost,
                                0,
                                false};
  }
}

void NeighborTable::begin(NodeId localNodeId, uint32_t timeoutMs) {
  // Store the local identity and central timeout setting once. The table uses
  // the receiver's millis() clock rather than trusting a sender's uptime.
  localNodeId_ = localNodeId;
  timeoutMs_ = timeoutMs;
}

NeighborEntry* NeighborTable::findMutable(NodeId nodeId) {
  // A linear search is simple and efficient for a maximum of eight entries.
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    if (entries_[i].occupied && entries_[i].nodeId == nodeId) {
      return &entries_[i];
    }
  }
  return nullptr;
}

const NeighborEntry* NeighborTable::find(NodeId nodeId) const {
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    if (entries_[i].occupied && entries_[i].nodeId == nodeId) {
      return &entries_[i];
    }
  }
  return nullptr;
}

NeighborEntry* NeighborTable::allocate(NodeId nodeId) {
  // Reuse the first empty fixed slot. LOST entries are intentionally preserved
  // so their signal history remains available and they can recover to ACTIVE.
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    if (!entries_[i].occupied) {
      entries_[i].occupied = true;
      entries_[i].nodeId = nodeId;
      return &entries_[i];
    }
  }
  return nullptr;
}

void NeighborTable::observeHello(const HelloPayload& hello,
                                 uint32_t receivedAtMs,
                                 float rssiDbm,
                                 float snrDb,
                                 Print& log) {
  // A radio should not add itself as its own neighbor, even if an echoed packet
  // is received during unusual bench wiring or testing.
  if (hello.nodeId == localNodeId_) {
    return;
  }

  NeighborEntry* entry = findMutable(hello.nodeId);
  const bool isNew = entry == nullptr;
  if (isNew) {
    // The bounded table fails safely when full: report the condition and keep
    // existing entries rather than overwriting an arbitrary known neighbor.
    entry = allocate(hello.nodeId);
    if (entry == nullptr) {
      log.print("[Node ");
      log.print(localNodeId_);
      log.println("] Neighbor table full; HELLO ignored");
      return;
    }
    entry->helloPacketsReceived = 0;
    entry->status = NeighborStatus::Lost;
    log.print("[Node ");
    log.print(localNodeId_);
    log.print("] Discovered Node ");
    log.println(hello.nodeId);
  }

  // Remember the old status before refreshing it so recovery can be logged.
  const bool recovered = !isNew && entry->status == NeighborStatus::Lost;
  entry->role = hello.role;
  // receivedAtMs is local time. RSSI/SNR belong to this direct over-the-air
  // reception, so they are valid evidence about the sender's direct link.
  entry->lastSeenMs = receivedAtMs;
  entry->rssiDbm = rssiDbm;
  entry->snrDb = snrDb;
  entry->status = NeighborStatus::Active;
  ++entry->helloPacketsReceived;

  log.print("[Node ");
  log.print(localNodeId_);
  log.print("] HELLO received from Node ");
  log.println(hello.nodeId);
  log.print("[Node ");
  log.print(localNodeId_);
  log.print("] Neighbor ");
  log.print(hello.nodeId);
  log.print(" RSSI: ");
  log.print(rssiDbm, 1);
  log.print(" dBm, SNR: ");
  log.print(snrDb, 1);
  log.println(" dB");

  if (recovered) {
    log.print("[Node ");
    log.print(localNodeId_);
    log.print("] Node ");
    log.print(hello.nodeId);
    log.println(" is ACTIVE again");
  }
}

void NeighborTable::updateTimeouts(uint32_t nowMs, Print& log) {
  // This runs every main-loop pass but prints only on an ACTIVE -> LOST change.
  // Keeping LOST entries prevents repeated warnings and preserves diagnostics.
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    NeighborEntry& entry = entries_[i];
    if (!entry.occupied || entry.status != NeighborStatus::Active) {
      continue;
    }
    // Unsigned subtraction remains correct across millis() wraparound.
    if (static_cast<uint32_t>(nowMs - entry.lastSeenMs) < timeoutMs_) {
      continue;
    }
    entry.status = NeighborStatus::Lost;
    log.print("[Node ");
    log.print(localNodeId_);
    log.print("] WARNING: Node ");
    log.print(entry.nodeId);
    log.println(" heartbeat timeout");
    log.print("[Node ");
    log.print(localNodeId_);
    log.print("] Node ");
    log.print(entry.nodeId);
    log.println(" marked LOST");
  }
}

void NeighborTable::print(Print& log) const {
  // The table is formatted for a 115200-baud Serial Monitor. HELLO count helps
  // distinguish a stable link from a newly discovered or intermittent one.
  log.print("[Node ");
  log.print(localNodeId_);
  log.println("] Neighbor Table");
  log.println("ID | Role    | RSSI   | SNR  | Status | HELLOs");
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    const NeighborEntry& entry = entries_[i];
    if (!entry.occupied) {
      continue;
    }
    log.print(entry.nodeId);
    log.print("  | ");
    log.print(nodeRoleName(entry.role));
    log.print(" | ");
    log.print(entry.rssiDbm, 1);
    log.print(" | ");
    log.print(entry.snrDb, 1);
    log.print(" | ");
    log.print(entry.status == NeighborStatus::Active ? "ACTIVE" : "LOST");
    log.print(" | ");
    log.println(entry.helloPacketsReceived);
  }
}

size_t NeighborTable::count() const {
  // Count both ACTIVE and LOST occupied records; both contain useful history.
  size_t total = 0;
  for (size_t i = 0; i < kMaxNeighbors; ++i) {
    if (entries_[i].occupied) {
      ++total;
    }
  }
  return total;
}

}  // namespace AndyMesh
