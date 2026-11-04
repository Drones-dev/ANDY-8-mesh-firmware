#include "NeighborTable.h"

namespace AndyMesh {

NeighborTable::NeighborTable() : localNodeId_(0), timeoutMs_(3000) {
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

NeighborEvent NeighborTable::observeHello(const HelloPayload& hello,
                                          uint32_t receivedAtMs,
                                          float rssiDbm,
                                          float snrDb) {
  // A radio should not add itself as its own neighbor, even if an echoed packet
  // is received during unusual bench wiring or testing.
  if (hello.nodeId == localNodeId_) {
    return NeighborEvent::None;
  }

  NeighborEntry* entry = findMutable(hello.nodeId);
  const bool isNew = entry == nullptr;
  if (isNew) {
    // The bounded table fails safely when full: report the condition and keep
    // existing entries rather than overwriting an arbitrary known neighbor.
    entry = allocate(hello.nodeId);
    if (entry == nullptr) {
      return NeighborEvent::TableFull;
    }
    entry->helloPacketsReceived = 0;
    entry->status = NeighborStatus::Lost;
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

  return isNew ? NeighborEvent::Discovered
               : (recovered ? NeighborEvent::Recovered : NeighborEvent::None);
}

size_t NeighborTable::updateTimeouts(uint32_t nowMs,
                                     NodeId* lostNodes,
                                     size_t capacity) {
  size_t lostCount = 0;
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
    if (lostNodes != nullptr && lostCount < capacity) {
      lostNodes[lostCount] = entry.nodeId;
    }
    ++lostCount;
  }
  return lostCount;
}

const NeighborEntry* NeighborTable::entryAt(size_t index) const {
  return index < kMaxNeighbors && entries_[index].occupied ? &entries_[index]
                                                            : nullptr;
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
