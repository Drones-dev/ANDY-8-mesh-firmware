#pragma once
#include <deque>
#include <vector>
#include "PacketTransport.h"
namespace AndyMesh {
// A simulated link is deliberately simple and deterministic. RSSI/SNR are
// metadata reported to the neighbor table; latency delays delivery; loss is a
// percentage chance of dropping an entire packet. This is not a propagation or
// LoRa-airtime model, so its values must never be presented as range evidence.
struct SimulatedLink {
  NodeId a, b;
  bool enabled;
  float rssiDbm, snrDb;
  uint32_t latencyMs;
  uint8_t lossPercent;
};
class SimulatedRadioNetwork;
class SimulatedRadioEndpoint : public PacketTransport {
 public:
  SimulatedRadioEndpoint(SimulatedRadioNetwork& network, NodeId id);
  bool poll(TransportPacket& packet) override;
  bool send(const uint8_t* data, size_t length) override;
  bool readyToSend() const override { return connected_; }
  void setConnected(bool value) { connected_ = value; }
  NodeId id() const { return id_; }

 private:
  friend class SimulatedRadioNetwork;
  // Pending packets wait until the fake clock reaches deliverAtMs. Tests can
  // move that clock instantly, avoiding slow real-time sleeps.
  struct Pending {
    TransportPacket packet;
    uint32_t deliverAtMs;
  };
  SimulatedRadioNetwork& network_;
  NodeId id_;
  bool connected_;
  std::deque<Pending> inbox_;
};
class SimulatedRadioNetwork {
 public:
  explicit SimulatedRadioNetwork(uint32_t seed = 8);
  void setNow(uint32_t now) { nowMs_ = now; }
  uint32_t now() const { return nowMs_; }
  void addEndpoint(SimulatedRadioEndpoint* endpoint);
  void addLink(const SimulatedLink& link);
  bool setLink(NodeId a, NodeId b, bool enabled);
  // Changes a link's state and signal strength (used by distance-based links).
  bool updateLink(NodeId a, NodeId b, bool enabled, float rssiDbm);
  void transmit(NodeId source, const uint8_t* data, size_t length);

 private:
  uint32_t random100();
  uint32_t nowMs_, randomState_;
  std::vector<SimulatedRadioEndpoint*> endpoints_;
  std::vector<SimulatedLink> links_;
};
}  // namespace AndyMesh
