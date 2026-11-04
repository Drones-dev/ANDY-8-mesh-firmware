#include "SimulatedRadio.h"
#include <string.h>
namespace AndyMesh {
SimulatedRadioEndpoint::SimulatedRadioEndpoint(SimulatedRadioNetwork& n,
                                               NodeId id)
    : network_(n), id_(id), connected_(true) {
  n.addEndpoint(this);
}
bool SimulatedRadioEndpoint::poll(TransportPacket& out) {
  if (!connected_ || inbox_.empty() ||
      static_cast<int32_t>(network_.now() - inbox_.front().deliverAtMs) < 0)
    return false;
  out = inbox_.front().packet;
  inbox_.pop_front();
  return true;
}
bool SimulatedRadioEndpoint::send(const uint8_t* data, size_t length) {
  if (!connected_ || data == nullptr || length == 0 || length > kMaxPacketSize)
    return false;
  network_.transmit(id_, data, length);
  return true;
}
SimulatedRadioNetwork::SimulatedRadioNetwork(uint32_t seed)
    : nowMs_(0), randomState_(seed ? seed : 8) {}
void SimulatedRadioNetwork::addEndpoint(SimulatedRadioEndpoint* e) {
  endpoints_.push_back(e);
}
void SimulatedRadioNetwork::addLink(const SimulatedLink& l) {
  links_.push_back(l);
}
bool SimulatedRadioNetwork::setLink(NodeId a, NodeId b, bool enabled) {
  for (auto& l : links_)
    if ((l.a == a && l.b == b) || (l.a == b && l.b == a)) {
      l.enabled = enabled;
      return true;
    }
  return false;
}
bool SimulatedRadioNetwork::updateLink(NodeId a, NodeId b, bool enabled,
                                      float rssiDbm) {
  for (auto& l : links_) {
    if ((l.a == a && l.b == b) || (l.a == b && l.b == a)) {
      l.enabled = enabled;
      l.rssiDbm = rssiDbm;
      return true;
    }
  }
  return false;
}
uint32_t SimulatedRadioNetwork::random100() {
  randomState_ = randomState_ * 1664525U + 1013904223U;
  return (randomState_ >> 16) % 100U;
}
void SimulatedRadioNetwork::transmit(NodeId source, const uint8_t* data,
                                     size_t length) {
  for (const auto& l : links_) {
    if (!l.enabled || random100() < l.lossPercent) continue;
    NodeId target =
        l.a == source ? l.b : (l.b == source ? l.a : kBroadcastNodeId);
    if (target == kBroadcastNodeId) continue;
    for (auto* e : endpoints_)
      if (e->id_ == target && e->connected_) {
        SimulatedRadioEndpoint::Pending p{};
        memcpy(p.packet.data, data, length);
        p.packet.length = length;
        p.packet.rssiDbm = l.rssiDbm;
        p.packet.snrDb = l.snrDb;
        p.deliverAtMs = nowMs_ + l.latencyMs;
        e->inbox_.push_back(p);
      }
  }
}
}  // namespace AndyMesh
