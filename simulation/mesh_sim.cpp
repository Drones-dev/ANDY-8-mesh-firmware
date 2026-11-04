#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include "MavlinkTelemetryDecoder.h"
#include "NeighborTable.h"
#include "Router.h"
#include "SimulatedRadio.h"
using namespace AndyMesh;

namespace {
uint32_t elapsedMs() {
  static const auto start = std::chrono::steady_clock::now();
  return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start)
          .count());
}
struct Node {
  NodeId id;
  NodeRole role;
  SimulatedRadioEndpoint radio;
  NeighborTable neighbors;
  Router router;
  MavlinkTelemetryDecoder decoder;
  uint16_t packetSeq = 0, telemetrySeq = 0;
  uint32_t nextHello = 0, nextTelemetry = 2000;
  int udp = -1;
  Node(SimulatedRadioNetwork& net, NodeId n)
      : id(n),
        role(n == 0 ? NodeRole::GroundStation : NodeRole::Drone),
        radio(net, n) {
    neighbors.begin(n, 3000);
    router.begin(n, n == 0);  // Node 0 is the Ground Station.
    decoder.begin(n, 3000);
    if (n) {
      udp = socket(AF_INET, SOCK_DGRAM, 0);
      sockaddr_in a{};
      a.sin_family = AF_INET;
      a.sin_addr.s_addr = htonl(INADDR_ANY);
      a.sin_port = htons(14539 + n);
      if (bind(udp, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0) {
        perror("MAVLink UDP bind");
      }
      fcntl(udp, F_SETFL, O_NONBLOCK);
      printf("[Node %u] MAVLink UDP :%u\n", id, 14539 + n);
    }
  }
  ~Node() {
    if (udp >= 0) close(udp);
  }
  void sendPayload(PacketType type, NodeId dest, const uint8_t* payload,
                   uint8_t length, uint32_t now) {
    uint8_t packet[kMaxPacketSize];
    size_t total = 0;
    PacketHeader h{kProtocolVersion, type, id, dest, ++packetSeq, now, length};
    if (encodePacket(h, payload, packet, sizeof packet, total) ==
        ProtocolError::None)
      radio.send(packet, total);
  }
  void updateMavlink(uint32_t now) {
    if (udp < 0) return;
    uint8_t bytes[2048];
    ssize_t n;
    while ((n = recv(udp, bytes, sizeof bytes, 0)) > 0)
      for (ssize_t i = 0; i < n; ++i) decoder.ingestByte(bytes[i], now);
  }
  // Prints the current route, e.g. "[Node 3] ROUTE -> via Node 2 (2 hops ...)".
  void printRoute() {
    if (router.hasRoute()) {
      printf("[Node %u] ROUTE -> via Node %u (%u hops to ground)\n", id,
             router.nextHop(), router.hopsToGround());
    } else {
      printf("[Node %u] ROUTE -> NONE (no path to ground)\n", id);
    }
  }

  // Telemetry addressed to this node: the Ground Station prints it, any other
  // node relays it to its own next hop.
  void handleTelemetry(const DecodedPacket& p, uint32_t now) {
    TelemetryData t;
    if (decodeTelemetryPayload(p.payload, p.header.payloadLength, t) !=
        ProtocolError::None) {
      return;
    }

    // Duplicate filter: each packet is relayed or recorded once.
    if (router.alreadySeen(t.originatingNodeId, t.sequenceNumber,
                           t.timestampMs)) {
      return;
    }

    if (id == kGroundStationId) {
      // Origin drone and last-hop relay are both shown.
      printf(
          "[GROUND] Drone %u telemetry (via Node %u): %.7f, %.7f, heading "
          "%.2f, GPS %s\n",
          t.originatingNodeId, p.header.sourceNodeId, t.latitudeE7 / 1e7,
          t.longitudeE7 / 1e7, t.headingCentidegrees / 100.0,
          gpsFixName(t.gpsFixStatus));
      return;
    }

    // Relay: payload forwarded unchanged.
    if (router.hasRoute()) {
      printf("[Node %u] RELAY Drone %u telemetry -> Node %u\n", id,
             t.originatingNodeId, router.nextHop());
      sendPayload(PacketType::Telemetry, router.nextHop(), p.payload,
                  p.header.payloadLength, now);
    } else {
      printf("[Node %u] Dropped Drone %u telemetry - no route\n", id,
             t.originatingNodeId);
    }
  }

  void tick(uint32_t now) {
    updateMavlink(now);
    TransportPacket received;
    while (radio.poll(received)) {
      DecodedPacket p;
      if (decodePacket(received.data, received.length, p) !=
              ProtocolError::None ||
          p.header.sourceNodeId == id ||
          (p.header.destinationNodeId != id &&
           p.header.destinationNodeId != kBroadcastNodeId))
        continue;
      if (p.header.packetType == PacketType::Hello) {
        HelloPayload h;
        if (decodeHelloPayload(p.payload, p.header.payloadLength, h) ==
            ProtocolError::None) {
          NeighborEvent e =
              neighbors.observeHello(h, now, received.rssiDbm, received.snrDb);
          if (e == NeighborEvent::Discovered || e == NeighborEvent::Recovered)
            printf(
                "[Node %u] Node %u %s\n", id, h.nodeId,
                e == NeighborEvent::Discovered ? "discovered" : "ACTIVE again");
        }
      } else if (p.header.packetType == PacketType::Telemetry) {
        handleTelemetry(p, now);
      }
    }
    NodeId lost[kMaxNeighbors];
    size_t count = neighbors.updateTimeouts(now, lost, kMaxNeighbors);
    for (size_t i = 0; i < count && i < kMaxNeighbors; ++i)
      printf("[Node %u] Node %u LOST (heartbeat timeout)\n", id, lost[i]);

    // Route re-evaluated after timeouts so a LOST next hop is replaced.
    if (router.update(neighbors)) {
      printRoute();
    }
    if (static_cast<int32_t>(now - nextHello) >= 0) {
      // HELLO advertises this node's route to its neighbors.
      HelloPayload h{id, role, kProtocolVersion, now, router.hopsToGround(),
                     router.nextHop()};
      uint8_t p[kHelloPayloadSize];
      encodeHelloPayload(h, p, sizeof p);
      sendPayload(PacketType::Hello, kBroadcastNodeId, p, sizeof p, now);
      nextHello = now + 1000;
    }
    if (id && static_cast<int32_t>(now - nextTelemetry) >= 0) {
      TelemetryData t;
      if (decoder.read(now, t)) {
        t.sequenceNumber = ++telemetrySeq;
        t.timestampMs = now;
        uint8_t p[kTelemetryPayloadSize];
        if (encodeTelemetryPayload(t, p, sizeof p) == ProtocolError::None) {
          // Own packet recorded so an echoed copy is not relayed.
          router.alreadySeen(id, t.sequenceNumber, t.timestampMs);

          if (router.hasRoute()) {
            // Destination is the next hop; relays forward it until node 0.
            sendPayload(PacketType::Telemetry, router.nextHop(), p, sizeof p,
                        now);
          } else {
            printf("[Node %u] No route to Ground Station - telemetry not sent\n",
                   id);
          }
        }
      }
      nextTelemetry = now + 2000;
    }
  }
};
}  // namespace
int main(int argc, char** argv) {
  uint32_t duration = 0;
  int disconnected = -1, disableA = -1, disableB = -1;
  bool failureDemo = false;
  bool diamond = false;  // --topology diamond
  unsigned loss = 0;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--duration-ms") == 0 && i + 1 < argc)
      duration = static_cast<uint32_t>(strtoul(argv[++i], nullptr, 10));
    else if (strcmp(argv[i], "--loss-percent") == 0 && i + 1 < argc)
      loss = static_cast<unsigned>(strtoul(argv[++i], nullptr, 10));
    else if (strcmp(argv[i], "--disconnect-node") == 0 && i + 1 < argc)
      disconnected = atoi(argv[++i]);
    else if (strcmp(argv[i], "--disable-link") == 0 && i + 2 < argc) {
      disableA = atoi(argv[++i]);
      disableB = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--failure-demo") == 0)
      failureDemo = true;
    else if (strcmp(argv[i], "--topology") == 0 && i + 1 < argc)
      diamond = strcmp(argv[++i], "diamond") == 0;
  }
  if (loss > 100) loss = 100;
  SimulatedRadioNetwork network;
  // Link format: {nodeA, nodeB, enabled, RSSI dBm, SNR dB, latency ms, loss %}
  const uint8_t lossPct = static_cast<uint8_t>(loss);
  if (diamond) {
    // Diamond: Drone 3 has no direct link to the Ground Station and two
    // possible relays. Drone 2 is preferred (stronger RSSI); Drone 1 is the
    // backup path used when Drone 2 is lost.
    //
    //   Ground ------ Drone 1 ------ Drone 3
    //     |                            |
    //     +-------- Drone 2 -----------+
    network.addLink({0, 1, true, -50, 9, 20, lossPct});
    network.addLink({0, 2, true, -55, 8, 20, lossPct});
    network.addLink({1, 3, true, -78, 2, 40, lossPct});
    network.addLink({2, 3, true, -62, 5, 35, lossPct});
  } else {
    // Line: Ground - Drone 1 - Drone 2 - Drone 3. No backup path.
    network.addLink({0, 1, true, -48, 9, 20, lossPct});
    network.addLink({1, 2, true, -63, 5, 35, lossPct});
    network.addLink({2, 3, true, -72, 2, 50, lossPct});
  }
  if (disableA >= 0) network.setLink(disableA, disableB, false);
  Node n0(network, 0), n1(network, 1), n2(network, 2), n3(network, 3);
  Node* nodes[] = {&n0, &n1, &n2, &n3};
  if (disconnected >= 0 && disconnected < 4)
    nodes[disconnected]->radio.setConnected(false);
  if (diamond) {
    puts("ANDY 8 mesh simulation: diamond topology 0-1, 0-2, 1-3, 2-3");
  } else {
    puts("ANDY 8 mesh simulation: line topology 0 <-> 1 <-> 2 <-> 3");
  }
  bool failed = false, recovered = false;
  while (!duration || elapsedMs() < duration) {
    uint32_t now = elapsedMs();
    if (failureDemo && !failed && now >= 5000) {
      n2.radio.setConnected(false);
      failed = true;
      puts("[SCENARIO] Node 2 disconnected");
    }
    if (failureDemo && !recovered && now >= 12000) {
      n2.radio.setConnected(true);
      recovered = true;
      puts("[SCENARIO] Node 2 restored");
    }
    network.setNow(now);
    for (auto* n : nodes) n->tick(now);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
