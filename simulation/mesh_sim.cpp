#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <cstring>
#include <random>
#include <deque>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "CommandExecutor.h"
#include "MavlinkTelemetryDecoder.h"
#include "NeighborTable.h"
#include "Router.h"
#include "SimulatedRadio.h"

// Mesh timing, the same values as the firmware's NodeConfig.h.
constexpr uint32_t kHelloIntervalMs = 2000;
constexpr uint32_t kNeighborTimeoutMs = 5000;
constexpr uint32_t kTelemetryIntervalMs = 2000;
using namespace AndyMesh;

namespace {
uint32_t elapsedMs();

// Recent network events (discoveries, LOST, route changes) for the dashboard.
std::deque<std::string> gEvents;
const size_t kMaxEvents = 12;

// Prints an event and keeps the most recent kMaxEvents for the status file.
void logEvent(const char* format, ...) {
  char text[160];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof text, format, args);
  va_end(args);
  puts(text);

  char stamped[180];
  snprintf(stamped, sizeof stamped, "%6.1f s  %s", elapsedMs() / 1000.0, text);
  gEvents.push_back(stamped);
  if (gEvents.size() > kMaxEvents) gEvents.pop_front();
}

// Distance in meters between two latitude/longitude points. A flat-earth
// approximation is accurate enough over a few kilometers.
double metersBetween(double lat1, double lon1, double lat2, double lon2) {
  const double kPi = 3.14159265358979;
  double north = (lat2 - lat1) * 111320.0;
  double east = (lon2 - lon1) * 111320.0 * cos(lat1 * kPi / 180.0);
  return sqrt(north * north + east * east);
}

// Simulated signal strength from distance: closer nodes get a stronger RSSI,
// so route ties are broken in favor of the nearer neighbor.
float rssiFromDistance(double meters) {
  if (meters < 1.0) meters = 1.0;
  return static_cast<float>(-40.0 - 25.0 * log10(meters));
}

// Latest telemetry received at the Ground Station for one drone.
struct LastTelemetry {
  bool received = false;
  TelemetryData data;
  NodeId via = 0;            // Neighbor that delivered it on the last hop.
  uint32_t receivedMs = 0;
  uint32_t count = 0;        // Packets received from this drone.
};

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
  CommandExecutor executor;          // Drones: LAND -> PX4 with retries.
  uint16_t commandSeq = 0;           // Ground Station: last command sent.
  sockaddr_in px4Address{};          // Where this drone's PX4 streams from.
  bool havePx4Address = false;
  // Frames sent later: command repeats, per-hop repeats, and relay jitter.
  struct Scheduled {
    uint32_t atMs;
    PacketType type;
    NodeId dest;
    uint8_t length;
    uint8_t payload[kMaxPayloadSize];
  };
  std::vector<Scheduled> scheduled;
  uint16_t packetSeq = 0, telemetrySeq = 0;
  uint32_t nextHello = 0, nextTelemetry = 2000;
  int udp = -1;
  LastTelemetry lastTelemetry[4];  // Used by the Ground Station only.
  bool hasPosition = false;        // Position from this drone's own PX4.
  double lat = 0.0, lon = 0.0;     // Ground Station: its fixed location.
  Node(SimulatedRadioNetwork& net, NodeId n)
      : id(n),
        role(n == 0 ? NodeRole::GroundStation : NodeRole::Drone),
        radio(net, n) {
    neighbors.begin(n, kNeighborTimeoutMs);
    router.begin(n, n == 0);  // Node 0 is the Ground Station.
    decoder.begin(n, 3000);
    executor.begin(n);
    commandSeq = static_cast<uint16_t>(std::random_device{}());
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
    sockaddr_in from{};
    socklen_t fromLength = sizeof from;
    while ((n = recvfrom(udp, bytes, sizeof bytes, 0,
                         reinterpret_cast<sockaddr*>(&from), &fromLength)) > 0) {
      px4Address = from;
      havePx4Address = true;
      for (ssize_t i = 0; i < n; ++i) decoder.ingestByte(bytes[i], now);
      fromLength = sizeof from;
    }
  }
  void schedule(uint32_t at, PacketType type, NodeId dest,
                const uint8_t* payload, uint8_t length) {
    Scheduled f{at, type, dest, length, {}};
    memcpy(f.payload, payload, length);
    scheduled.push_back(f);
  }
  void sendScheduled(uint32_t now) {
    for (size_t i = 0; i < scheduled.size();) {
      if (static_cast<int32_t>(now - scheduled[i].atMs) >= 0) {
        sendPayload(scheduled[i].type, scheduled[i].dest, scheduled[i].payload,
                    scheduled[i].length, now);
        scheduled.erase(scheduled.begin() + static_cast<long>(i));
      } else {
        ++i;
      }
    }
  }
  // Ground Station: flood a LAND command, repeated three times 700 ms apart.
  void sendLand(NodeId target, uint32_t now) {
    CommandPayload c{id, target, MeshCommand::Land, ++commandSeq, kMaxHops};
    uint8_t p[kCommandPayloadSize];
    if (encodeCommandPayload(c, p, sizeof p) != ProtocolError::None) return;
    for (uint32_t i = 0; i < 3; ++i)
      schedule(now + i * 700, PacketType::Command, kBroadcastNodeId, p,
               sizeof p);
    if (target == kBroadcastNodeId)
      logEvent("[GROUND] LAND -> ALL drones (cmd %u)", c.commandSequence);
    else
      logEvent("[GROUND] LAND -> Drone %u (cmd %u)", target,
               c.commandSequence);
  }
  void handleCommand(const DecodedPacket& p, uint32_t now) {
    CommandPayload c;
    if (decodeCommandPayload(p.payload, p.header.payloadLength, c) !=
            ProtocolError::None ||
        id == kGroundStationId ||
        router.commandAlreadySeen(PacketType::Command, c.originNodeId,
                                  c.commandSequence))
      return;
    if (c.targetNodeId == id || c.targetNodeId == kBroadcastNodeId) {
      logEvent("[Node %u] LAND received via Node %u (cmd %u)", id,
               p.header.sourceNodeId, c.commandSequence);
      executor.start(c.command, c.commandSequence, now);
    }
    if (c.targetNodeId == id || c.hopsRemaining <= 1) return;
    c.hopsRemaining--;
    uint8_t out[kCommandPayloadSize];
    if (encodeCommandPayload(c, out, sizeof out) != ProtocolError::None) return;
    printf("[Node %u] RELAY LAND command\n", id);
    const uint32_t first = now + 30 + static_cast<uint32_t>(rand() % 221);
    for (uint32_t i = 0; i < 2; ++i)
      schedule(first + i * 400, PacketType::Command, kBroadcastNodeId, out,
               sizeof out);
  }
  void handleCommandAck(const DecodedPacket& p, uint32_t now) {
    CommandAckPayload a;
    if (decodeCommandAckPayload(p.payload, p.header.payloadLength, a) !=
            ProtocolError::None ||
        router.commandAlreadySeen(PacketType::CommandAck, a.droneNodeId,
                                  a.commandSequence))
      return;
    if (id == kGroundStationId) {
      logEvent("[GROUND] Drone %u LAND %s (cmd %u, via Node %u)", a.droneNodeId,
               commandResultName(a.result), a.commandSequence,
               p.header.sourceNodeId);
      return;
    }
    if (router.hasRoute())
      for (uint32_t i = 0; i < 2; ++i)
        schedule(now + i * 400, PacketType::CommandAck, router.nextHop(),
                 p.payload, p.header.payloadLength);
  }
  // Drone: write MAVLink LAND to this drone's PX4 and report its answer.
  void serviceExecutor(uint32_t now) {
    uint8_t result = 0;
    if (executor.active() && decoder.takeLandAck(result))
      executor.finish(result);
    uint8_t attempt = 0;
    if (executor.attemptDue(now, attempt)) {
      uint8_t frame[300];
      const size_t length =
          havePx4Address ? decoder.packLandCommand(attempt, now, frame,
                                                   sizeof frame)
                         : 0;
      if (length == 0 ||
          sendto(udp, frame, length, 0,
                 reinterpret_cast<const sockaddr*>(&px4Address),
                 sizeof px4Address) < 0)
        executor.finish(kCommandResultNoAutopilot);
      else
        printf("[Node %u] LAND sent to PX4 (try %u)\n", id, attempt + 1);
    }
    CommandAckPayload report;
    if (!executor.takeReport(report)) return;
    logEvent("[Node %u] LAND result: %s", id, commandResultName(report.result));
    router.commandAlreadySeen(PacketType::CommandAck, report.droneNodeId,
                              report.commandSequence);
    uint8_t p[kCommandAckPayloadSize];
    if (encodeCommandAckPayload(report, p, sizeof p) == ProtocolError::None &&
        router.hasRoute())
      for (uint32_t i = 0; i < 2; ++i)
        schedule(now + i * 400, PacketType::CommandAck, router.nextHop(), p,
                 sizeof p);
  }
  // Prints the current route, e.g. "[Node 3] ROUTE -> via Node 2 (2 hops ...)".
  void printRoute() {
    if (router.hasRoute()) {
      logEvent("[Node %u] ROUTE -> via Node %u (%u hops to ground)", id,
               router.nextHop(), router.hopsToGround());
    } else {
      logEvent("[Node %u] ROUTE -> NONE (no path to ground)", id);
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

      // Stored for the dashboard status file.
      if (t.originatingNodeId < 4) {
        LastTelemetry& last = lastTelemetry[t.originatingNodeId];
        last.received = true;
        last.data = t;
        last.via = p.header.sourceNodeId;
        last.receivedMs = now;
        last.count++;
      }
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

    // Current position from this drone's PX4, used for distance-based links
    // and the dashboard map.
    TelemetryData own;
    if (id != 0 && decoder.read(now, own) &&
        (own.validityFlags & TelemetryValidPosition) != 0) {
      lat = own.latitudeE7 / 1e7;
      lon = own.longitudeE7 / 1e7;
      hasPosition = true;
    }
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
            logEvent(
                "[Node %u] Node %u %s", id, h.nodeId,
                e == NeighborEvent::Discovered ? "discovered" : "ACTIVE again");
        }
      } else if (p.header.packetType == PacketType::Telemetry) {
        handleTelemetry(p, now);
      } else if (p.header.packetType == PacketType::Command) {
        handleCommand(p, now);
      } else if (p.header.packetType == PacketType::CommandAck) {
        handleCommandAck(p, now);
      }
    }
    if (id != kGroundStationId) serviceExecutor(now);
    sendScheduled(now);
    NodeId lost[kMaxNeighbors];
    size_t count = neighbors.updateTimeouts(now, lost, kMaxNeighbors);
    for (size_t i = 0; i < count && i < kMaxNeighbors; ++i)
      logEvent("[Node %u] Node %u LOST (heartbeat timeout)", id, lost[i]);

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
      nextHello = now + kHelloIntervalMs;
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
      nextTelemetry = now + kTelemetryIntervalMs;
    }
  }
};
// Writes the network state as JSON for the dashboard. The file is written to a
// temporary name first and then renamed, so the browser never reads a
// half-written file.
void writeStatus(const char* path, Node* nodes[], int nodeCount,
                 const std::vector<std::pair<int, int>>& links,
                 const char* topology, double rangeM, uint32_t now) {
  std::string tmpPath = std::string(path) + ".tmp";
  FILE* f = fopen(tmpPath.c_str(), "w");
  if (f == nullptr) return;

  fprintf(f, "{\n  \"time_s\": %.1f,\n", now / 1000.0);
  fprintf(f, "  \"topology\": \"%s\",\n", topology);
  fprintf(f, "  \"range_m\": %.0f,\n", rangeM);

  // Each node: radio on/off and its current route.
  fprintf(f, "  \"nodes\": [\n");
  for (int i = 0; i < nodeCount; i++) {
    Node* n = nodes[i];
    fprintf(f,
            "    {\"id\": %u, \"radio_on\": %s, \"hops\": %u, \"via\": %u, "
            "\"has_pos\": %s, \"lat\": %.7f, \"lon\": %.7f}%s\n",
            n->id, n->radio.readyToSend() ? "true" : "false",
            n->router.hopsToGround(), n->router.nextHop(),
            n->hasPosition ? "true" : "false", n->lat, n->lon,
            i + 1 < nodeCount ? "," : "");
  }
  fprintf(f, "  ],\n");

  // A link is up only when both ends currently list each other as ACTIVE.
  fprintf(f, "  \"links\": [\n");
  for (size_t i = 0; i < links.size(); i++) {
    int a = links[i].first;
    int b = links[i].second;
    const NeighborEntry* ab = nodes[a]->neighbors.find(b);
    const NeighborEntry* ba = nodes[b]->neighbors.find(a);
    bool up = ab != nullptr && ab->status == NeighborStatus::Active &&
              ba != nullptr && ba->status == NeighborStatus::Active;
    double distance = -1.0;  // -1 = unknown (fixed layouts)
    if (nodes[a]->hasPosition && nodes[b]->hasPosition) {
      distance = metersBetween(nodes[a]->lat, nodes[a]->lon, nodes[b]->lat,
                               nodes[b]->lon);
    }
    fprintf(f, "    {\"a\": %d, \"b\": %d, \"up\": %s, \"distance_m\": %.0f}%s\n",
            a, b, up ? "true" : "false", distance,
            i + 1 < links.size() ? "," : "");
  }
  fprintf(f, "  ],\n");

  // Latest telemetry at the Ground Station, one entry per drone heard.
  fprintf(f, "  \"telemetry\": [");
  bool first = true;
  for (int d = 1; d < nodeCount; d++) {
    const LastTelemetry& last = nodes[0]->lastTelemetry[d];
    if (!last.received) continue;
    const TelemetryData& t = last.data;
    fprintf(f,
            "%s\n    {\"drone\": %d, \"lat\": %.7f, \"lon\": %.7f, "
            "\"alt_m\": %.1f, \"battery_pct\": %u, \"battery_v\": %.2f, "
            "\"via\": %u, \"count\": %u, \"age_s\": %.1f}",
            first ? "" : ",", d, t.latitudeE7 / 1e7, t.longitudeE7 / 1e7,
            t.altitudeCm / 100.0, t.batteryPercent, t.batteryMillivolts / 1000.0,
            last.via, last.count, (now - last.receivedMs) / 1000.0);
    first = false;
  }
  fprintf(f, "\n  ],\n");

  // Recent events, oldest first.
  fprintf(f, "  \"events\": [");
  for (size_t i = 0; i < gEvents.size(); i++) {
    fprintf(f, "%s\n    \"%s\"", i == 0 ? "" : ",", gEvents[i].c_str());
  }
  fprintf(f, "\n  ]\n}\n");
  fclose(f);
  rename(tmpPath.c_str(), path);
}

// Keyboard, one line at a time:
//   1-3        turn that node's radio off or on
//   LAND <n>   Ground Station floods LAND to drone n
//   LAND ALL   Ground Station floods LAND to every drone
// select() with a zero timeout checks for input without waiting.
std::string gKeyboardLine;
void runKeyboardLine(const std::string& line, Node* nodes[], uint32_t now) {
  std::string upper;
  for (char c : line) upper += static_cast<char>(toupper(static_cast<unsigned char>(c)));
  if (upper.compare(0, 4, "LAND") == 0) {
    char target[16] = {};
    if (sscanf(upper.c_str() + 4, " %15s", target) != 1) {
      puts("Usage: LAND <drone id> | LAND ALL");
    } else if (strcmp(target, "ALL") == 0) {
      nodes[0]->sendLand(kBroadcastNodeId, now);
    } else {
      int droneId = atoi(target);
      if (droneId >= 1 && droneId <= 3)
        nodes[0]->sendLand(static_cast<NodeId>(droneId), now);
      else
        puts("Usage: LAND <drone id 1-3> | LAND ALL");
    }
    return;
  }
  for (char c : upper) {
    int id = c - '0';
    if (id >= 1 && id <= 3) {
      bool on = nodes[id]->radio.readyToSend();
      nodes[id]->radio.setConnected(!on);
      logEvent("[SCENARIO] Node %d radio %s", id, on ? "OFF" : "ON");
    }
  }
}
void checkKeyboard(Node* nodes[], uint32_t now) {
  fd_set input;
  FD_ZERO(&input);
  FD_SET(STDIN_FILENO, &input);
  timeval noWait{0, 0};
  if (select(STDIN_FILENO + 1, &input, nullptr, nullptr, &noWait) <= 0) {
    return;
  }
  char buffer[64];
  ssize_t n = read(STDIN_FILENO, buffer, sizeof buffer);
  for (ssize_t i = 0; i < n; i++) {
    if (buffer[i] == '\n' || buffer[i] == '\r') {
      if (!gKeyboardLine.empty()) runKeyboardLine(gKeyboardLine, nodes, now);
      gKeyboardLine.clear();
    } else if (gKeyboardLine.size() < 32) {
      gKeyboardLine += buffer[i];
    }
  }
}
}  // namespace
int main(int argc, char** argv) {
  uint32_t duration = 0;
  int disconnected = -1, disableA = -1, disableB = -1;
  bool failureDemo = false;
  bool diamond = false;         // --topology diamond
  bool distanceMode = false;    // --topology distance
  double rangeM = 60.0;         // --range-m: simulated radio range
  double gsLat = 0.0, gsLon = 0.0;
  bool gsGiven = false;         // --gs-lat/--gs-lon; else Drone 1's first fix
  const char* statusPath = nullptr;  // --status FILE
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
    else if (strcmp(argv[i], "--topology") == 0 && i + 1 < argc) {
      const char* name = argv[++i];
      diamond = strcmp(name, "diamond") == 0;
      distanceMode = strcmp(name, "distance") == 0;
    } else if (strcmp(argv[i], "--range-m") == 0 && i + 1 < argc)
      rangeM = atof(argv[++i]);
    else if (strcmp(argv[i], "--gs-lat") == 0 && i + 1 < argc) {
      gsLat = atof(argv[++i]);
      gsGiven = true;
    } else if (strcmp(argv[i], "--gs-lon") == 0 && i + 1 < argc)
      gsLon = atof(argv[++i]);
    else if (strcmp(argv[i], "--status") == 0 && i + 1 < argc)
      statusPath = argv[++i];
  }
  if (loss > 100) loss = 100;
  SimulatedRadioNetwork network;
  // Link format: {nodeA, nodeB, enabled, RSSI dBm, SNR dB, latency ms, loss %}
  const uint8_t lossPct = static_cast<uint8_t>(loss);
  std::vector<std::pair<int, int>> links;  // Same pairs as the radio links.
  if (distanceMode) {
    // Distance: every pair has a possible link. A link is enabled only while
    // the two nodes are within rangeM of each other (updated in the loop).
    for (int a = 0; a < 4; a++) {
      for (int b = a + 1; b < 4; b++) {
        network.addLink({static_cast<NodeId>(a), static_cast<NodeId>(b), false,
                         -100, 5, 20, lossPct});
        links.push_back({a, b});
      }
    }
  } else if (diamond) {
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
    links = {{0, 1}, {0, 2}, {1, 3}, {2, 3}};
  } else {
    // Line: Ground - Drone 1 - Drone 2 - Drone 3. No backup path.
    network.addLink({0, 1, true, -48, 9, 20, lossPct});
    network.addLink({1, 2, true, -63, 5, 35, lossPct});
    network.addLink({2, 3, true, -72, 2, 50, lossPct});
    links = {{0, 1}, {1, 2}, {2, 3}};
  }
  if (disableA >= 0) network.setLink(disableA, disableB, false);
  Node n0(network, 0), n1(network, 1), n2(network, 2), n3(network, 3);
  Node* nodes[] = {&n0, &n1, &n2, &n3};
  if (disconnected >= 0 && disconnected < 4)
    nodes[disconnected]->radio.setConnected(false);
  const char* topologyName = distanceMode ? "distance"
                             : diamond    ? "diamond"
                                          : "line";
  if (distanceMode) {
    printf("ANDY 8 mesh simulation: distance-based links, range %.0f m\n",
           rangeM);
  } else if (diamond) {
    puts("ANDY 8 mesh simulation: diamond topology 0-1, 0-2, 1-3, 2-3");
  } else {
    puts("ANDY 8 mesh simulation: line topology 0 <-> 1 <-> 2 <-> 3");
  }
  uint32_t nextStatusMs = 0;
  uint32_t nextLinkUpdateMs = 0;
  if (distanceMode && gsGiven) {
    n0.lat = gsLat;
    n0.lon = gsLon;
    n0.hasPosition = true;
  }
  bool failed = false, recovered = false;
  while (!duration || elapsedMs() < duration) {
    uint32_t now = elapsedMs();
    if (failureDemo && !failed && now >= 5000) {
      n2.radio.setConnected(false);
      failed = true;
      logEvent("[SCENARIO] Node 2 disconnected");
    }
    if (failureDemo && !recovered && now >= 12000) {
      n2.radio.setConnected(true);
      recovered = true;
      logEvent("[SCENARIO] Node 2 restored");
    }
    // Distance-based links, recomputed five times per second.
    if (distanceMode && static_cast<int32_t>(now - nextLinkUpdateMs) >= 0) {
      // Ground Station location defaults to Drone 1's first position fix
      // (the launch point).
      if (!n0.hasPosition && n1.hasPosition) {
        n0.lat = n1.lat;
        n0.lon = n1.lon;
        n0.hasPosition = true;
        logEvent("[SCENARIO] Ground Station placed at %.7f, %.7f", n0.lat,
                 n0.lon);
      }
      for (const auto& link : links) {
        Node* a = nodes[link.first];
        Node* b = nodes[link.second];
        if (!a->hasPosition || !b->hasPosition) continue;
        double meters = metersBetween(a->lat, a->lon, b->lat, b->lon);
        network.updateLink(a->id, b->id, meters <= rangeM,
                           rssiFromDistance(meters));
      }
      nextLinkUpdateMs = now + 200;
    }

    network.setNow(now);
    for (auto* n : nodes) n->tick(now);
    checkKeyboard(nodes, now);

    // Status file refreshed twice per second.
    if (statusPath != nullptr && static_cast<int32_t>(now - nextStatusMs) >= 0) {
      writeStatus(statusPath, nodes, 4, links, topologyName,
                  distanceMode ? rangeM : 0.0, now);
      nextStatusMs = now + 500;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}
