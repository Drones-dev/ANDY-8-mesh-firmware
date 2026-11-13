#!/usr/bin/env python3
"""
ANDY 8: RED - three-drone flight scenario for the PX4 simulation.

Flies the three PX4 drones through fixed formations so the distance-based
mesh (andy_mesh_sim --topology distance) has to change its routes:

  1. LINE      Drones 1, 2, 3 in a line heading east. Drone 3 is out of the
               Ground Station's range and relays through Drone 2 and Drone 1.
  2. OVERTAKE  Drone 2 flies ahead of Drone 3, so Drone 2 now relays through
               Drone 3.
  3. DIAMOND   Drones 1 and 2 near the Ground Station, Drone 3 beyond them.
               Drone 3 has two possible relays, so a relay failure (radio
               turned off in andy_mesh_sim) causes a reroute.
  4. LAND      All drones land where they are.

Positions are meters east/north of Drone 1's home (the Ground Station
location used by andy_mesh_sim). The formations are sized for a 60 m
simulated radio range.

Connection: PX4 sends its ground-control link to UDP 14550. Commands go back
to each drone's own port (18570 + instance). The mesh simulator uses ports
14540-14542, so both programs run at the same time.
"""

import math
import time

from pymavlink import mavutil

NUM_DRONES = 3
GCS_LISTEN = "udpin:0.0.0.0:14550"
DRONE_PORT_BASE = 18570  # instance 0 = 18570, 1 = 18571, 2 = 18572

# Flight altitude per drone (meters above home). Different altitudes keep the
# drones apart vertically when their paths cross.
ALTITUDE_M = [15.0, 20.0, 25.0]

# Formations: (east m, north m) for Drone 1, 2, 3, and how long to hold there.
FORMATIONS = [
    ("LINE",     [(40, 0), (80, 0), (120, 0)],     30),
    ("OVERTAKE", [(40, 0), (145, 0), (95, 0)],     30),
    ("DIAMOND",  [(40, 35), (45, -30), (85, 0)],   60),
]

ARRIVE_DISTANCE_M = 4.0   # close enough to call a formation point "reached"
MOVE_TIMEOUT_S = 90       # give up waiting for a formation after this long


class Drone:
    """Latest known state of one PX4 drone, filled in from its messages."""

    def __init__(self, index):
        self.index = index                 # 0, 1, 2
        self.system_id = index + 1         # PX4 instance i uses system ID i+1
        self.address = ("127.0.0.1", DRONE_PORT_BASE + index)
        self.heard = False
        self.armed = False
        self.lat = None
        self.lon = None
        self.relative_alt_m = 0.0
        self.home_amsl_m = None            # home altitude above sea level


# ---------------------------------------------------------------------------
# Connection: receive from all drones on one port, send to each drone's port.
# ---------------------------------------------------------------------------

link = mavutil.mavlink_connection(GCS_LISTEN, source_system=255)
drones = [Drone(i) for i in range(NUM_DRONES)]
next_heartbeat = 0.0


def send(drone, message):
    """Packs a MAVLink message and sends it to one drone."""
    link.port.sendto(message.pack(link.mav), drone.address)
    link.mav.seq = (link.mav.seq + 1) % 256


def pump():
    """Reads all waiting messages, updates drone state, and sends the 1 Hz
    ground-station heartbeat that PX4 requires before it will arm."""
    global next_heartbeat

    while True:
        msg = link.recv_match(blocking=False)
        if msg is None:
            break
        index = msg.get_srcSystem() - 1
        if index < 0 or index >= NUM_DRONES or msg.get_srcComponent() != 1:
            continue
        drone = drones[index]
        drone.heard = True
        kind = msg.get_type()

        if kind == "HEARTBEAT":
            drone.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        elif kind == "GLOBAL_POSITION_INT":
            drone.lat = msg.lat / 1e7
            drone.lon = msg.lon / 1e7
            drone.relative_alt_m = msg.relative_alt / 1000.0
            if drone.home_amsl_m is None and not drone.armed:
                drone.home_amsl_m = (msg.alt - msg.relative_alt) / 1000.0
        elif kind == "STATUSTEXT":
            print(f"  [Drone {drone.system_id}] {msg.text}")

    now = time.time()
    if now >= next_heartbeat:
        for drone in drones:
            hb = link.mav.heartbeat_encode(
                mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            send(drone, hb)
        next_heartbeat = now + 1.0


def wait(seconds):
    """Waits while still reading messages and sending heartbeats."""
    end = time.time() + seconds
    while time.time() < end:
        pump()
        time.sleep(0.05)


def command_long(drone, command, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
    send(drone, link.mav.command_long_encode(
        drone.system_id, 1, command, 0, p1, p2, p3, p4, p5, p6, p7))


# ---------------------------------------------------------------------------
# Position math: meters east/north of the Ground Station <-> lat/lon.
# ---------------------------------------------------------------------------

def to_lat_lon(origin_lat, origin_lon, east_m, north_m):
    lat = origin_lat + north_m / 111320.0
    lon = origin_lon + east_m / (111320.0 * math.cos(math.radians(origin_lat)))
    return lat, lon


def distance_m(lat1, lon1, lat2, lon2):
    north = (lat2 - lat1) * 111320.0
    east = (lon2 - lon1) * 111320.0 * math.cos(math.radians(lat1))
    return math.hypot(north, east)


# ---------------------------------------------------------------------------
# Flight steps
# ---------------------------------------------------------------------------

def wait_for_drones():
    """Waits until every drone is heard and has a GPS position and home."""
    print("Waiting for all PX4 drones...")
    while not all(d.heard and d.lat is not None and d.home_amsl_m is not None
                  for d in drones):
        wait(0.5)
    print("All drones connected.")
    # Time for the estimators to settle (heading and height checks).
    wait(5)


def arm_and_take_off():
    """Arms each drone and starts the PX4 takeoff. Retries until armed,
    because preflight checks can take a few seconds to pass."""
    deadline = time.time() + 60
    took_off = [False] * NUM_DRONES
    while not all(took_off):
        for drone in drones:
            if not drone.armed:
                command_long(drone, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
        wait(1.0)
        # Takeoff is sent once, right after arming, before PX4's automatic
        # disarm (no takeoff within a few seconds) can trigger.
        for drone in drones:
            if drone.armed and not took_off[drone.index]:
                command_long(drone, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                             p4=float("nan"), p5=float("nan"),
                             p6=float("nan"), p7=float("nan"))
                took_off[drone.index] = True
        if time.time() > deadline:
            raise SystemExit("Drones did not arm within 60 s. Check the PX4 output.")
    print("All drones armed and taking off.")
    wait(8)


def go_to_formation(name, points, hold_s, origin):
    """Sends each drone to its formation point, waits until all arrive, then
    holds for hold_s seconds."""
    print(f"Formation {name}: flying...")
    targets = []
    for drone, (east, north), alt in zip(drones, points, ALTITUDE_M):
        lat, lon = to_lat_lon(origin[0], origin[1], east, north)
        targets.append((lat, lon))
        # DO_REPOSITION: fly to a point and hold there. The altitude is above
        # sea level, so home altitude is added to the height above home.
        send(drone, link.mav.command_int_encode(
            drone.system_id, 1, mavutil.mavlink.MAV_FRAME_GLOBAL,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0,
            -1,                       # default speed
            mavutil.mavlink.MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,
            0, float("nan"),          # yaw unchanged
            int(lat * 1e7), int(lon * 1e7),
            drone.home_amsl_m + alt))

    deadline = time.time() + MOVE_TIMEOUT_S
    while time.time() < deadline:
        wait(1.0)
        remaining = [distance_m(d.lat, d.lon, t[0], t[1])
                     for d, t in zip(drones, targets)]
        if all(r < ARRIVE_DISTANCE_M for r in remaining):
            break
    else:
        print(f"Formation {name}: not all drones arrived in {MOVE_TIMEOUT_S} s, continuing.")

    print(f"Formation {name}: holding {hold_s} s.")
    wait(hold_s)


def land_all():
    for drone in drones:
        command_long(drone, mavutil.mavlink.MAV_CMD_NAV_LAND,
                     p4=float("nan"), p5=float("nan"),
                     p6=float("nan"), p7=float("nan"))
    print("Landing...")
    deadline = time.time() + 60
    while any(d.armed for d in drones) and time.time() < deadline:
        wait(1.0)
    print("Landed." if not any(d.armed for d in drones) else "Landing timed out.")


def main():
    wait_for_drones()
    # Drone 1's home is the Ground Station location (same as andy_mesh_sim).
    origin = (drones[0].lat, drones[0].lon)
    print(f"Ground Station / origin: {origin[0]:.7f}, {origin[1]:.7f}")

    arm_and_take_off()
    for name, points, hold_s in FORMATIONS:
        go_to_formation(name, points, hold_s, origin)
    land_all()


if __name__ == "__main__":
    main()
