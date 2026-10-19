"""Fly the simulated aircraft with ArduPlane: arm in LOITER, and log the flight.

Start the simulator in lockstep mode and ArduPlane first (see the README), then run this script.
The simulator holds the aircraft until it is launched, with L in the viewer, or with
`--launch-after` in the headless simulator. ArduPlane then circles the launch point.

Usage: python3 scripts/sitl_loiter.py [SECONDS]
"""

import sys
import time

from pymavlink import mavutil

flight_seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 90

link = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=255)
link.wait_heartbeat(timeout=60)
print("Connected to ArduPlane", flush=True)
link.mav.request_data_stream_send(
    link.target_system, link.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1
)

start = time.time()
armed_at = None
last_report = 0.0
while armed_at is None or time.time() - armed_at < flight_seconds:
    msg = link.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue
    if msg.get_type() == "STATUSTEXT":
        print("ArduPlane:", msg.text, flush=True)

    # Give the EKF time to settle before arming
    if armed_at is None and time.time() - start > 25:
        link.set_mode("LOITER")
        link.mav.command_long_send(
            link.target_system, link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0,
        )
        armed_at = time.time()
        print("Armed in LOITER, launch the aircraft", flush=True)

    if msg.get_type() == "VFR_HUD" and armed_at is not None and time.time() - last_report > 5:
        last_report = time.time()
        print(
            f"airspeed {msg.airspeed:5.1f}  altitude {msg.alt:7.1f}  climb {msg.climb:5.1f}"
            f"  throttle {msg.throttle:3d}  heading {msg.heading}",
            flush=True,
        )
