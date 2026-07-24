#!/usr/bin/env python3
"""Mirror ArduPilot's sprayer pump output onto the Gazebo spray plumes.

Runs on the HOST (needs pymavlink + the gz CLI). For each sprayer instance it
connects to the drone's SERIAL2 MAVLink TCP port (5763 + 10*instance — SERIAL1
is used by the SprayerDrone node) and watches SERVO_OUTPUT_RAW channel 9, the
AC_Sprayer pump output (SERVO9_FUNCTION 22). Pump PWM above threshold ->
publish emitting=true on that drone's particle-emitter topic
(/sprayer/<instance>/spray_cmd); below -> emitting=false.

This makes the visual plume reflect the REAL firmware behaviour: AC_Sprayer
only pumps above SPRAY_SPEED_MIN and scales with ground speed.

Usage:
    python3 src/scripts/spray_visualizer.py --instances 2 3 4 5
"""
import argparse
import subprocess
import threading
import time

from pymavlink import mavutil

PUMP_ON_PWM = 1150  # pump servo PWM above this counts as "spraying"


def gz_pub(instance, emitting):
    msg = f"emitting: {{data: {'true' if emitting else 'false'}}}"
    subprocess.run(
        ["gz", "topic", "-t", f"/sprayer/{instance}/spray_cmd",
         "-m", "gz.msgs.ParticleEmitter", "-p", msg],
        capture_output=True, timeout=10)


def watch_instance(instance):
    port = 5763 + 10 * instance
    state = None
    while True:
        try:
            m = mavutil.mavlink_connection(f"tcp:127.0.0.1:{port}")
            m.wait_heartbeat(timeout=15)
            print(f"[inst {instance}] connected tcp:{port}", flush=True)
            # ask for servo output stream
            m.mav.request_data_stream_send(
                m.target_system, m.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 4, 1)
            while True:
                msg = m.recv_match(type="SERVO_OUTPUT_RAW",
                                   blocking=True, timeout=10)
                if msg is None:
                    continue
                pumping = msg.servo9_raw > PUMP_ON_PWM
                if pumping != state:
                    state = pumping
                    gz_pub(instance, pumping)
                    print(f"[inst {instance}] pump pwm={msg.servo9_raw} -> "
                          f"plume {'ON' if pumping else 'OFF'}", flush=True)
        except Exception as e:
            print(f"[inst {instance}] {e}; retrying in 5s", flush=True)
            if state:
                gz_pub(instance, False)
                state = False
            time.sleep(5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--instances", nargs="+", type=int, required=True,
                    help="0-indexed sprayer instances, e.g. --instances 2 3 4 5")
    args = ap.parse_args()
    threads = [threading.Thread(target=watch_instance, args=(i,), daemon=True)
               for i in args.instances]
    for t in threads:
        t.start()
    while True:
        time.sleep(60)


if __name__ == "__main__":
    main()
