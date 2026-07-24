#!/usr/bin/env python3
"""SprayerDrone: extension of BaseDrone for the spraying role.

Pairs with the base_drone_sprayer_* Gazebo models and sprayer.parm (which
enables ArduPilot's AC_Sprayer: pump on SERVO9/function 22, speed-proportional
flow, RC aux function 15 as the on/off switch).

The drone flies its assigned strip like any BaseDrone; this subclass simply
turns the REAL firmware sprayer on for the duration of the mission and off
afterwards, via MAV_CMD_DO_AUX_FUNCTION over the drone's SERIAL1 MAVLink TCP
port (5762 + 10*instance — free, since ground stations use the UDP outputs).
While flying, AC_Sprayer drives the pump proportionally to ground speed and
stops below SPRAY_SPEED_MIN; the host-side spray_visualizer.py mirrors the
pump servo output onto the Gazebo particle-emitter plumes.

MAVLink failures are non-fatal: without the side channel the drone still
flies its mission, just without spraying.
"""
import json
import os
import time

from base_drone_dds import BaseDrone, main

try:
    from pymavlink import mavutil
    _MAV_OK = True
except ImportError:  # pragma: no cover
    mavutil = None
    _MAV_OK = False

# MAV_CMD_DO_AUX_FUNCTION (218): param1 = aux function, param2 = switch pos
MAV_CMD_DO_AUX_FUNCTION = 218
AUX_FUNC_SPRAYER = 15
AUX_POS_LOW, AUX_POS_HIGH = 0, 2


class SprayerDrone(BaseDrone):
    def __init__(self, node_name):
        super().__init__(node_name)

        # SERIAL1 MAVLink TCP port of this instance (0 disables spraying)
        self.declare_parameter('mavlink_tcp_port', 0)
        self._mav_port = self.get_parameter(
            'mavlink_tcp_port').get_parameter_value().integer_value
        self._mav = None  # None = not tried, False = unavailable

        # Prescription plan produced by prescription_planner.py from the
        # mappers' weed detections. Empty path = fly assigned_waypoints
        # (blanket strip) as before.
        self.declare_parameter('prescription_file', '')
        self.declare_parameter('prescription_timeout', 600.0)
        self._rx_file = self.get_parameter(
            'prescription_file').get_parameter_value().string_value
        self._rx_timeout = self.get_parameter(
            'prescription_timeout').get_parameter_value().double_value

    def prepare_mission(self):
        """Wait ON THE GROUND for the prescription plan, then fly its weed
        targets instead of the blanket strip. On timeout, fall back to the
        assigned strip so the field still gets treated."""
        if not self._rx_file:
            return
        self.get_logger().info(
            f"[sprayer] waiting for prescription {self._rx_file} "
            f"(timeout {self._rx_timeout:.0f}s)")
        deadline = time.time() + self._rx_timeout
        while time.time() < deadline:
            if os.path.exists(self._rx_file):
                try:
                    with open(self._rx_file) as f:
                        plan = json.load(f)
                    self.waypoints = [tuple(wp) for wp in plan['waypoints']]
                    self.get_logger().info(
                        f"[sprayer] prescription loaded: "
                        f"{len(self.waypoints)} weed targets")
                    return
                except (json.JSONDecodeError, KeyError, OSError) as e:
                    self.get_logger().warning(
                        f"[sprayer] bad prescription file: {e}; retrying")
            time.sleep(5)
        self.get_logger().warning(
            "[sprayer] prescription timeout - falling back to "
            f"blanket strip ({len(self.waypoints)} waypoints)")

    def _mavlink(self):
        if self._mav is None:
            if not (_MAV_OK and self._mav_port):
                self._mav = False
            else:
                try:
                    m = mavutil.mavlink_connection(
                        f'tcp:127.0.0.1:{self._mav_port}')
                    m.wait_heartbeat(timeout=10)
                    self._mav = m
                    self.get_logger().info(
                        f"[sprayer] MAVLink up on tcp:{self._mav_port}")
                except Exception as e:
                    self.get_logger().warning(
                        f"[sprayer] MAVLink unavailable on "
                        f"tcp:{self._mav_port}: {e} - flying without spray")
                    self._mav = False
        return self._mav or None

    def set_spray(self, on):
        """Flip ArduPilot's sprayer aux switch (AC_Sprayer on/off)."""
        m = self._mavlink()
        if m is None:
            return
        m.mav.command_long_send(
            m.target_system, m.target_component,
            MAV_CMD_DO_AUX_FUNCTION, 0,
            AUX_FUNC_SPRAYER, AUX_POS_HIGH if on else AUX_POS_LOW,
            0, 0, 0, 0, 0)
        self.get_logger().info(f"[sprayer] spray {'ON' if on else 'OFF'}")

    def start_mission(self):
        self.set_spray(True)
        try:
            super().start_mission()
        finally:
            self.set_spray(False)


if __name__ == '__main__':
    main(node_class=SprayerDrone)
