#!/usr/bin/env python3
"""CameraDrone: extension of BaseDrone with a down-facing mapping camera.

Pairs with the base_drone_cam* Gazebo models (which wrap the plain base_drone
models with a GstCameraPlugin camera). BaseDrone itself stays camera-free;
this subclass adds:
  - a `camera_port` parameter (UDP port of the drone's H.264/RTP stream,
    5600 + instance; 0 disables the camera entirely),
  - a geotagged frame capture (+ ArUco detection) at every waypoint via the
    `on_waypoint_reached` extension hook.

Camera failures are non-fatal by design: if OpenCV, GStreamer, or the stream
is unavailable the drone flies its mission exactly like a BaseDrone.
"""
from base_drone_dds import BaseDrone, main
from scripts.camera_processor import CameraProcessor


class CameraDrone(BaseDrone):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter('camera_port', 0)
        self.camera_port = self.get_parameter(
            'camera_port').get_parameter_value().integer_value

        self.camera = CameraProcessor(
            self.drone_id, self.camera_port, logger=self.get_logger())

    def on_waypoint_reached(self, wp_index, lat, lon, alt):
        """Capture a geotagged mapping frame after arriving at a waypoint."""
        self.camera.capture(wp_index, lat=lat, lon=lon, alt=alt)

    def destroy_node(self):
        self.camera.close()
        super().destroy_node()


if __name__ == '__main__':
    main(node_class=CameraDrone)
