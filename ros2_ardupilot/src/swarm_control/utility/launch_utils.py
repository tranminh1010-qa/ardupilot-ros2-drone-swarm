#!/usr/bin/env python3

import os
import subprocess
from typing import List, Dict, Optional


class LaunchUtils:
    """Utility class for launch file operations and checks"""

    def __init__(self, sdf_path: str, param_file: str):
        """
        Initialize launch utilities

        Args:
            sdf_path: Path to the SDF model file
            param_file: Path to ArduPilot parameter file
        """
        self.sdf_path = sdf_path
        self.param_file = param_file

    def verify_paths(self) -> None:
        """Verify that required files exist"""
        for path in [self.sdf_path, self.param_file]:
            if not os.path.exists(path):
                raise FileNotFoundError(f"Required file not found: {path}")

    def check_running_processes(self, process_name: str) -> List[int]:
        """
        Check for running processes matching the given name

        Args:
            process_name: Name of process to check for

        Returns:
            List of PIDs for matching processes
        """
        try:
            output = subprocess.check_output(["pgrep", "-f", process_name]).decode().strip()
            return [int(pid) for pid in output.split('\n') if pid]
        except subprocess.CalledProcessError:
            return []

    def check_ros_nodes(self) -> List[str]:
        """
        Get list of running ROS nodes

        Returns:
            List of running ROS node names
        """
        try:
            output = subprocess.check_output(["ros2", "node", "list"]).decode().strip()
            return output.split('\n') if output else []
        except subprocess.CalledProcessError:
            return []

    def check_arming_status(self, drone_id: int) -> Optional[str]:
        """
        Check arming status of a specific drone

        Args:
            drone_id: ID of the drone to check

        Returns:
            Arming status message or None if not available
        """
        try:
            cmd = f"ros2 topic echo /drone{drone_id}/state -n 1 --no-arr"
            output = subprocess.check_output(cmd, shell=True, stderr=subprocess.DEVNULL).decode().strip()
            return output
        except subprocess.CalledProcessError:
            return None