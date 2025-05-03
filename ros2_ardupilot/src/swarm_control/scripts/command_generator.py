#!/usr/bin/env python3

from typing import List, Dict


class CommandGenerator:
    """Generate commands for launch file processes"""

    def __init__(self, host_address: str, param_file: str):
        """
        Initialize command generator

        Args:
            host_address: Host IP address
            param_file: Path to ArduPilot parameter file
        """
        self.host_address = host_address
        self.param_file = param_file

    def get_common_env(self, ros_domain_id: str = '1') -> Dict[str, str]:
        """
        Get common environment variables for processes

        Args:
            ros_domain_id: ROS domain ID to use

        Returns:
            Dictionary of environment variables
        """
        return {
            'ARDU_SIM_PROVIDED': 'gz',
            'GZ_SIM_SYSTEM_PLUGIN_PATH': '/root/ardu_ws/install/ardupilot_gazebo/lib/ardupilot_gazebo/',
            'GZ_SIM_RESOURCE_PATH': '/root/ardu_ws/install/ardupilot_gazebo/share/ardupilot_gazebo/',
            'ARDU_UDP_BUFSIZE': '65536',
            'ROS_DOMAIN_ID': ros_domain_id
        }

    def get_build_cmd(self) -> List[str]:
        """
        Get command to build ArduPilot

        Returns:
            Shell command to build ArduPilot
        """
        return [
            'cd /root/ardu_ws/src/ardupilot && ./waf configure --board sitl && ./waf copter'
        ]

    def get_sitl_cmd(self, instance: int, sysid: int, ros_port: int, gazebo_port: int) -> List[str]:
        """
        Get command to start SITL instance

        Args:
            instance: SITL instance number
            sysid: System ID
            ros_port: Port for ROS communication
            gazebo_port: Port for Gazebo communication

        Returns:
            Command to start SITL instance
        """
        return [
            'sim_vehicle.py',
            '-v', 'ArduCopter',
            '-f', 'gazebo-iris',
            '--model', 'JSON',
            f'--instance', str(instance),
            f'--sysid', str(sysid),
            '--speedup', '1',
            f'--sim-address={self.host_address}',
            '--custom-location=40.072842,-105.230575,1586,0',
            f'--out=udp:127.0.0.1:{ros_port}',
            f'--out=udp:{self.host_address}:{gazebo_port}',
            '--add-param-file', self.param_file,
        ]

    def get_monitor_cmd(self) -> List[str]:
        """
        Get command to monitor system status

        Returns:
            Shell command for system monitoring
        """
        return [
            'bash', '-c',
            """
            for i in $(seq 1 10); do
                echo "-------------------- Status Check $i --------------------"
                echo "SITL processes:"
                pgrep -af arducopter
                echo ""
                echo "ROS2 nodes:"
                ros2 node list
                echo ""

                # Check arming status
                echo "Drone status:"
                if ros2 topic list | grep -q "/drone1/state"; then
                    echo "Drone 1 state:"
                    ros2 topic echo /drone1/state -n 1 --no-arr 2>/dev/null || echo "  Not available"
                fi

                sleep 5
            done
            """
        ]