#!/usr/bin/env python3

import socket
import time
import sys
import os
import signal
import argparse
import json
import subprocess
from datetime import datetime


def log(message, level="INFO"):
    """Log a message with timestamp"""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] [{level}] {message}")


def test_connection(host, port, name="Gazebo", timeout=2):
    """Test connection to a specific host and port"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        log(f"Testing connection to {name} at {host}:{port}")
        result = sock.connect_ex((host, port))

        if result == 0:
            log(f"✅ Connection to {name} at {host}:{port} successful")
            return True
        else:
            log(f"❌ Connection to {name} at {host}:{port} failed (Error code: {result})", "ERROR")
            return False
    except Exception as e:
        log(f"❌ Error testing connection to {name} at {host}:{port}: {e}", "ERROR")
        return False
    finally:
        sock.close()


def test_udp_bound(port, interface='0.0.0.0'):
    """Test if we can bind to a UDP port"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((interface, port))
        log(f"✅ Successfully bound to UDP port {port}")
        sock.close()
        return True
    except Exception as e:
        log(f"❌ Failed to bind to UDP port {port}: {e}", "ERROR")
        return False


def test_ardupilot_ports():
    """Test common ArduPilot SITL ports"""
    sitl_ports = [
        5760,  # Default SITL TCP port
        5762, 5764, 5766,  # Additional TCP ports for multi-vehicle
        14550, 14551,  # UDP GCS ports
        9002, 9003,  # Gazebo plugin ports (older plugin)
        5002, 5003,  # Alternative Gazebo plugin ports
    ]

    for port in sitl_ports:
        test_udp_bound(port)


def check_environment_variables():
    """Check if required environment variables are set"""
    gazebo_vars = [
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        "GZ_SIM_RESOURCE_PATH",
        "GAZEBO_MASTER_URI",
        "GAZEBO_MODEL_PATH",
        "GAZEBO_PLUGIN_PATH"
    ]

    log("=== Checking Gazebo environment variables ===")
    for var in gazebo_vars:
        value = os.environ.get(var)
        if value:
            log(f"✅ {var} is set to: {value}")
        else:
            log(f"❌ {var} is not set", "WARN")

    # Check ROS environment if ROS is being used
    ros_vars = ["ROS_DOMAIN_ID", "ROS_MASTER_URI", "RMW_IMPLEMENTATION"]

    log("=== Checking ROS environment variables ===")
    for var in ros_vars:
        value = os.environ.get(var)
        if value:
            log(f"✅ {var} is set to: {value}")
        else:
            log(f"❓ {var} is not set (may not be needed if not using ROS)", "WARN")


def check_network_interfaces():
    """Get all network interfaces and their IPs"""
    log("=== Network interfaces ===")
    try:
        # Use 'hostname -I' to get all IPs
        result = subprocess.check_output(['hostname', '-I']).decode('utf-8').strip()
        log(f"All IP addresses: {result}")

        # Try to get more detailed info with ip addr
        try:
            ip_output = subprocess.check_output(['ip', 'addr']).decode('utf-8')
            interfaces = []
            for line in ip_output.split('\n'):
                if 'inet ' in line:
                    parts = line.strip().split()
                    interface = parts[-1]
                    ip = parts[1].split('/')[0]
                    interfaces.append(f"{interface}: {ip}")

            if interfaces:
                for iface in interfaces:
                    log(f"Interface: {iface}")
        except (subprocess.SubprocessError, FileNotFoundError):
            log("Couldn't get detailed interface info with 'ip addr'", "WARN")

    except subprocess.SubprocessError:
        log("Failed to get IP addresses", "ERROR")


def check_docker_network():
    """Check Docker network configuration"""
    log("=== Docker network configuration ===")
    try:
        # Check if docker is installed
        subprocess.check_output(['which', 'docker'])

        # Get Docker network info
        net_output = subprocess.check_output(['docker', 'network', 'ls']).decode('utf-8')
        log("Docker networks:")
        for line in net_output.split('\n')[1:]:  # Skip header
            if line.strip():
                log(f"  {line.strip()}")

        # Check if we're running in a container
        if os.path.exists('/.dockerenv'):
            log("✅ Running inside a Docker container")

            # Get container network info
            container_id = subprocess.check_output(['hostname']).decode('utf-8').strip()
            log(f"Container hostname: {container_id}")

            try:
                inspect_output = subprocess.check_output(['docker', 'container', 'inspect', container_id]).decode(
                    'utf-8')
                network_info = json.loads(inspect_output)
                if network_info and len(network_info) > 0:
                    networks = network_info[0].get('NetworkSettings', {}).get('Networks', {})
                    for net_name, net_data in networks.items():
                        log(f"Network: {net_name}")
                        log(f"  IP Address: {net_data.get('IPAddress', 'Unknown')}")
                        log(f"  Gateway: {net_data.get('Gateway', 'Unknown')}")
                        log(f"  Subnet: {net_data.get('IPPrefixLen', 'Unknown')}")
            except subprocess.SubprocessError:
                log("Failed to inspect container network", "WARN")
    except (subprocess.SubprocessError, FileNotFoundError):
        log("Docker not found or not running", "WARN")


def check_ports_in_use():
    """Check which ports are already in use"""
    log("=== Checking ports in use ===")
    try:
        # Different command for Linux/macOS
        if sys.platform.startswith('linux'):
            cmd = ['ss', '-tuln']
        elif sys.platform.startswith('darwin'):
            cmd = ['netstat', '-an']
        else:
            log(f"Unsupported platform: {sys.platform}", "WARN")
            return

        output = subprocess.check_output(cmd).decode('utf-8')

        # Look for common Gazebo/ArduPilot ports
        ports_to_check = [
            '5760', '14550', '14551', '9002', '9003', '5002', '5003',
            '11345', '11346'  # Gazebo server ports
        ]

        for line in output.split('\n'):
            for port in ports_to_check:
                if f":{port}" in line:
                    log(f"Port {port} is in use: {line.strip()}")

    except subprocess.SubprocessError:
        log("Failed to check ports in use", "ERROR")


def check_ardupilot_gazebo_plugin():
    """Check if the ArduPilot Gazebo plugin is installed correctly"""
    log("=== Checking ArduPilot Gazebo Plugin ===")

    # Check for plugin in common locations
    plugin_paths = [
        "/usr/lib/x86_64-linux-gnu/gz-sim-7/plugins/",
        "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/",
        os.path.expanduser("~/gz_ws/build/"),
        os.path.expanduser("~/ardu_ws/build/"),
    ]

    plugin_found = False
    for path in plugin_paths:
        if os.path.exists(path):
            log(f"Checking {path}")
            for file in os.listdir(path):
                if "ArduPilotPlugin" in file or "ardupilot" in file.lower():
                    log(f"✅ Found plugin: {path}/{file}")
                    plugin_found = True

    if not plugin_found:
        log("❌ ArduPilot Gazebo plugin not found in common locations", "ERROR")

        # Check environment paths
        plugin_path = os.environ.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        if plugin_path:
            log(f"Checking environment plugin path: {plugin_path}")
            for path_part in plugin_path.split(':'):
                if path_part and os.path.exists(path_part):
                    for file in os.listdir(path_part):
                        if "ArduPilotPlugin" in file or "ardupilot" in file.lower():
                            log(f"✅ Found plugin in environment path: {path_part}/{file}")
                            plugin_found = True

        if not plugin_found:
            log("❌ ArduPilot Gazebo plugin not found in environment paths", "ERROR")


def check_for_gazebo_processes():
    """Check if Gazebo or SITL processes are running"""
    log("=== Checking for Gazebo and SITL processes ===")

    try:
        # Check for gazebo processes
        ps_output = subprocess.check_output(['ps', 'aux']).decode('utf-8')

        gazebo_processes = []
        sitl_processes = []

        for line in ps_output.split('\n'):
            if 'gz ' in line or 'gazebo' in line:
                gazebo_processes.append(line.strip())
            if 'sim_vehicle.py' in line or 'ardupilot' in line:
                sitl_processes.append(line.strip())

        if gazebo_processes:
            log("Gazebo processes found:")
            for proc in gazebo_processes:
                log(f"  {proc}")
        else:
            log("❌ No Gazebo processes found", "WARN")

        if sitl_processes:
            log("SITL processes found:")
            for proc in sitl_processes:
                log(f"  {proc}")
        else:
            log("❌ No SITL processes found", "WARN")

    except subprocess.SubprocessError:
        log("Failed to check for running processes", "ERROR")


def test_host_connectivity():
    """Test various hostname and IP combinations to check connectivity"""
    log("=== Testing host connectivity ===")

    # Common hostnames and IPs to test
    hosts_to_test = [
        ("localhost", 11345),
        ("127.0.0.1", 11345),
        ("172.17.0.1", 11345),  # Common Docker host gateway
        ("host.docker.internal", 11345),  # Docker for Mac/Windows hostname
        ("gazebo", 11345),  # In case there's a host named 'gazebo'
    ]

    for host, port in hosts_to_test:
        test_connection(host, port, f"Gazebo on {host}")


def main():
    """Main function to run all the tests"""
    parser = argparse.ArgumentParser(description='Debug Gazebo and ArduPilot SITL connectivity')
    parser.add_argument('--full', action='store_true', help='Run all tests')
    parser.add_argument('--connection', action='store_true', help='Test connections to common Gazebo ports')
    parser.add_argument('--env', action='store_true', help='Check environment variables')
    parser.add_argument('--network', action='store_true', help='Check network interfaces')
    parser.add_argument('--docker', action='store_true', help='Check Docker network configuration')
    parser.add_argument('--ports', action='store_true', help='Check ports in use')
    parser.add_argument('--plugin', action='store_true', help='Check ArduPilot Gazebo plugin')
    parser.add_argument('--processes', action='store_true', help='Check running processes')

    args = parser.parse_args()

    # If no arguments provided, show help
    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit(1)

    # If --full or no specific tests selected, run all tests
    run_all = args.full or not any([
        args.connection, args.env, args.network, args.docker,
        args.ports, args.plugin, args.processes
    ])

    log("Starting Gazebo-ArduPilot connection debugging")

    if run_all or args.connection:
        test_host_connectivity()
        test_ardupilot_ports()

    if run_all or args.env:
        check_environment_variables()

    if run_all or args.network:
        check_network_interfaces()

    if run_all or args.docker:
        check_docker_network()

    if run_all or args.ports:
        check_ports_in_use()

    if run_all or args.plugin:
        check_ardupilot_gazebo_plugin()

    if run_all or args.processes:
        check_for_gazebo_processes()

    log("Debugging completed")


if __name__ == "__main__":
    main()