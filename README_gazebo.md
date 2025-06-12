
# Connecting Gazebo Harmonic on Host with ArduPilot SITL in Docker

This guide explains how to connect Gazebo Simulation running on your host machine with ArduPilot SITL running in a Docker container.

## Setting Up Gazebo on Host with ArduPilot SITL in Docker

### 1. Configure Gazebo Transport on Host

Before starting Gazebo on your host, set these environment variables:
```bash
# On host machine
export GZ_IP=0.0.0.0  # Make Gazebo listen on all interfaces
export GZ_PARTITION=partition01  # Set a specific partition name
```
### 2. Find the Correct Host Address

**Method A: Docker Desktop (Windows/Mac)**
1. Open Docker Desktop → Settings → Resources → Network
2. Enable "Enable host networking"
3. Apply and restart Docker Desktop
4. From inside the container, ping the host:
```
bash
ping host.docker.internal
```
5. Note the resolved IP address (e.g., `192.168.65.2`)

**Method B: Linux Docker**
1. Check the Docker bridge interface:
```
bash
ip -4 addr show docker0 | grep -Po 'inet \K[\d.]+'
```
2. This will typically return `172.17.0.1`

### 3. Update Your Launch File

Edit your `decentralized_swarm.launch.py` file to use the correct host address:
```
python
# Replace with your identified Docker host address
host_address = '172.17.0.1'  # Linux Docker
# or
host_address = '192.168.65.2'  # Docker Desktop
```
Also update any UDP output addresses:
```
python
f'--out=udp:{host_address}:{gazebo_port}'
```
### 4. Configure Environment Variables in Docker

Inside your Docker container, set these environment variables:
```
bash
# Inside Docker container
export GZ_IP=host.docker.internal
export GZ_PARTITION=partition01
export IGN_IP=host.docker.internal
export IGN_PARTITION=partition01
```
You can add these to your Dockerfile or to the container's `.bashrc` to make them permanent.

### 5. Configure Gazebo on the Host

1. Follow the [ArduPilot with Gazebo setup guide](https://ardupilot.org/dev/docs/sitl-with-gazebo.html)
2. Rebuild your Gazebo workspace if required
3. Add the path of the project custom models to the gazebo models path in bashrc. Example:
```bash

```

### 6. Launch Sequence

1. Start Gazebo on the host machine:
```
bash
export GZ_IP=0.0.0.0
export GZ_PARTITION=partition01
gz sim -v4 -r swarm_drone.sdf
```
Make sure this launches the Gazebo simulation on your host machine and you are able to see the three drones before continuing further.

2. Launch ArduPilot SITL from the container:
```
bash
export GZ_IP=host.docker.internal
export GZ_PARTITION=partition01
export IGN_IP=host.docker.internal
export IGN_PARTITION=partition01
ros2 launch swarm_control decentralized_swarm.launch.py
```
## Troubleshooting

### Connection Issues

If you see this warning, it's usually harmless:
```

Error setting socket option (IP_MULTICAST_IF).
Did you set the environment variable IGN_IP with a correct IP address? 
[host.docker.internal] seems an invalid local IP address.
Using 127.0.0.1 as hostname.
```
This occurs because `host.docker.internal` is a DNS name, not an actual IP address on the local machine. As long as you can see the Gazebo topics when running `gz topic -l`, the connection is working.

### Other Issues

- If connection issues persist, check if firewalls are blocking communication
- Verify network settings in Docker configuration
- For Docker Desktop, ensure host networking is enabled
- Confirm the correct ports are being used in both Gazebo and ArduPilot configuration

### Testing Connection

You can verify your Docker host address is correctly configured by running:
```
bash
# From inside the container
ping host.docker.internal
```
This should show successful communication between your container and the host machine.

You can also verify that the Gazebo topics are visible from the container:
```
bash
# From inside the container
gz topic -l
```
If you see topics like `/world/iris_runway/model/drone1/model/iris_with_standoffs/link/imu_link/sensor/imu_sensor/imu`, the connection is working correctly.

## Notes on Gazebo Harmonic Transport

Unlike older Gazebo versions which used fixed ports (11345/11346), Gazebo Harmonic (gz-sim) uses a custom transport mechanism. This is why traditional port scanning won't show the expected ports open. Instead, the communication happens through a combination of:

1. UDP multicast discovery
2. Direct TCP connections for data transport
3. Named partitions for isolation

As long as the environment variables are set correctly, the Docker container should be able to communicate with the Gazebo instance running on the host.
```
