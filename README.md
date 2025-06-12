# ReadMe .....

This system relies on using Ardupilot (Autopilot) in conjunction with ROS2 (off-board computing).

Docker will be used to manage the images so that they work consistently across different environments.

## The steps

### 1. Ensure docker is set up properly
```bash
sudo usermod -aG docker $USER
```

Verify the group was added successfully: `groups $USER`
You should see 'docker' in the list of groups.
Important: For the group changes to take effect, you need to log out and log back in. You can do this by: `newgrp docker`

### 2. Build the image
The following will build the image with the name `ardupilot/ardupilot-dev-ros` using the Dockerfile `Dockerfile_dev-ros`. Now, we need to go into the container and actually run the code.

This is to update the submodules build the image.
```bash
cd ros2_ardupilot/
docker build -t ardupilot-ros2 .
```

### 3. Run the container
This is to run the container in interactive mode and maintain the name `ardupilot-ros`. The `-v` flag is used to mount the `src` folder in the current directory to the `src` folder in the container. This is so that we can edit the code in the host machine and run it in the container.
```bash
xhost +local:docker
docker run -it --net=host \
  --name="ardupilot-ros2" \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --device=/dev/dri:/dev/dri \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$(pwd)/src/swarm_control:/root/ardu_ws/src/swarm_control" \
  --volume="$(pwd)/src/custom_gz:/root/ardu_ws/src/custom_gz" \
 --runtime=nvidia \
    ardupilot-ros2
```

If you have GPU and Cuda runtime set up on the device, you can also run the following for much faster performance 
```bash
xhost +local:docker
# Optimized ArduPilot ROS2 Docker Command
docker run -it \
  --name="ardupilot_ros2" \
  --hostname="ardupilot-dev" \
  --network=host \
  --ipc=host \
  --pid=host \
  --privileged \
  \
  `# GPU Configuration` \
  --gpus all \
  --device=/dev/dri:/dev/dri \
  \
  `# Display & GUI` \
  --env DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --env LIBGL_ALWAYS_INDIRECT=0 \
  --env XAUTHORITY=/tmp/.Xauth \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --volume $HOME/.Xauthority:/tmp/.Xauth:ro \
  \
  `# NVIDIA GPU Settings` \
  --env NVIDIA_VISIBLE_DEVICES=all \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env CUDA_VISIBLE_DEVICES=all \
  \
  `# Gazebo Configuration (Optimized)` \
  --env GZ_RENDERING_ENGINE=ogre2 \
  --env GZ_SIM_WORKER_THREADS=8 \
  --env GZ_PARTITION=$(hostname -f) \
  --env GZ_IP=127.0.0.1 \
  --env GZ_DISCOVERY_MULTICAST=1 \
  --env GZ_TRANSPORT_TOPIC_STATISTICS=0 \
  --env GZ_VERBOSE=0 \
  --env IGN_GAZEBO_PHYSICS_ENGINE=dart \
  `# Performance Optimizations` \
  --shm-size=2g \
  --cpus=6 \
  --memory=14g \
  --memory-swap=16g \
  \
  `# System Capabilities` \
  --cap-add=SYS_NICE \
  --cap-add=SYS_PTRACE \
  --cap-add=NET_ADMIN \
  --ulimit rtprio=99:99 \
  --ulimit memlock=-1:-1 \
  --ulimit nofile=65536:65536 \
  \
  `# Volume Mounts (Optimized)` \
  --volume "$(pwd)/src:/root/ardu_ws/src:cached" \
  --volume "$(pwd)/config:/root/config:ro" \
  --volume "$(pwd)/logs:/root/logs:delegated" \
  --volume "/dev/input:/dev/input:ro" \
  --volume "/run/udev:/run/udev:ro" \
  ardu_edited \
  bash
```
The terminal should hang, to continue, open a new terminal and keep working. 

IF there is an error, try this:: `sudo chmod 1777 /tmp/.X11-unix `.

### 4. Build the ROS2 and Ardupilot workspaces

To access once the container is running
```bash
docker container exec -it ardupilot-ros2 /bin/bash
````

### Test everything is working correctly  (Optional)
Then we use the following to test everything is working correctly once inside the container

```bash
colcon build 
colcon test-result --all --verbose
```
If all the tests pass, then the environment is set up correctly, and we are ready to start running the code. 

We can also make sure that Ardupilot is working correctly with ROS2. We can do so by running the following.
For more information refer to [ardupilot/Tools/ros2/README.md](https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2#readme). There you can find examples of launches using serial connection instead of udp, as well as a step-by-step breakdown of what the launch files are doing.
```bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
  transport:=udp4 \
  synthetic_clock:=True \
  wipe:=False \
  model:=quad \
  speedup:=1 \
  dds_enable:=1 \
  slave:=0 \
  instance:=0 \
  defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,
      $(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm,
      ~/ardu_ws/src/swarm_control/parameters/ardu_gps_noise.parm \
  sim_address:=127.0.0.1 \
  master:=tcp:127.0.0.1:5760 \
  sitl:=127.0.0.1:5501
```

In a separate terminal, run the following to see the list of publishers
```bash
docker container exec -it ardupilot_ros2 /bin/bash
export ROS_DOMAIN_ID=0
ros2  topic list
```

Finally, to run the simulation in gazebo, run the following in yet another terminal
```bash
docker container exec -it ardupilot_ros2 /bin/bash
export ROS_DOMAIN_ID=0
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```

### 5. Running the code

To run the code, we need to first build the workspace. This is done by running the following in the container.
```bash
cd ~/ardu_ws
. ~/.profile
````

### Set Up Scripts and Launch Files

Initially make sure all packages are built
```bash
cd ~/ardu_ws/src
colcon build
source install/setup.bash
```
Then after making changes to the source code, you can simply use `colcon build --packages-select swarm_control --symlink-install` to only rebuild the package. The use of --symlink-install will make code changes reflect without rebuilding (unless you change the package structure to update dependencies)
## 6. Launch Simulation
### Launch Gazebo
Open an ubuntu terminal and execute the following command to start Gazebo:
```bash
gz sim -v4 -r swarm_drone.sdf
```
Make sure the following variables are set on the host machine and docker machine - they need to match
```bash
# Host machine environment variables
export GZ_PARTITION=$(hostname)
export GZ_IP=127.0.0.1
export GZ_VERBOSE=4
export GZ_DISCOVERY_MULTICAST=1
export GZ_TRANSPORT_TOPIC_STATISTICS=1
```

In the case of running the gazebo on the host machine, read the README_gazebo.md instructions.

Changing the number of drones will require updating the world file, but it is a trivial change to add or remove drones
### Launch the Swarm Control
```bash
ros2 launch swarm_control decentralized_swarm.launch.py num_drones:=4
```

## 9. Test Custom SITL Binary
To run a test if the custom SITL binary is working along with the custom parameters, you first need to start and enter the docker container by running these commands:
 ```bash
docker container start ardupilot-ros2
docker container exec -it ardupilot-ros2 /bin/bash
```

Then navigate to the testing script directory inside the docker container and run the testing script:
```bash
cd /root/ardu_ws/src/swarm_control/custom-files
./run-test.sh
```

### 10. Debug Potential Issue (Optional)

If you get an error in the log that display cannot be opened. Exit the container and run this again in the terminal. This will usually need to be run when you restart the computer
```bash
xhost +local:docker
```

In case there is an issue creating a connection between the ros2 nodes and the ardupilot SIL, let's start by checking running processes
```bash
sudo lsof -i :5760 #Change the port depending on the situation
sudo kill -9 <PID>  # Replace <PID> with the process ID from the lsof command
```

Now, we check if it is a permission issue. Close down all running operations and run as a non-root user
```bash
pkill -f arducopter
pkill -f mavproxy
. ~/.profile
```

If you get the error message: "xterm is not installed. Please install it first.", then you need to run this command inside the docker container to install xterm:
```bash
apt-get update && apt-get install -y xterm
```

If you get issues during build with micro_ros_messages, remove the build directory for it then try colcon build again
```bash
rm -rf /root/ardu_ws/build/micro_ros_msgs/ament_cmake_python/micro_ros_msgs/micro_ros_msgs
cd ~/ardu_ws
rm -rf build/micro_ros_msgs install/micro_ros_msgs log/build_*/micro_ros_msgs
```

If some instances fail to start because they did not shut down correctly. You can see what is still running and terminate it
```bash
# Check for all ArduPilot SITL processes
ps aux | grep -E "arducopter|arduplane|arduhexa|ardurover" | grep -v grep

# Check for specific instance numbers
ps aux | grep "instance" | grep -v grep

# Check all related processes (SITL, MAVProxy, etc.)
ps aux | grep -E "ardu|mavproxy|sitl" | grep -v grep
```
You can also check which processes are using specific ports:
```bash
# Install netstat if not available
apt-get update && apt-get install -y net-tools

# Check ports used by ArduPilot SITL
netstat -tuln | grep -E "5760|6180|14550|5501"
# Check all relevant ports
sudo lsof -i -P | grep -E "5760|6180|14550|5501|2019"
```

You can terminate running processes by running `kill -9 $pid`
