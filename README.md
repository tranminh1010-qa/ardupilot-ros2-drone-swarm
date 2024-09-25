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
git submodule update --init --recursive
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
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$(pwd)/src/swarm_control:/home/ardupilot/ardu_ws/src/swarm_control" \
  ardupilot-ros2
```
The terminal should hang, to continue, open a new terminal and keep working. 

IF there is an error, try this:: `sudo chmod 1777 /tmp/.X11-unix `.

### 4. Build the ROS2 and Ardupilot workspaces

To access once the container is running
```bash
docker container exec -it ardupilot-ros2 /bin/bash
````

### 5. Test everything is working correctly 
Then we use the following to test everything is working correctly once inside the container

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ardupilot_dds_tests
colcon test-result --all --verbose
```
If all the tests pass, then the environment is set up correctly, and we are ready to start running the code. 

We can also make sure that Ardupilot is working correctly with ROS2. We can do so by running the following.
For more information refer to [ardupilot/Tools/ros2/README.md](https://github.com/ArduPilot/ardupilot/tree/master/Tools/ros2#readme). There you can find examples of launches using serial connection instead of udp, as well as a step-by-step breakdown of what the launch files are doing.
```bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501```
```

In a separate terminal, run the following to see the list of publishers
```bash
docker container exec -it ardupilot-ros23 /bin/bash
 source ~/ardu_ws/install/setup.bash
ros2 -t topic list
```