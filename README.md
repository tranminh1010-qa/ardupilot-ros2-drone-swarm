# ReadMe .....

This system relies on using Ardupilot (Autopilot) in conjunction with ROS2 (off-board computing).

Docker will be used to manage the images so that they work consistently across different environments.

## The steps

The following will build the image with the name `ardupilot/ardupilot-dev-ros` using the Dockerfile `Dockerfile_dev-ros`. Now, we need to go into the container and actually run the code.


This is to update the submodules build the image.
```bash
cd ros2_ardupilot/
git submodule update --init --recursive
docker build -t ardupilot-ros2 .
```

In order to have the GUI working, we need to do the following step to allow the container to access the host's display. This is so that we can run the Gazebo simulation and see the output.
```
1. Open Docker Desktop.
2. Navigate to Preferences -> Resources -> File Sharing.
3. Add /tmp/.X11-unix to the list of shared paths.
4. Apply and restart Docker.
```


This is to run the container in interactive mode and maintain the name `ardupilot-ros`. The `-v` flag is used to mount the `src` folder in the current directory to the `src` folder in the container. This is so that we can edit the code in the host machine and run it in the container.
```bash
xhost +local:docker
docker run --net=host \
  --name="ardupilot-ros2" \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$HOME/.Xauthority:/root/.Xauthority:rw" \
  --volume="$(pwd)/ardupilot:/root/ardu_ws/src/ardupilot" \
  --volume="$(pwd)/src/swarm_control:/root/ardu_ws/src/swarm_control" \
  --entrypoint /bin/bash \
  ardupilot-ros2 -c "tail -f /dev/null"
```
IF there is an error, try this:: `sudo chmod 1777 /tmp/.X11-unix `

To access once the container is running
```bash
docker container exec -it ardupilot-ros2 /bin/bash
````


Then we use the following to test everything is working correctly once inside the container

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ardupilot_dds_tests
```
If all the tests pass, then the environment is set up correctly, and we are ready to start running the code. 

We need to first compile the ardupilot SILT code and then run the swarm control code.
```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --console -I0
```
Run the following commands to launch the drones (all inside the container)
```bash
cd src/swarm_control/
colcon build
source install/setup.bash
ros2 launch swarm_control swarm.launch.py
```

In a separate terminal, run the following to see the list of publushers
```bash
source /opt/ros/humble/setup.bash
ros2 -t topic list
```