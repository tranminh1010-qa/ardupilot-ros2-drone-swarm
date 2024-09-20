# ReadMe .....

This system relies on using Ardupilot (Autopilot) in conjunction with ROS2 (off-board computing).

Docker will be used to manage the images so that they work consistently across different environments.

## The steps

The following will build the image with the name `ardupilot/ardupilot-dev-ros` using the Dockerfile `Dockerfile_dev-ros`. Now, we need to go into the container and actually run the code.


This is to build the image
```bash
cd ros2_ardupilot/
docker build -t ardupilot-ros .
```

This is to run the container in interactive mode and maintain the name `ardupilot-ros`. The `-v` flag is used to mount the `src` folder in the current directory to the `src` folder in the container. This is so that we can edit the code in the host machine and run it in the container.
```bash
docker run -it --name ardupilot-ros -v $(pwd)/src/swarm_control:/root/ardu_ws/src/swarm_control ardupilot-ros
```

To access once the container is running
```bash
docker container exec -it ardupilot-ros /bin/bash
````


Then we use the following to test everything is working correctly once inside the container

```bash
source /opt/ros/humble/setup.bash
rosdep update
colcon build --packages-up-to ardupilot_dds_tests
```

After that the environment is ready to be used.
```bash
rosdep install --from-paths ~/ardu_ws/src --ignore-src -r -y
colcon build
```