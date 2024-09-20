# ReadMe .....

This system relies on using Ardupilot (Autopilot) in conjunction with ROS2 (off-board computing).

Docker will be used to manage the images so that they work consistently across different environments.

## The steps

The following will build the image with the name `ardupilot/ardupilot-dev-ros` using the Dockerfile `Dockerfile_dev-ros`. Now, we need to go into the container and actually run the code.


This is to build the image
```bash
cd ros2_ardupilot/ardupilot_dev_docker/docker
docker build -t ardupilot/ardupilot-dev-ros -f Dockerfile_dev-ros .
```


This is to run the container in interactive mode
```bash
docker run -it --name ardupilot-dds ardupilot/ardupilot-dev-ros
```

Then we need to run the following inside the container

```bash
mkdir -p ~/ardu_ws/src
cd ~/ardu_ws
vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

# Now, we need to build the workspace

cd ~/ardu_ws
sudo apt update
rosdep update
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src

# Installing the MicroXRCEDDSGen build dependency:

sudo apt install default-jre
git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
cd ~/ardu_ws/src/Micro-XRCE-DDS-Gen
./gradlew assemble
echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc
source ~/.bashrc

# Source the environment and start building

sudo apt update
source /opt/ros/humble/setup.bash
cd ~/ardu_ws
colcon build --packages-up-to ardupilot_dds_tests
```