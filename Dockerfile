# Use an official Ubuntu image as the base
FROM ubuntu:20.04

# Set environment variables to avoid user interaction during package installation
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /src/app
# Update package list and install dependencies
RUN apt-get update -y && \
    apt-get upgrade -y && \
    apt-get install -y \
    git \
    python3-pip \
    python3-dev \
    build-essential \
    sudo \
    lsb-release \
    gnupg \
    curl

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    apt-get update -y && \
    apt-get install -y ros-noetic-desktop-full python3-rosdep && \
    rosdep init && \
    rosdep update

# Install required Python packages
RUN pip3 install numpy scipy matplotlib -U

# Create a non-root user
#RUN useradd -m -s /bin/bash ardupilot_user
#USER ardupilot_user
#WORKDIR /home/ardupilot_user
# Clone ArduPilot repository
#RUN git clone --recurse-submodules https://github.com/snktshrma/ardupilot.git -b swarm-gazebo ~/ardupilot

# Set up environment and build ArduPilot SITL
RUN cd ~/ardupilot && \
    ./Tools/environment_install/install-prereqs-ubuntu.sh -y && \
    . ~/.profile && \
    ./waf configure --board=SITL && \
    ./waf build

# Clone ROS workspace repository
#RUN git clone https://github.com/guessit1000/drone.git -b gps-swarm ~/drone

# Set up ROS workspace
RUN cd ~/swarm_ap_ws && \
    rosdep install --from-path src --ignore-src -r -y && \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Set the entrypoint
ENTRYPOINT ["/bin/bash"]