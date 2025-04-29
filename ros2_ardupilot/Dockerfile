# Start from ROS Humble base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV PATH="/usr/lib/ccache:$PATH"

# Install system dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release sudo wget software-properties-common build-essential ccache \
    g++ gdb gawk git make cmake ninja-build libtool libxml2-dev libxml2-utils \
    libxslt1-dev python3-numpy python3-pyparsing python3-serial python-is-python3 \
    libpython3-stdlib libtool-bin zip default-jre socat ros-dev-tools \
    ros-humble-launch-pytest gazebo ros-humble-gazebo-ros-pkgs iputils-ping \
    ros-humble-mavros-msgs python3-pip python3-setuptools libasio-dev \
    python3-vcstool python3-rosdep python3-colcon-common-extensions \
    ros-humble-rmw-fastrtps-cpp python3-scipy curl \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rmw-cyclonedds-cpp \
    openjdk-11-jdk \
    maven \
    xterm \
    default-jre \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir -U future lxml pexpect flake8 empy==3.3.4 pyelftools \
    tabulate pymavlink pre-commit junitparser mavproxy geopy ament-black

# Create workspace directory
WORKDIR /root
RUN mkdir -p /root/ardu_ws/src

# Import the required repositories for ROS 2 integration using vcs
WORKDIR /root/ardu_ws
# Complete ArduPilot build
RUN cd ~/ardu_ws/src/ \
    && git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive \
    && . ~/.profile


RUN vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src

# Install rosdep dependencies
RUN rosdep update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Install Micro-XRCE-DDS-Gen build dependency
WORKDIR /root/ardu_ws/src
RUN apt-get update && apt-get install -y && \
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git && \
    cd Micro-XRCE-DDS-Gen && \
    ./gradlew assemble && \
    mkdir -p /usr/local/share/microxrceddsgen/java && \
    cp build/libs/microxrceddsgen.jar /usr/local/share/microxrceddsgen/java/ && \
    cp -r scripts/* /usr/local/bin/ && \
    chmod +x /usr/local/bin/microxrceddsgen

# Set environment variable for microxrceddsgen
ENV PATH="/usr/local/bin:$PATH"
ENV MICROXRCEDDSGEN_HOME="/usr/local/share/microxrceddsgen"

WORKDIR /root/ardu_ws/src
RUN git clone https://github.com/micro-ROS/micro_ros_msgs.git



RUN cd ~/ardu_ws \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src --ignore-src -r -y || true

# Build ROS 2 packages
WORKDIR /root/ardu_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-up-to ardupilot_dds_tests"


# Create helper script for microxrceddsgen
WORKDIR /root/ardu_ws/src/ardupilot
RUN mkdir -p Tools/scripts && \
    echo '#!/bin/bash\n\
/usr/local/bin/microxrceddsgen "$@"' > Tools/scripts/run_microxrceddsgen.sh && \
    chmod +x Tools/scripts/run_microxrceddsgen.sh

# Build ArduPilot with DDS support
RUN ./waf distclean && \
    ./waf configure --board sitl --enable-dds --debug && \
    sed -i '/MICROXRCEDDSGEN.*-default-container-prealloc-size/s/-default-container-prealloc-size [^ ]* //g' libraries/AP_DDS/wscript

RUN ./waf build -v # || (echo "Build failed with detailed error:" && cat build/sitl/libraries/AP_DDS/AP_DDS_Client.cpp.0.o.err || true)

# Set environment variables for Ardupilot
ENV PATH="/root/ardu_ws/src/ardupilot/Tools/autotest:$PATH"
ENV DISABLE_MAVNATIVE=True
ENV BUILDLOGS=/tmp/buildlogs
ENV XRCE_DDS_TOOL=/usr/local/bin/microxrceddsgen
# Install Gazebo Harmonic
ENV GZ_VERSION=harmonic
RUN apt-get update && apt-get install -y \
    lsb-release wget gnupg \
    && wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
        gz-harmonic \
        libgz-sim7-dev \
        libgz-math7-dev \
        libgz-transport12-dev \
        libsdformat13-dev \
        libgz-cmake3-dev \
        libgz-plugin2-dev \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-interfaces \
        rapidjson-dev \
        ros-humble-topic-tools \
        ros-humble-ament-cmake-mypy \
        ros-humble-ament-cmake-pclint \
        ros-humble-ament-cmake-pycodestyle \
    && rm -rf /var/lib/apt/lists/*

# Import Gazebo repositories
RUN cd /root/ardu_ws \
    && vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

# Install additional dependencies
RUN add-apt-repository universe \
    && apt-get update \
    && apt-get install -y python3-colcon-common-extensions \
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-tools \
        libzip-dev

# Build Gazebo packages
RUN /bin/bash -c "cd /root/ardu_ws \
    && source /opt/ros/humble/setup.bash \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y -r \
    && colcon build --packages-up-to ardupilot_gz_bringup"

# Source the workspace in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source /root/ardu_ws/install/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /root/ardu_ws/

# Handle graphical settings for Gazebo
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    x11-xserver-utils \
    libosmesa6-dev \
    xauth \
    && rm -rf /var/lib/apt/lists/*

# Set environment variables for display
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root
ENV XAUTHORITY=/tmp/.docker.xauth
ENV XDG_SESSION_TYPE=x11
ENV XDG_DATA_DIRS=/usr/local/share:/usr/share:/var/lib/snapd/desktop
ENV XDG_CONFIG_DIRS=/etc/xdg/xdg-ubuntu:/etc/xdg
ENV XDG_CURRENT_DESKTOP=ubuntu:GNOME
ENV XDG_SESSION_DESKTOP=ubuntu
ENV XDG_SESSION_CLASS=user
ENV XDG_SESSION_ID=1
ENV XDG_SESSION_COOKIE=1
ENV XDG_SEAT=seat0
ENV XDG_VTNR=2
ENV XDG_GREETER_DATA_DIR=/var/lib/lightdm-data/root
ENV XDG_CONFIG_HOME=/root/.config
ENV XDG_CACHE_HOME=/root/.cache
ENV XDG_DATA_HOME=/root/.local/share
# Set the GZ_SIM_RESOURCE_PATH environment variables
ENV GZ_SIM_RESOURCE_PATH=/root/ardu_ws/src/custom_gz/models


RUN mkdir -p $XDG_RUNTIME_DIR && chmod 700 $XDG_RUNTIME_DIR

# Create entrypoint script
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/ardu_ws/install/setup.bash\n\
export PATH=$PATH:/root/ardu_ws/src/ardupilot/Tools/autotest\n\
exec "$@"' > /root/entrypoint.sh && \
chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]