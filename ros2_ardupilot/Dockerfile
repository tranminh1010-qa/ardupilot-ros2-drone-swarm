# Start from ROS Humble base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=UTC \
    PATH="/usr/lib/ccache:$PATH" \
    MICROXRCEDDSGEN_HOME="/usr/local/share/microxrceddsgen" \
    PATH="/usr/local/bin:$PATH" \
    DISABLE_MAVNATIVE=True \
    BUILDLOGS=/tmp/buildlogs \
    XRCE_DDS_TOOL=/usr/local/bin/microxrceddsgen

# Install system dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    # Build tools
    build-essential ccache g++ gdb make cmake ninja-build \
    # Development tools
    gawk git libtool libxml2-dev libxml2-utils libxslt1-dev sudo \
    # Python tools
    python3-numpy python3-pyparsing python3-serial python-is-python3 \
    libpython3-stdlib python3-pip python3-setuptools python3-vcstool \
    python3-rosdep python3-colcon-common-extensions python3-scipy \
    # System utilities
    curl iproute2 lsb-release libtool-bin net-tools software-properties-common \
    wget xterm zip iputils-ping \
    # Java
    default-jre openjdk-11-jdk maven \
    # ROS packages
    ros-dev-tools \
    ros-humble-launch-pytest \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-mavros-msgs \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rmw-cyclonedds-cpp \
    gazebo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir -U \
    future lxml pexpect flake8 empy==3.3.4 pyelftools \
    tabulate pymavlink pre-commit junitparser mavproxy geopy ament-black

# Create workspace directory and clone repositories
WORKDIR /root
RUN mkdir -p /root/ardu_ws/src && \
    cd /root/ardu_ws/src/ && \
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git && \
    cd ardupilot && \
    git submodule update --init --recursive

# Import required repositories and install dependencies
WORKDIR /root/ardu_ws
RUN vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src && \
    cd /root/ardu_ws/src && \
    git clone https://github.com/micro-ROS/micro_ros_msgs.git && \
    rosdep update && \
    source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y || true

# Install and setup Micro-XRCE-DDS-Gen
WORKDIR /root/ardu_ws/src
RUN apt-get update && \
    git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git && \
    cd Micro-XRCE-DDS-Gen && \
    ./gradlew assemble && \
    mkdir -p /usr/local/share/microxrceddsgen/java && \
    cp build/libs/microxrceddsgen.jar /usr/local/share/microxrceddsgen/java/ && \
    cp -r scripts/* /usr/local/bin/ && \
    chmod +x /usr/local/bin/microxrceddsgen && \
    cd /root/ardu_ws/src/ardupilot && \
    mkdir -p Tools/scripts && \
    echo '#!/bin/bash\n/usr/local/bin/microxrceddsgen "$@"' > Tools/scripts/run_microxrceddsgen.sh && \
    chmod +x Tools/scripts/run_microxrceddsgen.sh

# Build ROS 2 packages
WORKDIR /root/ardu_ws
RUN  . /opt/ros/humble/setup.sh && colcon build --packages-up-to ardupilot_dds_tests

# Build ArduPilot with DDS support
WORKDIR /root/ardu_ws/src/ardupilot
RUN ./waf distclean && \
    ./waf configure --board sitl --enable-dds --debug && \
    sed -i '/MICROXRCEDDSGEN.*-default-container-prealloc-size/s/-default-container-prealloc-size [^ ]* //g' libraries/AP_DDS/wscript && \
    ./waf build -v

# Install Gazebo Sim 8 to match host version
RUN apt-get update && \
    wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null && \
    apt-get update && \
    apt-get install -y \
        # Gazebo packages
        gz-garden \
        libgz-sim8-dev \
        libgz-math7-dev \
        libgz-transport12-dev \
        libsdformat13-dev \
        libgz-cmake3-dev \
        libgz-plugin2-dev \
        # ROS-Gazebo integration
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-sim \
        ros-humble-ros-gz-interfaces \
        # Additional dependencies
        rapidjson-dev \
        ros-humble-topic-tools \
        ros-humble-ament-cmake-mypy \
        ros-humble-ament-cmake-pclint \
        ros-humble-ament-cmake-pycodestyle \
    && rm -rf /var/lib/apt/lists/*

# Import Gazebo repositories and install additional dependencies
WORKDIR /root/ardu_ws
RUN apt-get update && \
    apt-add-repository universe && \
    apt-get update && \
    apt-get install -y \
        # Gstreamer packages
        libgstreamer1.0-dev \
        libgstreamer-plugins-base1.0-dev \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-ugly \
        gstreamer1.0-libav \
        gstreamer1.0-tools \
        libzip-dev \
        # Graphics support
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        libglu1-mesa \
        x11-xserver-utils \
        libosmesa6-dev \
        xauth \
        # Missing dependencies
        python3-pip \
        socat \
    && rm -rf /var/lib/apt/lists/* && \
    pip3 install geopy && \
    vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src && \
    bash -c ". /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y -r --rosdistro humble || true && \
    colcon build --packages-up-to ardupilot_gz_bringup"

# Setup display and environment directories
RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/ardu_ws/install/setup.bash" >> ~/.bashrc

# Set environment variables for display
ENV DISPLAY=:0 \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_SOFTWARE=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root \
    XAUTHORITY=/tmp/.docker.xauth \
    XDG_SESSION_TYPE=x11 \
    PATH="/root/ardu_ws/src/ardupilot/Tools/autotest:$PATH" \
    GZ_SIM_RESOURCE_PATH=/root/ardu_ws/src/custom_gz/models

# Unset problematic environment variables and set correct Gazebo Sim environment variables
ENV GAZEBO_MASTER_URI="" \
    IGN_IP="" \
    IGN_PARTITION="" \
    ROS_DOMAIN_ID="" \
    RMW_IMPLEMENTATION="" \
    GZ_PARTITION="$(hostname)" \
    GZ_IP="127.0.0.1" \
    GZ_VERBOSE="4" \
    GZ_DISCOVERY_MULTICAST="1" \
    GZ_TRANSPORT_TOPIC_STATISTICS="1"

# Create entrypoint script with proper environment setup
WORKDIR /root/ardu_ws/
RUN echo '#!/bin/bash\n\
source /opt/ros/humble/setup.bash\n\
source /root/ardu_ws/install/setup.bash\n\
export PATH=$PATH:/root/ardu_ws/src/ardupilot/Tools/autotest\n\
\n\
# Unset problematic environment variables\n\
unset GAZEBO_MASTER_URI\n\
unset IGN_IP\n\
unset IGN_PARTITION\n\
unset ROS_DOMAIN_ID\n\
unset RMW_IMPLEMENTATION\n\
\n\
# Set correct Gazebo Sim environment variables\n\
export GZ_PARTITION=$(hostname)\n\
export GZ_IP=127.0.0.1\n\
export GZ_VERBOSE=4\n\
export GZ_DISCOVERY_MULTICAST=1\n\
export GZ_TRANSPORT_TOPIC_STATISTICS=1\n\
\n\
exec "$@"' > /root/entrypoint.sh && \
chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]