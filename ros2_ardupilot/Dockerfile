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
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir -U future lxml pexpect flake8 empy==3.3.4 pyelftools \
    tabulate pymavlink pre-commit junitparser mavproxy geopy ament-black

# Install Micro-XRCE-DDS-Gen
RUN apt-get update && apt-get install -y \
    ros-humble-ament-black \
    ros-humble-micro-ros-msgs \
    ros-humble-ament-cmake-black \
    python3-geopy \
    openjdk-11-jdk \
    maven \
    && rm -rf /var/lib/apt/lists/*

# Install and set up Micro-XRCE-DDS-Gen
RUN git clone --recursive https://github.com/eProsima/Micro-XRCE-DDS-Gen.git \
    && cd Micro-XRCE-DDS-Gen \
    && ./gradlew assemble \
    && mkdir -p /usr/local/share/microxrcedds \
    && cp build/libs/microxrceddsgen.jar /usr/local/share/microxrcedds/ \
    && cp -r share/* /usr/local/share/microxrcedds/ \
    && cp scripts/microxrceddsgen /usr/local/bin \
    && sed -i 's|${script_dir}/../share/microxrcedds/|/usr/local/share/microxrcedds/|g' /usr/local/bin/microxrceddsgen \
    && chmod +x /usr/local/bin/microxrceddsgen \
    && cd .. \
    && rm -rf Micro-XRCE-DDS-Gen

# Set up environment variables
ENV PATH="/usr/local/bin:$PATH"
ENV MICROXRCEDDSGEN_HOME="/usr/local/share/microxrcedds"

# Set up ArduPilot
RUN mkdir -p ~/ardu_ws/src \
    && cd ~/ardu_ws/src/ \
    && git clone https://github.com/ArduPilot/ardupilot.git \
    && cd ardupilot \
    && git submodule update --init --recursive \
    && cd ~/ardu_ws \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths src --ignore-src -r -y || true

# Create and modify run_microxrceddsgen.sh script
WORKDIR /root/ardu_ws/src/ardupilot
RUN mkdir -p Tools/scripts \
    && echo '#!/bin/bash\n\
/usr/local/bin/microxrceddsgen "$@"' > Tools/scripts/run_microxrceddsgen.sh \
    && chmod +x Tools/scripts/run_microxrceddsgen.sh

# Build ArduPilot
RUN ./waf distclean \
    && ./waf configure --board sitl --enable-dds --debug \
    && sed -i '/MICROXRCEDDSGEN.*-default-container-prealloc-size/s/-default-container-prealloc-size [^ ]* //g' libraries/AP_DDS/wscript \
    && ./waf build -v

ENV PATH="/root/ardu_ws/src/ardupilot/Tools/autotest:$PATH"
ENV DISABLE_MAVNATIVE=True
ENV BUILDLOGS=/tmp/buildlogs
ENV XRCE_DDS_TOOL=/usr/local/bin/microxrceddsgen

# Install ROS2 dependencies
RUN /bin/bash -c " cd ~/ardu_ws  \
    && vcs import --recursive --input  https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src \
    && rosdep update \
    && . /opt/ros/humble/setup.bash \
    && rosdep install --from-paths src --ignore-src \
    && colcon build --packages-up-to ardupilot_dds_tests"


#Install Gazebo
ENV GZ_VERSION=harmonic
# Install Gazebo Harmonic and other necessary packages
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

RUN cd ~/ardu_ws \
    && vcs import --input https://raw.githubusercontent.com/ArduPilot/ardupilot_gz/main/ros2_gz.repos --recursive src

RUN add-apt-repository universe \
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

RUN /bin/bash -c " cd ~/ardu_ws \
    && source /opt/ros/humble/setup.bash \
    && apt-get update \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -y  -r \
    && colcon build --packages-up-to ardupilot_gz_bringup"

# Source the workspace in .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "source ~/ardu_ws/install/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /root/ardu_ws/

#Hanlde graphical settings for gazebo
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    x11-xserver-utils \
    xauth \
    && rm -rf /var/lib/apt/lists/*

ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

# Change ENTRYPOINT to run a shell script
COPY --chown=ardupilot:ardupilot entrypoint.sh /home/ardupilot/entrypoint.sh
RUN chmod +x /home/ardupilot/entrypoint.sh

ENTRYPOINT ["/bin/bash", "-c"]
# Set the default command to run your entrypoint script and then keep the container running
CMD ["/home/ardupilot/entrypoint.sh && tail -f /dev/null"]