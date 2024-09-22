# syntax=docker/dockerfile:1

# Stage 1: Base image with ROS2 and common dependencies
FROM ros:humble-ros-base AS base
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV DISABLE_MAVNATIVE=True
ENV BUILDLOGS=/tmp/buildlogs
ENV SDL_AUDIODRIVER=dummy
ENV ALSA_CONFIG_PATH=/usr/share/alsa/alsa.conf.example
ENV ALSOFT_CONF=/root/alsoft.conf

RUN echo "drivers=null" > /root/alsoft.conf

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release sudo wget software-properties-common build-essential ccache \
    g++ gdb gawk git make cmake ninja-build libtool libxml2-dev libxml2-utils \
    libxslt1-dev python3-numpy python3-pyparsing python3-serial python-is-python3 \
    libpython3-stdlib libtool-bin zip default-jre socat ros-dev-tools \
    ros-humble-launch-pytest gazebo ros-humble-gazebo-ros-pkgs iputils-ping \
    ros-humble-mavros-msgs python3-pip python3-setuptools libasio-dev \
    python3-vcstool python3-rosdep python3-colcon-common-extensions \
    ros-humble-rmw-fastrtps-cpp python3-scipy \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -U future lxml pexpect flake8 empy==3.3.4 pyelftools \
    tabulate pymavlink pre-commit junitparser mavproxy geopy ament-black

# Stage 2: Build and install Fast-CDR, foonathan_memory_vendor, and Fast-DDS
FROM base AS dds_builder
RUN git clone https://github.com/eProsima/Fast-CDR.git \
    && cd Fast-CDR \
    && mkdir build && cd build \
    && cmake .. \
    && cmake --build . --target install \
    && cd ../.. && rm -rf Fast-CDR

RUN git clone https://github.com/eProsima/foonathan_memory_vendor.git \
    && cd foonathan_memory_vendor \
    && mkdir build && cd build \
    && cmake .. \
    && cmake --build . --target install \
    && cd ../.. && rm -rf foonathan_memory_vendor

RUN git clone https://github.com/eProsima/Fast-DDS.git \
    && cd Fast-DDS \
    && mkdir build && cd build \
    && cmake .. \
    && cmake --build . --target install \
    && cd ../.. && rm -rf Fast-DDS

# Stage 3: Install ARM toolchain
FROM base AS arm_toolchain
ARG ARM_ROOT="gcc-arm-none-eabi-10"
ARG ARM_ROOT_EXT="-2020-q4-major"
ARG ARM_TARBALL="$ARM_ROOT$ARM_ROOT_EXT-x86_64-linux.tar.bz2"
ARG ARM_TARBALL_URL="https://firmware.ardupilot.org/Tools/STM32-tools/$ARM_TARBALL"

RUN wget -qO- "$ARM_TARBALL_URL" | tar jx -C /opt \
    && mv "/opt/$ARM_ROOT$ARM_ROOT_EXT" "/opt/$ARM_ROOT" \
    && rm -rf "/opt/$ARM_ROOT/share/doc" \
    && ln -s /usr/bin/ccache /usr/lib/ccache/arm-none-eabi-g++ \
    && ln -s /usr/bin/ccache /usr/lib/ccache/arm-none-eabi-gcc \
    && mkdir -p $HOME/arm-gcc \
    && ln -s -f /opt/$ARM_ROOT $HOME/arm-gcc/g++-10.2.1

# Stage 4: Final image
FROM base AS final
COPY --from=dds_builder /usr/local /usr/local
COPY --from=arm_toolchain /opt /opt
COPY --from=arm_toolchain /usr/lib/ccache /usr/lib/ccache
COPY --from=arm_toolchain /root/arm-gcc /root/arm-gcc

ENV PATH="/opt/gcc-arm-none-eabi-10/bin:/usr/lib/ccache:/usr/local/bin:$PATH"
ENV PATH="/root/ardu_ws/src/ardupilot/Tools/autotest:${PATH}"

RUN mkdir -p ~/ardu_ws/src \
    && cd ~/ardu_ws \
    && vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths ~/ardu_ws/src --ignore-src -r -y || true \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc \
    && echo "if [ -f ~/ardu_ws/install/setup.bash ]; then source ~/ardu_ws/install/setup.bash; fi" >> ~/.bashrc \
    && echo "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/usr/local/share/fastcdr:/usr/local/share/fastrtps" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
WORKDIR /root/ardu_ws

ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1

RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]