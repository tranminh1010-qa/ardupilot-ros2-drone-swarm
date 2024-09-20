# syntax=docker/dockerfile:1
FROM ros:humble-ros-base AS base

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV DISABLE_MAVNATIVE=True
ENV BUILDLOGS=/tmp/buildlogs
ENV PYTHON_PIP_VERSION=23.1.2
ENV PYTHON_SETUPTOOLS_VERSION=65.5.1
ENV PYTHON_GET_PIP_URL=https://github.com/pypa/get-pip/raw/9af82b715db434abb94a0a6f3569f43e72157346/public/get-pip.py
ENV PYTHON_GET_PIP_SHA256=45a2bb8bf2bb5eff16fdd00faef6f29731831c7c59bd9fc2bf1f3bed511ff1fe

# Add these lines to suppress audio-related errors
ENV SDL_AUDIODRIVER=dummy
ENV ALSA_CONFIG_PATH=/usr/share/alsa/alsa.conf.example
ENV ALSOFT_CONF=/root/alsoft.conf

# Create the OpenAL configuration file
RUN echo "drivers=null" > /root/alsoft.conf

RUN apt-get update && apt-get install --no-install-recommends -y \
    lsb-release sudo wget software-properties-common build-essential ccache \
    g++ gdb gawk git make cmake ninja-build libtool libxml2-dev libxml2-utils \
    libxslt1-dev python3-numpy python3-pyparsing python3-serial python-is-python3 \
    libpython3-stdlib libtool-bin zip default-jre socat ros-dev-tools \
    ros-humble-launch-pytest gazebo ros-humble-gazebo-ros-pkgs iputils-ping \
    ros-humble-mavros-msgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN wget -O get-pip.py "$PYTHON_GET_PIP_URL" \
    && echo "$PYTHON_GET_PIP_SHA256 *get-pip.py" | sha256sum -c - \
    && python get-pip.py --disable-pip-version-check --no-cache-dir --no-compile \
       "pip==$PYTHON_PIP_VERSION" "setuptools==$PYTHON_SETUPTOOLS_VERSION" \
    && rm -f get-pip.py \
    && pip install --no-cache-dir -U future lxml pexpect flake8 empy==3.3.4 pyelftools \
       tabulate pymavlink pre-commit junitparser

FROM eclipse-temurin:19-jdk-jammy AS dds-gen-builder

RUN apt-get update && apt-get install --no-install-recommends -y git \
    && rm -rf /var/lib/apt/lists/* \
    && git clone -b master --recurse-submodules --depth 1 \
       https://github.com/ArduPilot/Micro-XRCE-DDS-Gen.git dds-gen \
    && cd dds-gen \
    && ./gradlew assemble

FROM base AS final

COPY --from=dds-gen-builder /dds-gen/scripts /dds-gen/scripts
COPY --from=dds-gen-builder /dds-gen/share /dds-gen/share

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
    && ln -s -f /opt/gcc-arm-none-eabi-10/ g++-10.2.1

ENV PATH="/opt/$ARM_ROOT/bin:/dds-gen/scripts:/usr/lib/ccache:$PATH"

RUN mkdir -p ~/ardu_ws/src \
    && cd ~/ardu_ws \
    && until ping -c1 github.com; do echo "Waiting for internet connection..."; sleep 5; done \
    && until vcs import --recursive --input https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos src; do echo "Retrying vcs import..."; sleep 5; done \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths ~/ardu_ws/src --ignore-src -r -y || true \
    && apt-get update && apt-get install -y python3-ament-package \
    && rosdep install --from-paths ~/ardu_ws/src --ignore-src -r -y || true \
    && echo "export PATH=\$PATH:$PWD/scripts" >> ~/.bashrc \
    && . ~/.bashrc \
    && export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/ros/humble/share/:/opt/ros/humble/share/fastcdr:/opt/ros/humble/share/mavros_msgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root/ardu_ws

# Set up environment for Gazebo
ENV DISPLAY=:0
ENV QT_X11_NO_MITSHM=1