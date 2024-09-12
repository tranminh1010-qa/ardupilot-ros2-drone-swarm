#!/bin/bash


set -e

GREEN='\033[0;32m'
NC='\033[0m'

print_message() {
    echo -e "${GREEN}$1${NC}"
}

print_message "Updating package list and upgrading installed packages..."
sudo apt-get update -y
sudo apt-get upgrade -y

print_message "Installing dependencies..."
sudo apt-get install -y git python3-pip python3-dev build-essential

print_message "Cloning ArduPilot repository..."
git clone --recurse-submodules https://github.com/snktshrma/ardupilot.git -b swarm-gazebo ~/ardupilot

print_message "Installing required Python packages..."
pip3 install numpy scipy matplotlib -U

print_message "Setting up environment..."
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y

print_message "Building ArduPilot SITL..."
. ~/.profile
./waf configure --board=SITL
./waf build

print_message "SITL setup is complete!"

cd ..

print_message "Installing ROS ws"

git clone https://github.com/guessit1000/drone.git -b gps-swarm

cd drone/swarm_ap_ws

rosdep install --from-path src --ignore-src -r -y

catkin build
