#!/bin/bash
sudo rm -rf build/ install/ log/
colcon build
source install/setup.bash