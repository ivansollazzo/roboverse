#!/bin/bash

WORKING_DIR=$(pwd)

source install/setup.bash
ros2 launch ./roboverse.launch.py
