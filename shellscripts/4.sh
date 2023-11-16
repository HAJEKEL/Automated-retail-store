#!/bin/bash

# Set the ROS_PACKAGE_PATH to your workspace source directory
ROS_PACKAGE_PATH=ro47014_ws/src:$ROS_PACKAGE_PATH

# Export the ROS_PACKAGE_PATH environment variable
export ROS_PACKAGE_PATH

# Source the ROS environment
source /opt/ros/noetic/setup.bash

# Source your workspace
source ~/ro47014_ws/devel/setup.bash

# Run your launch file
roslaunch retail_store_skills load_skills.launch
