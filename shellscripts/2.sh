#!/bin/bash

# Set the ROS_PACKAGE_PATH to your workspace source directory
ROS_PACKAGE_PATH=ro47014_ws/src:$ROS_PACKAGE_PATH

# Export the ROS_PACKAGE_PATH environment variable
export ROS_PACKAGE_PATH

# Source the ROS environment
source /opt/ros/melodic/setup.bash

# Source your workspace
source ~/ROSPlan_ws/devel/setup.bash

# Run your shell script
cd ~/ROSPlan_ws/src/rosplan/retail_store_planning/
./rosplan_executor.bash

