# RO47014 template repository 

This repository contains information regarding the simulation and other packages used for the course on Knowledge Representation and Reasoning RO47014 to be given in Q3 21-22 by Carlos Hernández Corbato.

## Installation

### (recommended) install using singularity image
**Please follow the instructions in the practicum 1.2 PDF**

### (optional) install locally
You can install the TIAGo simulation and our code with the following steps
First, open a terminal, create an empty workspace and clone this repository:
```bash
mkdir -p ~/ROSPlan_ws/src
cd ~/ROSPlan_ws/src
git clone https://gitlab.tudelft.nl/cor/ro47014/2023_course_projects/group_00/rosplan.git
```
### From source

Use vcs to clone the additionally required repositories:

```bash
vcs import < retail_store_skills/retail_store_skills.repos
```

>*Note*: The command above requires [vcstool](https://github.com/dirk-thomas/vcstool), if you do not have it installed you can install it with the command:
`sudo apt install python3-vcstool`

Set up **rosdep**
```bash
sudo rosdep init
rosdep update
```

Then you may run the following instruction to make sure that all dependencies referenced in the workspace are installed
```bash
cd ~/ROSPlan_ws
rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys="opencv2 opencv2-nonfree pal_laser_filters speed_limit_node sensor_to_cloud hokuyo_node libdw-dev python-graphitesend-pip python-statsd pal_filters pal_vo_server pal_usb_utils pal_pcl pal_pcl_points_throttle_and_filter pal_karto pal_local_joint_control camera_calibration_files pal_startup_msgs pal-orbbec-openni2 dummy_actuators_manager pal_local_planner gravity_compensation_controller current_limit_controller dynamic_footprint dynamixel_cpp tf_lookup opencv3"
```

Before building, run this to make sure you have all the [ROSPlan](https://github.com/KCL-Planning/ROSPlan) dependencies
```bash
sudo apt install flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet
```
Finally, build the workspace
```bash
cd ~/ROSPlan_ws
source /opt/ros/melodic/setup.bash
catkin build -DCATKIN_ENABLE_TESTING=0
source devel/setup.bash
```
From this point on it's all ready to go.

## Using the code from group 3

To run Rospplan and to load the knowledge base from the problem_place2.pddl, domain_place2.pddl and the load_product_info.py run the followin code:

```bash
singularity shell -p /path/to/ro47014-22-2.simg
source ~/ROSPlan_ws/devel/setup.bash
roslaunch retail_store_planning rosplan_place.launch
```

To create a plan and call on the plan, run in an other terminal:

```bash
singularity shell -p /path/to/ro47014-22-2.simg
source ~/ROSPlan_ws/devel/setup.bash
cd ~/ROSPlan_ws/src/rosplan/retail_store_planning/
./rosplan_executor.bash
cat  path/to/ROSPlan_ws/src/ROSplan/rosplan_planning_system/test/pddl/turtlebot/plan.pddl
```

To run the simulation run the following code in another terminal:

```bash
singularity shell -p /path/to/ro47014-23-2.sif
source ~/ro47014_ws/devel/setup.bash
roslaunch albert_gazebo albert_gazebo_navigation.launch
```

To run the pick and place server run in separate terminal: (this is needed for the simulation)

```bash
singularity shell -p /path/to/ro47014-23-2.sif
source ~/ro47014_ws/devel/setup.bash
roslaunch retail store skills load skills.launch
```