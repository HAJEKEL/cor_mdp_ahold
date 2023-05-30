# Retail Store Skills

This repository contains an example set of skills for the course Knowledge Representation and Reasoning (KRR).

## Install

### Activate Singularity Environment (Recommended)
TODO

### Clone repo + dependencies
```bash
# Create a catkin workspace or cd into an existing one
mkdir -p krr_ws/src
cd krr_ws/src

# Clone this repository
# Make sure to change the <XX> into your group number.
# Also make sure you have added your public ssh key to your tudelft gitlab account, as was explained in the pdf at the instruction session.
git clone git@gitlab.tudelft.nl:cor/ro47014/2022_course_projects/group_<XX>/retail_store_skills.git

# Clone dependencies for tag detection.
git clone https://github.com/AprilRobotics/apriltag.git
git clone https://github.com/AprilRobotics/apriltag_ros.git
```

## Example

Make sure you first activate the singularity environment and source your workspace for every new terminal instance.

To test a pick skill first launch the simulation:

```bash
roslaunch albert_gazebo albert_gazebo_navigation.launch
```

In another terminal window launch the skills:

```bash
roslaunch retail_store_skills load_skills.launch
```

In yet another terminal window, wait until moveit is fully initialized in the simulation then run the following to execute a pick action:
```bash
rosrun actionlib_tools axclient.py /pick_server
```

A GUI will pop up in which you can specify which apriltag id you would like to pick. For example id 18 is reachable from the initial position.
