# Retail Store Simulation (MDP 2023 version)

Common ROS packages for the simulation of the Airlab Albert platform.

**IMPORTANT**: For people familiar with the simulation from the KRR course in the previous quarter, we will not be using a singularity environment this time. This is because singularity containers are write-only and for the MDP course we expect you will need the ability to install additional dependencies. Instead we will provide you with a template for a Dockerfile, or suggest you to install things locally.

## Overview

Welcome to the Supermarket Order Picking Robot project! This repository contains the software stack developed to enable a robot to navigate supermarket aisles, pick and pack customer orders, and provide assistance to shoppers. Our goal is to enhance the shopping experience by improving order accuracy, efficiency, and customer satisfaction.

### Key Features

- Order Picking: The robot autonomously navigates supermarket aisles, identifies the items in customer orders, and picks them from shelves with precision.
- Customer Support: The robot interacts with customers, guiding them to specific product locations, answering queries, and providing recommendations based on customer preferences.
- Intuitive Interface: We've designed a user-friendly local-based interface for creating customer orders.
- Collaborative Planning: The software leverages advanced algorithms for task allocation, path planning, and collision avoidance to optimize the overall order picking process and ensure smooth robot operation.
- Integration with Supermarket Systems: Our solution integrates with existing supermarket systems, such as inventory management and point-of-sale, ensuring seamless order processing and synchronization.
- Fully implemented FlexBE behavior engine allowing complex behavior for the robot. 


## Installation

Our code is made to be run on Ubuntu 20.04 with [ROS noetic](http://wiki.ros.org/noetic/Installation). Also, make sure that the [moveit](https://moveit.ros.org/install/) library is installed, as well as the catkin build tool.

First, we create a fresh catkin workspace.
```bash
#source your environment:
 source /opt/ros/noetic/setup.bash

#create and build a catkin workspace:
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/
 cd src
```

Now, clone the required repositories into the `src` folder of your catkin workspace.
We use vcstool to install required dependencies that should be built from source:
``` bash
# Inside your catkin workspace
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-20/cor_mdp_ahold.git

# clone the FlexBE behavior engine and the states and behavior we made. 
# We used FlexBE to create a state machine model so that the robot can complete entire tasks without the need for running separate scripts. 
# More information can be found in the readme in the albert_flexbe_git repository.
git clone https://github.com/team-vigir/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/flexbe_app.git

# Clone our self-made states and behaviors into repository
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-20/albert_flexbe.git flex_albert_behaviors

# clone franka_ros and apriltag_ros repositories
git clone https://github.com/frankaemika/franka_ros.git
git clone https://github.com/AprilRobotics/apriltag_ros.git 
git clone https://github.com/AprilRobotics/apriltag.git 

# If not installed yet
sudo apt install python3-vcstool

vcs import --input cor_mdp_ahold/retail_store_simulation.rosinstall .
```

Next, we install all system dependencies that the packages in our workspace depend upon but are missing on your computer.

```bash
# If not installed yet
sudo apt install python3-rosdep

sudo rosdep init
rosdep update
rosdep install --from-paths . --ignore-src -y
```

Finally, we build the packages using catkin.
**Important**: Make sure that any python virtual environments are deactivated!
```bash
# If not installed yet
sudo apt install python3-catkin-tools
cd .. && catkin build
source devel/setup.bash
```

## Simulation

To start the simulation environment:

```bash
roslaunch albert_gazebo albert_gazebo_navigation.launch
```

The default simulation world was changed to the `AH_store`

