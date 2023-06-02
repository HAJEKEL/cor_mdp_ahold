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

To get started with the Supermarket Order Picking Robot, follow the installation and setup instructions provided below. It covers the necessary hardware requirements, software dependencies, and configuration steps. Once you've completed the setup, you can run the simulation and interact with the order interface.

## Full local environment
For this option it's required to have ubuntu 20.04, ROS noetic and moveit installed on your computer.

First, we create a fresh catkin workspace. This tutorial assumes that you have installed catkin. 
```bash
#source your environment:
 source /opt/ros/noetic/setup.bash

#create and build a catkin workspace:
 mkdir -p ~/catkin_ws/src
 cd ~/catkin_ws/
 cd src
```

We use vcstool to clone git repositories of dependencies that should be built from source:
``` bash
# Inside your catkin workspace
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-20/cor_mdp_ahold.git

# clone the FlexBE behavior engine and the states and behavior we made:
git clone https://github.com/team-vigir/flexbe_behavior_engine.git
git clone https://github.com/FlexBE/flexbe_app.git

# Clone our made states and behaviors into repository
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-20/albert_flexbe.git 

# clone franka_ros and apriltag_ros repositories
git clone https://github.com/frankaemika/franka_ros.git
git clone https://github.com/AprilRobotics/apriltag_ros.git 

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

