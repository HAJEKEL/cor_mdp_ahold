# Retail Store Simulation (MDP 2023 version)

Common ROS packages for the simulation of the Airlab Albert platform.

**IMPORTANT**: For people familiar with the simulation from the KRR course in the previous quarter, we will not be using a singularity environment this time. This is because singularity containers are write-only and for the MDP course we expect you will need the ability to install additional dependencies. Instead we will provide you with a template for a Dockerfile, or suggest you to install things locally.

## Installation

### Option 1: Full local environment
For this option it's required to have ubuntu 20.04, ROS noetic and moveit installed on your computer.

We use vcstool to clone git repositories of dependencies that should be built from source:
``` bash
# Inside your catkin workspace
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-0/cor_mdp_ahold.git

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

### Option 2: Docker environment

We use vcstool to clone git repositories of dependencies that should be built from source:
``` bash
# Inside your catkin workspace
git clone git@gitlab.tudelft.nl:cor/ro47007/2023/team-0/cor_mdp_ahold.git

# If not installed yet
sudo apt install python3-vcstool

vcs import --input cor_mdp_ahold/retail_store_simulation.rosinstall .
```

Next we build and activate the docker environment.
```bash
docker build -t retail_store_simulation .

# allow GUI's from docker by adding x server hosts
xhost + local:docker

docker run -it -v <catkin_ws>:/catkin_ws -e DISPLAY=$DISPLAY --device /dev/dri:/dev/dri --net host retail_store_simulation
```

Finally, we build the packages using catkin.
```bash
cd /catkin_ws && catkin build
source devel/setup.bash
```

## Simulation

To start the simulation environment:

```bash
roslaunch albert_gazebo albert_gazebo_navigation.launch
```

The default simulation world was changed to the `AH_store`
