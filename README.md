# Retail Store Simulation (KRR version)

Common ROS packages for the Airlab Albert platform.

## Using singularity (recommended)

Run the provided singularity image:
```bash
TODO
```

### Run the simulation

```bash
roslaunch albert_gazebo albert_gazebo_navigation.launch
```

## Install locally (optional)

Clone repository and install dependencies using [vcstool](https://github.com/dirk-thomas/vcstool):
``` bash
mkdir -p albert_ws/src && cd albert_ws/src
git clone git@gitlab.tudelft.nl:airlab-delft/ng-staging/albert/albert.git

# If not installed yet
sudo apt install python3-vcstool

vcs import --input albert/albert_{melodic, noetic}.rosinstall .
```

Next, install all system dependencies that packages in `albert_ws` depend upon but are missing on your computer.

```bash
# If not installed yet
sudo apt install python3-rosdep

sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Finally, build the packages using catkin.
**Important**: Make sure that any python virtual environments are deactivated!
```bash
# If not installed yet
sudo apt install python3-catkin-tools
catkin build
source devel/setup.bash
```

