FROM moveit/moveit:noetic-release

# Use the "noninterace" debconf frontend
ENV DEBIAN_FRONTEND noninteractive

# ===== Install essentials =====
RUN apt-get update 
RUN apt-get install -y -q \
  python3-catkin-tools \
  python3-vcstool git \
  libgl1-mesa-glx libgl1-mesa-dri

# ===== Install user packages =====
RUN apt-get install -y -q \
  vim nano tmux tmuxp ranger htop

# ===== Clone dependencies to be built from source =====
ARG WS_BASE=/albert_ws
WORKDIR $WS_BASE/src
ADD . albert
RUN vcs import --input albert/albert_noetic.rosinstall .
WORKDIR $WS_BASE

# ===== Use Rosdep to install known global system dependencies =====
RUN apt-get update \
 && rosdep init || true \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src --rosdistro noetic -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN bash -c "source /opt/ros/noetic/setup.bash && catkin build"

WORKDIR /
SHELL [ "/bin/bash" , "-c" ]

# Define entrypoint
COPY ./docker-entrypoint.sh /
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]

## IMPORTANT:
# make sure to run 'xhost +local:' first if you're using nvidia-docker
