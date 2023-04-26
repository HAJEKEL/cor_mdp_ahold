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
COPY . retail_store_simulation
RUN vcs import --input retail_store_simulation/retail_store_simulation.rosinstall .
WORKDIR $WS_BASE

# ===== Use Rosdep to install known global system dependencies =====
RUN apt-get update \
 && rosdep init || true \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src --rosdistro noetic -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# ===== All dependencies are installed so the WS_BASE can be removed =====
WORKDIR /
RUN rm -rf $WS_BASE
