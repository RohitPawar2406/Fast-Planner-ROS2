# Use the official ROS 2 Foxy image as the base
FROM ros:foxy

# Set the timezone non-interactively to India (Asia/Kolkata)
ENV DEBIAN_FRONTEND=noninteractive

# Install ros-foxy-desktop package
RUN apt-get update && apt-get install -y \
    ros-foxy-desktop \
    && rm -rf /var/lib/apt/lists/*

    # Clone NLopt repository
RUN git clone https://github.com/stevengj/nlopt.git && \
    cd nlopt && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make && \
    make install


RUN apt-get update && apt-get install -y \
    ros-foxy-cv-bridge \
    ros-foxy-visualization-msgs \
    libeigen3-dev \
    libnlopt-dev \
    libpcl-dev \
    python3-argcomplete \
    ros-dev-tools \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*
# (Optional) Set up your workspace directory
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws
# (Optional) Source your workspace setup file on container startup
CMD source /opt/ros/foxy/setup.bash && bash
