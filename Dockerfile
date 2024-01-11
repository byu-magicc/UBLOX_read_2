# Base image
ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

# Update system
RUN apt update
RUN apt upgrade -y
RUN rosdep update

# Install utilities/packages
RUN apt install\
    ros-dev-tools\
    vim\
    -y

# Set working directory
WORKDIR /ublox_read_2_ws

# Install ros package dependencies
COPY . src/ublox_read_2/
RUN rosdep install --from-paths . --ignore-src -y

# Build packages
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build"

# Setup ROS environment variables
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /ublox_read_2_ws/install/setup.bash" >> /root/.bashrc
