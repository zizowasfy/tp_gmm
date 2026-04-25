# Use the official ROS Noetic image as the base
FROM osrf/ros:noetic-desktop-full

# Set the working directory
WORKDIR /catkin_ws/src

# Install system dependencies, including Python and tools for GUI applications
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-moveit \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/*

# Copy the entire project into the container
COPY . /catkin_ws/src/tp_gmm

# Make Python scripts executable during the build process
RUN chmod +x /catkin_ws/src/tp_gmm/scripts/tp-gmm.py
RUN chmod +x /catkin_ws/src/tp_gmm/scripts/tp_gmm_client_test.py

# Install Python dependencies. Upgrade scipy separately to avoid version conflicts.
RUN pip3 install -r /catkin_ws/src/tp_gmm/requirements.txt
RUN pip3 install --upgrade scipy

# Build the ROS package
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make"

# Source ROS setup files in .bashrc for interactive shells
RUN echo " \
source /opt/ros/noetic/setup.bash; \
source /catkin_ws/devel/setup.bash \
" >> /root/.bashrc

# # Make the ROS entrypoint script executable
# RUN chmod +x /catkin_ws/src/tp_gmm/ros_entrypoint.sh

# # Set up the entrypoint to a custom script that starts roscore
# ENTRYPOINT ["/catkin_ws/src/tp_gmm/ros_entrypoint.sh"]

# Set the default command to launch a terminal
CMD ["bash"]