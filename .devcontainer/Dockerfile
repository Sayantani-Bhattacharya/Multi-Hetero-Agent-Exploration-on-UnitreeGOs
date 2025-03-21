FROM althack/ros2:humble-gazebo 


ENV DEBIAN_FRONTEND=noninteractive


# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc


RUN apt-get update \
 && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git-all \
    libusb-1.0-0 \
    udev \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*


RUN wget https://github.com/IntelRealSense/librealsense/raw/master/scripts/libuvc_installation.sh

RUN chmod +x libuvc_installation.sh

RUN /bin/bash -c "./libuvc_installation.sh"
 
RUN apt-get update \
 && apt-get install -y \ 
   #  ros-humble-librealsense2* \
    ros-humble-realsense2-* \
 && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
 && apt-get install -y ros-humble-rtabmap-ros \
 && apt-get install -y ros-humble-turtlebot3* \
 && apt-get install -y usbutils \
 && rm -rf /var/lib/apt/lists/*


RUN /bin/bash -c "source /opt/ros/humble/setup.bash"

ENV DEBIAN_FRONTEND=dialog

CMD [ "rs-enumerate-devices", "--compact" ]