#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start base image
#----
#---------------------------------------------------------------------------------------------------------------------------

# Use a base image suitable for Raspberry Pi with ROS 2 Humble
FROM ros:humble-ros-base-jammy AS base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE_ROOT=/omron_ws

#############################################################################################################################
#####
#####   Install Dependencies
#####
#############################################################################################################################

# Install dependencies
RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    build-essential \
    curl \
    ros-humble-moveit \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-vision-opencv


RUN apt-get clean

#############################################################################################################################
#####
#####   Clone and build the workspace
#####
#############################################################################################################################

# Create and clone workspace
WORKDIR ${WORKSPACE_ROOT}/src

RUN git clone https://github.com/CollaborativeRoboticsLab/Omron_MoMa_ROS2.git
RUN git clone https://github.com/CollaborativeRoboticsLab/Omron_AMR_ROS2.git 
RUN git clone https://github.com/CollaborativeRoboticsLab/Omron_TM_ROS2.git
RUN git clone https://github.com/CollaborativeRoboticsLab/Omron_TM_CORE_ROS2.git -b humble

WORKDIR ${WORKSPACE_ROOT}

# Build the workspace
RUN . /opt/ros/humble/setup.sh && colcon build

WORKDIR /

#############################################################################################################################
#####
#####   Remove workspace source and build files that are not relevent to running the system
#####
#############################################################################################################################

RUN rm -rf ${WORKSPACE_ROOT}/src
RUN rm -rf ${WORKSPACE_ROOT}/log
RUN rm -rf ${WORKSPACE_ROOT}/build

RUN rm -rf /var/lib/apt/lists/*
RUN rm -rf /tmp/*
RUN apt-get clean


#---------------------------------------------------------------------------------------------------------------------------
#----
#----   Start final release image
#----
#---------------------------------------------------------------------------------------------------------------------------

FROM ros:humble-ros-base-jammy AS final

## Parameters
ENV WORKSPACE_ROOT=/navstack
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

WORKDIR /

COPY --from=base / /

COPY docker/ros_entrypoint.sh ros_entrypoint.sh

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
