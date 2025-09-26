# Omron_MoMa_ROS2

Original packages are from [OmronAPAC](https://github.com/OmronAPAC) 

This repository allows controlling the Base and Arm of the Omron Mobile Manipulator using packages,

- [omron_arm](https://github.com/CollaborativeRoboticsLab/omron_arm) package 
- [omron_base](https://github.com/CollaborativeRoboticsLab/omron_base) package

For supported features and limitations, see the individual repositories on the features supported by the MoMa.

## Setup

Create a workspace

```sh
mkdir -p omron_ws/src
cd omron_ws/src
```

Install dependencies
```sh
sudo apt install ros-humble-moveit ros-humble-controller-manager ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster ros-humble-rmw-cyclonedds-cpp ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-vision-opencv ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox
```

Clone the repositories into the `src` folder by

```sh
git clone https://github.com/CollaborativeRoboticsLab/omron_arm.git
git clone https://github.com/CollaborativeRoboticsLab/omron_base.git
git clone https://github.com/CollaborativeRoboticsLab/omron_mobile_manipulator.git
```

finally build by

```sh
cd ..
colcon build
```

**or save time and use devcontainer or docker image** 

## Docker

Clone this reposiotory

```bash
git clone https://github.com/CollaborativeRoboticsLab/Omron_MoMa_ROS2.git 
cd Omron_MoMa_ROS2/docker
```

Pull the Docker image and start compose (No need to run `docker compose build`)
```bash
docker compose pull
docker compose up
```

To clean the system,
```bash
docker compose down
```

