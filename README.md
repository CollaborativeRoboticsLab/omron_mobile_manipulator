# Omron_MoMa_ROS2

Original packages are from [OmronAPAC](https://github.com/OmronAPAC) 

This repository allows controlling a Omron MoMa (Omron Mobile Manipulator) using packages that bridges,

- [Omron_AMR_ROS2](https://github.com/CollaborativeRoboticsLab/Omron_AMR_ROS2) package 
- [Omron_TM_ROS2](https://github.com/CollaborativeRoboticsLab/Omron_TM_ROS2) package

For supported features and limitations, see the individual repositories on the features supported by the MoMa.

## Setup

Create a workspace

```sh
mkdir -p omron_ws/src
cd omron_ws/src
```

Install dependencies
```sh
sudo apt install ros-humble-moveit ros-humble-controller-manager ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster ros-humble-rmw-cyclonedds-cpp
```

Clone the repositories into the `src` folder by

```sh
git clone https://github.com/CollaborativeRoboticsLab/Omron_MoMa_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_AMR_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_TM_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_TM_CORE_ROS2.git -b humble
```

Then install dependencies by 

```sh
cd ..
rosdep install --from-paths src -y --ignore-src
```

# Docker

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