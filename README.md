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

Clone the repositories into the `src` folder by

```sh
git clone https://github.com/CollaborativeRoboticsLab/Omron_MoMa_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_AMR_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_TM_ROS2.git
```

Then install dependencies by 

```sh
cd ..
rosdep install --from-paths src -y --ignore-src
```
