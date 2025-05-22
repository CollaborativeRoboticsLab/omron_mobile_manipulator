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
sudo apt install ros-humble-moveit ros-humble-controller-manager ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster ros-humble-rmw-cyclonedds-cpp ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-vision-opencv
```

Clone the repositories into the `src` folder by

```sh
git clone https://github.com/CollaborativeRoboticsLab/Omron_MoMa_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_AMR_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_TM_ROS2.git
git clone https://github.com/CollaborativeRoboticsLab/Omron_TM_CORE_ROS2.git -b humble
```

finally buildby

```sh
cd ..
colcon build
```

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

## Usage 

### TM Robot with TMFlow

1. [Startup TMFLow software with a listener node](https://github.com/CollaborativeRoboticsLab/tmr_ros2?tab=readme-ov-file#-tmflow-listen-node-setup)

2. [Establish Remote connection to TM Robot](https://github.com/CollaborativeRoboticsLab/tmr_ros2?tab=readme-ov-file#-remote-connection-to-tm-robot)

3. Start TM Driver

```sh
source install/setup.bash
ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>
```

### TM Robot Arm with Moveit 

TM driver node is included in the tm12x_run_move_group.launch.py file.
```sh
source install/setup.bash
ros2 launch tm12x_moveit_config tm12x_run_move_group.launch.py robot_ip:=<robot_ip_address>
```

### TM Robot Arm with Moveit (Simulation)

TM driver node is included in the tm12x_run_move_group.launch.py file.
```sh
source install/setup.bash
ros2 launch tm12x_moveit_config tm12x_run_move_group.launch.py
```


## Usage with docker

### TM Robot with TMFlow

1. [Startup TMFLow software with a listener node](https://github.com/CollaborativeRoboticsLab/tmr_ros2?tab=readme-ov-file#-tmflow-listen-node-setup)

2. [Establish Remote connection to TM Robot](https://github.com/CollaborativeRoboticsLab/tmr_ros2?tab=readme-ov-file#-remote-connection-to-tm-robot)

3. Start TM Driver by uncommenting the following line under `command` on the docker compose.yaml file and then running `docker compose up`

```yaml
command:
  - ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>
```

### TM Robot Arm with Moveit 

1. TM driver node is included in the tm12x_run_move_group.launch.py file. To start the headless moveit server, uncomment the following line on the companion computer

    ```yaml
    command:
    - ros2 launch tm12x_moveit_config tm12x_run_move_group_headless.launch.py robot_ip:=<robot_ip_address>
    ```

    and run

    ```sh
    docker compose up
    ```

2. On the remote computer run the following command,
    ```sh
    source install/setup.bash
    ros2 launch tm12x_moveit_config tm12x_run_move_group_visualize.launch.py
    ```