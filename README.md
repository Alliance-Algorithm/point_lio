## PointLio For Sentry

### Introduction

### Install

#### Environment

- `ubuntu-server` 22.04.4
- `ros:humble` in docker
- livox mid-360 lidar

#### Dependencies

1. you can **manually** download all dependencies

- livox-ros-driver2

    [how to install](https://github.com/Livox-SDK/livox_ros_driver2)

- pcl

    ```sh
    sudo apt-get install ros-humble-pcl-conversions
    ```

- eigen

    ```sh
    sudo apt-get install libeigen3-dev
    ```
2. or you can use one-click dependency installation of ros2

    ```sh
    # entry you workspace
    cd /path/to/workspace
    # install
    sudo rosdep install --from-paths src --ignore-src -r -y
    ```

    but livox_ros_driver2 still needs to be installed manually

#### Build

```sh
# entry your workspace/src
cd /path/to/workspace/src

# clone the source codes
git clone git@github.com:Alliance-Algorithm/point_lio.git

# return to the workspace
cd ..

# build
colcon build --merge-install

# if you just want to build this package
colcon build --merge-install --packages-select point_lio

# then what you want is in the workspace/install/ 
```


### Usage
#### Start up

First of all, you should set the correct ip of your wired connection and mid-360 config (192.168.1.120 and **the same nuc ip**)

```sh
# ensure that setup.bash is sourced
# terminal used by this example is zsh
source /opt/ros/humble/setup.zsh
source /path/to/livox_ws/install/setup.zsh

# then start the lidar msg publisher
ros2 launch livox_ros_driver2 msg_MID360_launch.py

# open another terminal
# source the environment of point_lio
source /opt/ros/humble/setup.zsh
source /path/to/point_ws/install/setup.zsh

# start
ros2 launch point_lio mapping_mid360.launch.py

# if you want to check situation in rviz
ros2 launch point_lio mapping_mid360.launch.py rviz:=true
```

#### config

The config file is in `/path/to/point_lio/config`, for dfficiency in runtime, any point cloud msg should not be published

```yaml
publish:
    path_en: false 
    scan_publish_en: false 
    scan_bodyframe_pub_en: false 

pcd_save:
    pcd_save_en: false
    interval: -1 

```

Now you can subscribe the topic "/position" to get position relative to point of starting

### Develop
#### 
