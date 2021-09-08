rtabmap_ros [![Build Status](https://travis-ci.org/introlab/rtabmap_ros.svg?branch=ros2)](https://travis-ci.org/introlab/rtabmap_ros)
===========

RTAB-Map's ROS2 package (branch `ros2`). **UNDER CONSTRUCTION**: currently most nodes are ported to ROS2, however they are not all tested yet. The interface is the same than on ROS1 (parameters and topic names should still match ROS1 documentation on [rtabmap_ros](http://wiki.ros.org/rtabmap_ros)). See `launch/ros2` subfolder for some ROS2 examples with turtlebot3 in simulation (tested under ROS2 Eloquent distro).

## 编译安装
1. On dog

* RTAB-Map库:
   ```bash
    $ cd ~
    $ git clone https://github.com/introlab/rtabmap.git rtabmap
    $ cd rtabmap/build
    $ cmake ..
    $ make -j4
    $ sudo make install
    ```
* RTAB-Map ROS2包:
    ```bash
    $ cd ~/ros2_ws
    $ git clone git@partner-gitlab.mioffice.cn:slam/rtabmap_ros.git src/rtabmap_ros
    $ MAKEFLAGS="-j4" colcon build --merge-install --install-base=/opt/ros2/cyberdog
    ```

* [ROS2官方教程](
https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
   ```bash
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
   ```

* 依赖项
   ```bash
    $ sudo apt install libeigen3-dev libsuitesparse-dev libpcl-dev liboctomap-dev -y
    $ sudo apt install -y ros-foxy-vision-opencv \
                          ros-foxy-image-common \
                          ros-foxy-diagnostic-* \
                          ros-foxy-perception-pcl \
                          ros-foxy-octomap
    $ ROS2 pkg: athena_cyberdog/athena_common/athena_util

    $ mkdir ~/software && cd $_
    $ git clone -b 20201223_git git@github.com:RainerKuemmerle/g2o.git
    $ cd g2o
    $ cmake -H. -Bbuild
    $ sudo make -Cbuild install -j4

    $ cd ~/software
    $ git clone -b 4.2.0 https://github.com/opencv/opencv
    $ git clone -b 4.2.0 https://github.com/opencv/opencv_contrib
    $ cd opencv
    $ cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release \
                        -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules
    $ sudo make -Cbuild install -j4
   ```

* RTAB-Map库:
   ```bash
    $ cd ~
    $ git clone git@partner-gitlab.mioffice.cn:slam/rtabmap.git rtabmap
    $ cd rtabmap
    $ cmake -H. -Bbuild -DCMAKE_BUILD_TYPE=Release \
                        -DCMAKE_INSTALL_PREFIX=/opt/ros/foxy \
                        -DWITH_FREENECT=OFF -DWITH_FREENECT2=OFF \
                        -DWITH_K4W2=OFF -DWITH_K4A=OFF -DWITH_OPENNI2=OFF \
                        -DWITH_DC1394=OFF -DWITH_FLYCAPTURE2=OFF \
                        -DWITH_ZED=OFF -DWITH_REALSENSE=OFF \
                        -DWITH_REALSENSE_SLAM=OFF -DWITH_REALSENSE2=OFF \
                        -DWITH_MYNTEYE=OFF -DBUILD_APP=OFF -DBUILD_TOOLS=OFF \
                        -DBUILD_EXAMPLES=OFF -DWITH_QT=OFF -DWITH_CERES=OFF
    $ sudo make -Cbuild install -j4
    ```
* RTAB-Map ROS2包:
    ```bash
    $ cd ~/ros2_ws
    $ git clone git@partner-gitlab.mioffice.cn:slam/rtabmap_ros.git src/rtabmap_ros
    $ MAKEFLAGS="-j4" colcon build --merge-install
    ```
## 使用方法
与[原版](https://github.com/introlab/rtabmap_ros/tree/ros2)相比，增加了[lifecycle](http://design.ros2.org/articles/node_lifecycle.html)生命周期管理.
```bash
$ ros2 launch rtabmap_ros rs_d455.launch.py
$ ros2 service call /rtabmap/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}" #on_configure
$ ros2 service call /rtabmap/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}" #on_activate
$ ros2 service call /rtabmap/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 4}}" #on_deactivate
```

## 节点信息

### 节点名称

* /rtabmap

### 订阅的Topics

* /camera/aligned_depth_to_color/image_raw: sensor_msgs/msg/Image
* /camera/color/camera_info: sensor_msgs/msg/CameraInfo
* /camera/color/image_raw: sensor_msgs/msg/Image
* /cascade_lifecycle_activations: cascade_lifecycle_msgs/msg/Activation
* /cascade_lifecycle_states: cascade_lifecycle_msgs/msg/State
* /global_pose: geometry_msgs/msg/PoseWithCovarianceStamped
* /goal: geometry_msgs/msg/PoseStamped
* /goal_node: rtabmap_ros/msg/Goal
* /gps/fix: sensor_msgs/msg/NavSatFix
* /imu/data: sensor_msgs/msg/Imu
* /initialpose: geometry_msgs/msg/PoseWithCovarianceStamped
* /parameter_events: rcl_interfaces/msg/ParameterEvent
* /tracking_pose: geometry_msgs/msg/PoseStamped
* /user_data_async: rtabmap_ros/msg/UserData

### 发布的Topics

* /cascade_lifecycle_activations: cascade_lifecycle_msgs/msg/Activation
* /cascade_lifecycle_states: cascade_lifecycle_msgs/msg/State
* /cloud_ground: sensor_msgs/msg/PointCloud2
* /cloud_map: sensor_msgs/msg/PointCloud2
* /cloud_obstacles: sensor_msgs/msg/PointCloud2
* /global_path: nav_msgs/msg/Path
* /global_path_nodes: rtabmap_ros/msg/Path
* /goal_out: geometry_msgs/msg/PoseStamped
* /goal_reached: std_msgs/msg/Bool
* /grid_prob_map: nav_msgs/msg/OccupancyGrid
* /info: rtabmap_ros/msg/Info
* /labels: visualization_msgs/msg/MarkerArray
* /landmarks: geometry_msgs/msg/PoseArray
* /local_grid_empty: sensor_msgs/msg/PointCloud2
* /local_grid_ground: sensor_msgs/msg/PointCloud2
* /local_grid_obstacle: sensor_msgs/msg/PointCloud2
* /local_path: nav_msgs/msg/Path
* /local_path_nodes: rtabmap_ros/msg/Path
* /localization_pose: geometry_msgs/msg/PoseWithCovarianceStamped
* /map: nav_msgs/msg/OccupancyGrid
* /mapData: rtabmap_ros/msg/MapData
* /mapGraph: rtabmap_ros/msg/MapGraph
* /mapPath: nav_msgs/msg/Path
* /octomap_empty_space: sensor_msgs/msg/PointCloud2
* /octomap_global_frontier_space: sensor_msgs/msg/PointCloud2
* /octomap_grid: nav_msgs/msg/OccupancyGrid
* /octomap_ground: sensor_msgs/msg/PointCloud2
* /octomap_obstacles: sensor_msgs/msg/PointCloud2
* /octomap_occupied_space: sensor_msgs/msg/PointCloud2
* /parameter_events: rcl_interfaces/msg/ParameterEvent
* /relocalizationState: std_msgs/msg/Bool
* /rosout: rcl_interfaces/msg/Log
* /rtabmap/transition_event: lifecycle_msgs/msg/TransitionEvent
* /slam_error_status: std_msgs/msg/UInt8

### Service Servers

* /backup: std_srvs/srv/Empty
* /cancel_goal: std_srvs/srv/Empty
* /get_map: nav_msgs/srv/GetMap
* /get_map_data: rtabmap_ros/srv/GetMap
* /get_plan: nav_msgs/srv/GetPlan
* /get_plan_nodes: rtabmap_ros/srv/GetPlan
* /get_prob_map: nav_msgs/srv/GetMap
* /list_labels: rtabmap_ros/srv/ListLabels
* /log_debug: std_srvs/srv/Empty
* /log_error: std_srvs/srv/Empty
* /log_info: std_srvs/srv/Empty
* /log_warning: std_srvs/srv/Empty
* /pause: std_srvs/srv/Empty
* /processImage: std_srvs/srv/SetBool
* /publish_map: rtabmap_ros/srv/PublishMap
* /reset: std_srvs/srv/Empty
* /resume: std_srvs/srv/Empty
* /rtabmap/change_state: lifecycle_msgs/srv/ChangeState
* /rtabmap/describe_parameters: rcl_interfaces/srv/DescribeParameters
* /rtabmap/get_available_states: lifecycle_msgs/srv/GetAvailableStates
* /rtabmap/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
* /rtabmap/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
* /rtabmap/get_parameters: rcl_interfaces/srv/GetParameters
* /rtabmap/get_state: lifecycle_msgs/srv/GetState
* /rtabmap/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
* /rtabmap/list_parameters: rcl_interfaces/srv/ListParameters
* /rtabmap/set_parameters: rcl_interfaces/srv/SetParameters
* /rtabmap/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
* /set_goal: rtabmap_ros/srv/SetGoal
* /set_label: rtabmap_ros/srv/SetLabel
* /set_mode_localization: std_srvs/srv/Empty
* /set_mode_mapping: std_srvs/srv/Empty
* /trigger_new_map: std_srvs/srv/Empty

### Service Clients

* /rtabmap/describe_parameters: rcl_interfaces/srv/DescribeParameters
* /rtabmap/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
* /rtabmap/get_parameters: rcl_interfaces/srv/GetParameters
* /rtabmap/list_parameters: rcl_interfaces/srv/ListParameters
* /rtabmap/set_parameters: rcl_interfaces/srv/SetParameters
* /rtabmap/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
