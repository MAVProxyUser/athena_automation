# interactive

## 简介

Interactive是一个基于ROS2的包，该包主要包含如下功能：

* 子模式切换: 新建地图、更新地图、AB点导航、自动跟随、人脸跟随；
* 跟随与导航模块的语音管理；
* 消息整合与发布。


## 依赖项

编译该包需要具备ROS2环境及基础功能包，除此之外，还需要如下依赖：

* athena_utils
* interaction_msgs
* automation_msgs
* motion_msgs
* cascade_lifecycle_msgs
* ception_msgs


## 编译安装

ubuntu

```
$ mkdir -p ~/ros2_ws/src
$ cd ros2_ws/src
$ git clone git@partner-gitlab.mioffice.cn:interactive/interactive.git
$ cd ..
$ colcon build --packages-select interacitive --install-base=/opt/ros2/cyberdog/ --merge-install
```


## 使用方法

Start and activate interactive node

```
$ ros2 run interactive interactive_node
$ ros2 service call /interactive/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"
$ ros2 service call /interactive/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"
```


## 节点信息

### 节点名称

* /interactive

### Subscribed Topics

* /relocalizationState: std_msgs/msg/Bool
* /move_base_status: automation_msgs/msg/NavStatus
* /tracking_status: automation_msgs/msg/TrackingStatus

### Published Topics

* /dog_pose: motion_msgs/msg/SE3Pose
* /initialpose: geometry_msgs/msg/PoseWithCovarianceStamped
* /nav_status: automation_msgs/msg/Caution

### Service Servers

* /nav_mode: automation_msgs/srv/NavMode

### Service Clients

* /set_mode_localization: std_srvs/srv/Empty
* /set_mode_mapping: std_srvs/srv/Empty
* /reset: std_srvs/srv/Empty
* /processImage: std_srvs/srv/SetBool
* /camera/enable: std_srvs/srv/SetBool
* /get_mode: automation_msgs/srv/NavMode
* /tracking_mode: automation_msgs/srv/NavMode
* /ultrasonic_server: ception_msgs/srv/SensorDetectionNode