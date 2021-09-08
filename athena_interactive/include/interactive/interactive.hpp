// Copyright 2021 Xiaomi Inc
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTERACTIVE__INTERACTIVE_HPP_
#define INTERACTIVE__INTERACTIVE_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <athena_utils/lifecycle_node.hpp>
#include <automation_msgs/msg/caution.hpp>
#include <automation_msgs/msg/nav_status.hpp>
#include <automation_msgs/msg/tracking_status.hpp>
#include <automation_msgs/srv/nav_mode.hpp>
#include <automation_msgs/srv/restricted_area.hpp>
#include <automation_msgs/srv/target.hpp>
#include <ception_msgs/srv/sensor_detection_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <motion_msgs/action/change_gait.hpp>
#include <motion_msgs/msg/gait.hpp>
#include <motion_msgs/msg/mode.hpp>
#include <motion_msgs/msg/safety.hpp>
#include <motion_msgs/msg/se3_pose.hpp>
#include <motion_msgs/msg/se3_velocity_cmd.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <chrono>
#include <thread>

#include "interactive/speaker_client.hpp"

namespace interactive
{
using CallbackReturn_T = athena_utils::CallbackReturn;
using Mode_T = motion_msgs::msg::Mode;
using Gait_T = motion_msgs::msg::Gait;
using FrameID_T = motion_msgs::msg::Frameid;
using Caution_T = automation_msgs::msg::Caution;
using Safety_T = motion_msgs::msg::Safety;
using SubMode_T = automation_msgs::srv::NavMode_Request;
using ChangeGait_T = motion_msgs::action::ChangeGait;
using NavStatus_T = automation_msgs::msg::NavStatus;
using TrackingStatus_T = automation_msgs::msg::TrackingStatus;

class Interactive : public athena_utils::LifecycleNode
{
public:
  Interactive();
  ~Interactive()
  {
  }

  void pubMsg();

  void gridMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
  void relocalizationStateCallback(
    const std_msgs::msg::Bool::ConstSharedPtr relocalization_state);
  void moveBaseStatusCallback(
    const automation_msgs::msg::NavStatus::ConstSharedPtr move_base_status);
  void trackingStatusCallback(
    const automation_msgs::msg::TrackingStatus::ConstSharedPtr
    tracking_status);
  void slamStatusCallback(const std_msgs::msg::UInt8::ConstSharedPtr slam_status);

  void getModeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<automation_msgs::srv::NavMode::Request> req,
    std::shared_ptr<automation_msgs::srv::NavMode::Response> res);
  void getTargetCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<automation_msgs::srv::Target::Request> req,
    std::shared_ptr<automation_msgs::srv::Target::Response> res);
  void getRestrictedAreaCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<automation_msgs::srv::RestrictedArea::Request> req,
    std::shared_ptr<automation_msgs::srv::RestrictedArea::Response> res);

protected:
  CallbackReturn_T on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn_T on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  bool checkPatternSync();
  bool modeServiceCall(std::string service_name);
  bool modeServiceCall(std::string service_name, bool process);
  bool modeServiceCall(
    std::string service_name,
    const std::shared_ptr<automation_msgs::srv::NavMode::Request> request);

  rclcpp_lifecycle::LifecyclePublisher<motion_msgs::msg::SE3Pose>::SharedPtr
    dog_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
    restricted_grid_map_pub_;
  rclcpp_lifecycle::LifecyclePublisher<automation_msgs::msg::Caution>::SharedPtr
    caution_pub_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::ConstSharedPtr
    grid_map_sub_;

  rclcpp::Subscription<std_msgs::msg::Bool>::ConstSharedPtr
    relocalization_state_sub_;
  rclcpp::Subscription<automation_msgs::msg::NavStatus>::ConstSharedPtr
    move_base_status_sub_;
  rclcpp::Subscription<automation_msgs::msg::TrackingStatus>::ConstSharedPtr
    tracking_status_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::ConstSharedPtr slam_status_sub_;

  rclcpp::Service<automation_msgs::srv::NavMode>::SharedPtr get_mode_;
  rclcpp::Service<automation_msgs::srv::Target>::SharedPtr get_target_;
  rclcpp::Service<automation_msgs::srv::RestrictedArea>::SharedPtr
    get_restricted_area_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist twist_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<std::pair<nav_msgs::msg::MapMetaData, cv::Mat>> res_areas_;

  Mode_T mode_;
  SpeakerClient audio_speaker_;
  Caution_T caution_;

  // There are two steps. From the first step to the second step, you need to
  // wait for a successful relocation.
  bool explr_map_update_next_state_;
  bool relocalization_state_;
  bool nav_target_reach_;

  // audio
  std::chrono::system_clock::time_point last_audio_play_time_;
  std::chrono::system_clock::time_point last_tracking_status_time_;
  std::chrono::system_clock::time_point last_move_base_status_time_;

  // ultrasonic
  bool ultrasonic_is_open_;
  bool ultrasonicSwitch(bool open);

  std::shared_ptr<std::thread> publisher_thread_;
  int rate_control_;
  bool thread_flag_;
};
}  // namespace interactive
#endif  // INTERACTIVE__INTERACTIVE_HPP_
