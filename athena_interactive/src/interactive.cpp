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

#include <automation_msgs/srv/navigate_to_pose.hpp>

#include <thread>
#include <string>
#include <memory>
#include <algorithm>
#include <utility>

#include "interactive/interactive.hpp"

using namespace std::chrono_literals;  // ms

namespace interactive
{
Interactive::Interactive()
: athena_utils::LifecycleNode("interactive"),
  explr_map_update_next_state_(false),
  relocalization_state_(false),
  nav_target_reach_(false),
  ultrasonic_is_open_(false),
  rate_control_(10),
  thread_flag_(false) {}

// pub
void Interactive::pubMsg()
{
  rclcpp::WallRate control_rate_(rate_control_);
  while (rclcpp::ok() && thread_flag_) {
    if (mode_.control_mode == SubMode_T::NULL_SUBMODE) {
      control_rate_.sleep();
      continue;
    }

    if (mode_.control_mode == SubMode_T::EXPLR_MAP_UPDATE &&
      !explr_map_update_next_state_ && relocalization_state_)
    {
      if (!modeServiceCall("set_mode_mapping")) {
        RCLCPP_ERROR(
          this->get_logger(),
          "set_mode_mapping service call failed in EXPLR_MAP_UPDATE mode");
      } else {
        explr_map_update_next_state_ = true;
      }
    }

    if (caution_pub_ != nullptr && caution_pub_->get_subscription_count()) {
      automation_msgs::msg::Caution caution;
      if ((mode_.control_mode == SubMode_T::EXPLR_NAV_AB ||
        mode_.control_mode == SubMode_T::EXPLR_MAP_UPDATE) &&
        !relocalization_state_)
      {
        caution.robot_mode = Caution_T::NAV_RELOCATING;
      } else if (mode_.control_mode == SubMode_T::EXPLR_NAV_AB && nav_target_reach_) {
        RCLCPP_INFO(this->get_logger(), "nav_debug: publish caution.");
        caution.robot_mode = Caution_T::NAV_TARGET_REACH;
        nav_target_reach_ = false;
      } else {
        caution.robot_mode = Caution_T::NAV_NORMAL;
      }
      caution.error_type = caution_.error_type;
      caution_pub_->publish(caution);
      caution_.error_type = Caution_T::NO_ERROR;
    }

    if (dog_pose_pub_ != nullptr && dog_pose_pub_->get_subscription_count()) {
      std::string tf_error;
      bool found = tf_buffer_->canTransform(
        "map", "base_footprint",
        tf2::TimePointZero, &tf_error);
      if (!found) {
        RCLCPP_WARN(
          this->get_logger(),
          "can not found the 'map' to 'base_footprint' transform in tf");
        return;
      }
      geometry_msgs::msg::TransformStamped footprint_to_map;
      motion_msgs::msg::SE3Pose dog_pose;
      footprint_to_map = tf_buffer_->lookupTransform(
        "map", "base_footprint",
        tf2::TimePointZero);
      dog_pose.frameid.id = FrameID_T::MAP_FRAME;
      dog_pose.timestamp = this->get_clock()->now();
      dog_pose.position_x = footprint_to_map.transform.translation.x;
      dog_pose.position_y = footprint_to_map.transform.translation.y;
      dog_pose.position_z = footprint_to_map.transform.translation.z;
      dog_pose.rotation_w = footprint_to_map.transform.rotation.w;
      dog_pose.rotation_x = footprint_to_map.transform.rotation.x;
      dog_pose.rotation_y = footprint_to_map.transform.rotation.y;
      dog_pose.rotation_z = footprint_to_map.transform.rotation.z;
      dog_pose_pub_->publish(dog_pose);
    }
    control_rate_.sleep();
  }
}

void Interactive::gridMapCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
  if (mode_.control_mode != SubMode_T::EXPLR_MAP_NEW &&
    mode_.control_mode != SubMode_T::EXPLR_NAV_AB)
  {
    restricted_grid_map_pub_->publish(*map_msg);
    return;
  }

  float resolution = map_msg->info.resolution;
  float x_min, x_max, y_min, y_max;
  x_min = map_msg->info.origin.position.x + resolution;  // has one grid error
  y_min = map_msg->info.origin.position.y + resolution;
  x_max = x_min + map_msg->info.width * resolution - 2 * resolution;
  y_max = y_min + map_msg->info.height * resolution - 2 * resolution;
  for (auto iter : res_areas_) {
    size_t row_trans = iter.first.height;
    size_t col_trans = iter.first.width;
    float x_min_tmp = iter.first.origin.position.x;
    float y_min_tmp = iter.first.origin.position.y;
    for (size_t i = row_trans; i < iter.second.rows + row_trans; i++) {
      for (size_t j = col_trans; j < iter.second.cols + col_trans; j++) {
        if (static_cast<int>(iter.second.at<uchar>(i - row_trans, j - col_trans)) == 0) {
          continue;
        }
        float x_metric = i * resolution + x_min_tmp;
        float y_metric = j * resolution + y_min_tmp;
        if (x_metric < x_min || x_metric > x_max || y_metric < y_min ||
          y_metric > y_max)
        {
          continue;
        }
        int x_grid = (x_metric - map_msg->info.origin.position.x) / resolution;
        int y_grid = (y_metric - map_msg->info.origin.position.y) / resolution;
        int idx = y_grid * map_msg->info.width + x_grid;
        map_msg->data[idx] = 100;
      }
    }
  }
  std::cout << map_msg->info.origin.position.x << std::endl;
  restricted_grid_map_pub_->publish(*map_msg);
}

void Interactive::relocalizationStateCallback(
  const std_msgs::msg::Bool::ConstSharedPtr relocalization_state)
{
  RCLCPP_INFO(this->get_logger(), "relocation success.");
  relocalization_state_ = relocalization_state->data;
}

void Interactive::moveBaseStatusCallback(
  const automation_msgs::msg::NavStatus::ConstSharedPtr move_base_status)
{
  if (move_base_status->status == NavStatus_T::SUCCESS_REACHED) {
    RCLCPP_INFO(this->get_logger(), "nav_debug: nav_target_reach_");
    nav_target_reach_ = true;
  }

  auto now = std::chrono::system_clock::now();
  auto duration =
    std::chrono::duration_cast<std::chrono::seconds>(now - last_move_base_status_time_);
  if (duration.count() < 20) {
    return;
  }
  last_move_base_status_time_ = now;

  switch (move_base_status->status) {
    case NavStatus_T::EMERGENCY_STOP:
      audio_speaker_.sendGoal(122, 4);
      break;
    case NavStatus_T::FAILED_NOPATH:
      audio_speaker_.sendGoal(121, 4);
      break;
    case NavStatus_T::FAILED_TRAPPED:
      audio_speaker_.sendGoal(121, 4);
      break;
    default:
      break;
  }
  RCLCPP_INFO(
    this->get_logger(), "moveBaseStatus: %d",
    move_base_status->status);
}

void Interactive::trackingStatusCallback(
  const automation_msgs::msg::TrackingStatus::ConstSharedPtr
  tracking_status)
{
  // 1/5sec times
  auto now = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_audio_play_time_);
  if ((tracking_status->status == TrackingStatus_T::OBJECT_INVISIBLE ||
    tracking_status->status == TrackingStatus_T::OBJECT_LOST) &&
    duration.count() >= 5.0)
  {
    audio_speaker_.sendGoal(119, 4);
    last_audio_play_time_ = now;
    return;
  }

  // 1 times
  switch (tracking_status->status) {
    case TrackingStatus_T::START_FAIL:
      audio_speaker_.sendGoal(111, 4);
      return;
    case TrackingStatus_T::START_SUCCESS:
      audio_speaker_.sendGoal(112, 4);
      return;
  }

  // 1/20sec times
  duration = std::chrono::duration_cast<std::chrono::seconds>(now - last_tracking_status_time_);
  if (duration.count() < 20) {
    return;
  }
  last_tracking_status_time_ = now;
  switch (tracking_status->status) {
    case TrackingStatus_T::OBJECT_FAR:
      audio_speaker_.sendGoal(116, 4);
      break;
    case TrackingStatus_T::OBJECT_NEAR:
      audio_speaker_.sendGoal(117, 4);
      break;
    default:
      break;
  }
  RCLCPP_INFO(
    this->get_logger(), "trackingStatus: %d",
    tracking_status->status);
}

void Interactive::slamStatusCallback(const std_msgs::msg::UInt8::ConstSharedPtr slam_status)
{
  RCLCPP_INFO(this->get_logger(), "slam status: %d", slam_status->data);
  switch (slam_status->data) {
    case 1:
      caution_.error_type = Caution_T::MAP_LOAD_FAILED;
      break;
    case 2:
      caution_.error_type = Caution_T::REALSENSE_FAILED;
      break;
    default:
      caution_.error_type = Caution_T::NO_ERROR;
      break;
  }
}


void Interactive::getModeCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<automation_msgs::srv::NavMode::Request> req,
  std::shared_ptr<automation_msgs::srv::NavMode::Response> res)
{
  RCLCPP_INFO(this->get_logger(), "checkout submode ...");
  res->success = true;
  relocalization_state_ = false;
  switch (req->sub_mode) {
    case SubMode_T::EXPLR_MAP_UPDATE:  // Update based on old map
      // Switch to the localization mode first, wait for the original map to be
      // relocated and then switch to the mapping mode.
      if (!checkPatternSync() || !modeServiceCall("set_mode_localization")) {
        res->success = false;
      }
      explr_map_update_next_state_ = false;
      break;
    case SubMode_T::EXPLR_MAP_NEW:  // Delete old map, create new map, mapping
                                    // mode.
      if (!checkPatternSync() || !modeServiceCall("set_mode_mapping") ||
        !modeServiceCall("reset"))
      {
        res->success = false;
      }
      break;
    case SubMode_T::EXPLR_NAV_AB:  // Don't update map, localization mode.
    case SubMode_T::TRACK_F:
    case SubMode_T::TRACK_S:
      if (!checkPatternSync() || !modeServiceCall("get_mode", req) ||
        (req->sub_mode == SubMode_T::EXPLR_NAV_AB &&
        !modeServiceCall("process_image_in_localization", true)) ||
        (req->sub_mode != SubMode_T::EXPLR_NAV_AB && modeServiceCall("tracking_mode", req) &&
        !modeServiceCall("process_image_in_localization", false)))
      {
        res->success = false;
      }

      // open ultrasonic
      if (res->success && !ultrasonic_is_open_ && ultrasonicSwitch(true)) {
        RCLCPP_INFO(get_logger(), "Open ultrasonic success.");
      }
      break;
    case SubMode_T::MODE_STOP:
      break;
    default:
      res->success = false;
  }
  if (res->success) {
    RCLCPP_INFO(this->get_logger(), "checkout submode success.");
    mode_.control_mode = req->sub_mode;
  } else {
    RCLCPP_ERROR(this->get_logger(), "checkout submode filed.");
  }
}

bool Interactive::modeServiceCall(std::string service_name, bool process)
{
  auto client = this->create_client<std_srvs::srv::SetBool>(service_name);
  int i = 0;
  while (!client->wait_for_service(std::chrono::seconds(1)) && ++i < 5) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        service_name +
        " client interrupted while waiting for service to appear.");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "waiting for " + service_name + " service to appear...");
  }
  if (i >= 5) {
    RCLCPP_ERROR(this->get_logger(), service_name + " service call failed.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = process;

  bool ok = false;
  using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;
  auto response_received_callback = [&](ServiceResponseFuture future) {
      auto result = future.get();
      if (!result->success) {
        RCLCPP_ERROR(this->get_logger(), service_name + " service call failed");
      } else {
        RCLCPP_INFO(this->get_logger(), service_name + " service call success");
        ok = true;
      }
    };
  auto future_result =
    client->async_send_request(request, response_received_callback);
  if (service_name == "camera/enable") {
    RCLCPP_INFO(
      this->get_logger(), "camera/enable service has been called, %s",
      process ? "open" : "close");
    return true;
  }
  int j = 0;
  while (!ok && j++ < 100) {  // wait for 10s
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (j % 10 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "wait for " + service_name + "'s result: %d s! ", j / 10);
    }
  }

  if (!ok) {
    RCLCPP_ERROR(this->get_logger(), service_name + " service call failed");
    return false;
  }
  return true;
}

bool Interactive::checkPatternSync()
{
  Gait_T goal_gait;
  goal_gait.timestamp = this->get_clock()->now();
  goal_gait.gait = Gait_T::GAIT_SLOW_TROT;  // gait switching
  auto goal = ChangeGait_T::Goal();
  goal.motivation = athena_utils::MODE_TRIG;
  goal.gaitstamped = goal_gait;

  auto pattern_client_ =
    rclcpp_action::create_client<ChangeGait_T>(this, "checkout_gait");
  int i = 0;
  while (!pattern_client_->wait_for_action_server(std::chrono::seconds(2)) &&
    ++i < 5)
  {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "checkout_gait action was interrupted while waiting for the "
        "action. Exiting.\n");
      return 0;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "checkout_gait action not available, waiting again...\n");
  }
  if (i >= 5) {
    RCLCPP_ERROR(this->get_logger(), "checkout_gait failed.");
    return false;
  }

  auto goal_handle = pattern_client_->async_send_goal(goal);
  auto result = pattern_client_->async_get_result(goal_handle.get());
  RCLCPP_INFO(this->get_logger(), "wait_for the result of checkout_gait...\n");
  result.wait_for(std::chrono::seconds(20));
  if (goal_handle.get()->is_result_aware()) {
    if (result.get().result->succeed) {
      RCLCPP_INFO(this->get_logger(), "checkout_gait success.\n");
      return true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "checkout_gait failed.\n");
      return false;
    }
  }
  return true;
}

bool Interactive::modeServiceCall(std::string service_name)
{
  auto client = this->create_client<std_srvs::srv::Empty>(service_name);
  int i = 0;
  while (!client->wait_for_service(std::chrono::seconds(2)) && ++i < 5) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        service_name +
        " client interrupted while waiting for service to appear.");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "waiting for " + service_name + " service to appear...");
  }
  if (i >= 5) {
    RCLCPP_ERROR(this->get_logger(), service_name + " service call failed.");
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Empty::Request>();
  bool ok = false;
  using ServiceResponseFuture =
    rclcpp::Client<std_srvs::srv::Empty>::SharedFuture;
  auto response_received_callback = [&](ServiceResponseFuture) {
      RCLCPP_INFO(this->get_logger(), service_name + "%s call success");
      ok = true;
    };
  auto future_result =
    client->async_send_request(request, response_received_callback);
  if (service_name == "set_mode_mapping") {
    RCLCPP_INFO(
      this->get_logger(), "set_mode_mapping service has been called");
    return true;
  }
  int j = 0;
  while (!ok && j++ < 100) {  // wait for 10s
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (j % 10 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "wait for " + service_name + "'s result: %d s! ", j / 10);
    }
  }

  if (!ok) {
    RCLCPP_ERROR(this->get_logger(), service_name + " service call failed");
    return false;
  }
  return true;
}

bool Interactive::modeServiceCall(
  std::string service_name,
  const std::shared_ptr<automation_msgs::srv::NavMode::Request> request)
{
  auto client =
    this->create_client<automation_msgs::srv::NavMode>(service_name);
  int i = 0;
  while (!client->wait_for_service(std::chrono::seconds(2)) && ++i < 5) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        service_name +
        " client interrupted while waiting for service to appear.");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "waiting for " + service_name + " service to appear...");
  }
  if (i >= 5) {
    RCLCPP_INFO(this->get_logger(), service_name + " service call failed");
    return false;
  }

  bool ok = false;
  using ServiceResponseFuture =
    rclcpp::Client<automation_msgs::srv::NavMode>::SharedFuture;
  auto response_received_callback = [&](ServiceResponseFuture future) {
      if (!future.get()->success) {
        RCLCPP_ERROR(this->get_logger(), service_name + " service call failed");
      } else {
        RCLCPP_INFO(this->get_logger(), service_name + " service call success");
        ok = true;
      }
    };
  auto future_result =
    client->async_send_request(request, response_received_callback);

  int j = 0;
  while (!ok && j++ < 100) {  // wait for 10s
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (j % 10 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "wait for " + service_name + "'s result: %d s! ", j / 10);
    }
  }

  if (!ok) {
    RCLCPP_ERROR(this->get_logger(), service_name + " service call failed");
    return false;
  }
  return true;
}

void Interactive::getTargetCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<automation_msgs::srv::Target::Request> req,
  std::shared_ptr<automation_msgs::srv::Target::Response> res)
{
  // only in explr_nav_ab mode
  res->success = false;
  if (mode_.control_mode != SubMode_T::EXPLR_NAV_AB) {
    RCLCPP_INFO(
      this->get_logger(),
      "current mode is %d, must be in EXPLR_NAV_AB mode.",
      mode_.control_mode);
    return;
  }
  geometry_msgs::msg::PoseStamped target_pose;
  // Convert grid coordinates to physical coordinates
  float x_metric =
    req->target_x * req->info.resolution + req->info.origin.position.x;
  float y_metric =
    req->target_y * req->info.resolution + req->info.origin.position.y;
  target_pose.header = req->header;
  target_pose.pose.position.x = x_metric;
  target_pose.pose.position.y = y_metric;
  target_pub_->publish(target_pose);

  RCLCPP_INFO(
    this->get_logger(), "resolution: %f   ori_x: %f  ori_y: %f",
    req->info.resolution, req->info.origin.position.x,
    req->info.origin.position.y);
  RCLCPP_INFO(
    this->get_logger(), "pix row : %d   col: %d", req->target_x,
    req->target_y);
  RCLCPP_INFO(this->get_logger(), "metric x: %f     y: %f", x_metric, y_metric);

  // auto node = rclcpp::Node::make_shared("NaviTos_client");
  auto client =
    this->create_client<automation_msgs::srv::NavigateToPose>("NaviTo");
  int i = 0;
  while (!client->wait_for_service(std::chrono::seconds(1)) && ++i < 5) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "client interrupted while waiting for service to appear.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "waiting for NaviTos service to appear...");
  }
  if (i >= 5) {
    RCLCPP_INFO(this->get_logger(), "NaviTos service call failed");
    return;
  }

  auto request =
    std::make_shared<automation_msgs::srv::NavigateToPose::Request>();
  request->controller_id = "FollowPath";
  request->planner_id = "GridBased";
  request->goal.header = req->header;
  request->goal.header.frame_id = "map";
  request->goal.pose.position.x = x_metric;
  request->goal.pose.position.y = y_metric;

  using ServiceResponseFuture =
    rclcpp::Client<automation_msgs::srv::NavigateToPose>::SharedFuture;
  auto response_received_callback = [&](ServiceResponseFuture future) {
      if (!future.get()->result) {
        RCLCPP_ERROR(this->get_logger(), "NaviTos service call failed");
      } else {
        RCLCPP_INFO(this->get_logger(), "NaviTos service call success");
        res->success = true;
      }
    };
  auto future_result =
    client->async_send_request(request, response_received_callback);

  int j = 0;
  while (!res->success && j++ < 100) {  // wait for 10s
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (j % 10 == 0) {
      RCLCPP_INFO(
        this->get_logger(), "wait for NaviTos's result: %d s",
        j / 10);
    }
  }

  if (!res->success) {
    RCLCPP_ERROR(this->get_logger(), "NaviTos service call failed");
  }
}

void Interactive::getRestrictedAreaCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<automation_msgs::srv::RestrictedArea::Request> req,
  std::shared_ptr<automation_msgs::srv::RestrictedArea::Response> res)
{
  // only in explr_nav_ab mode
  if (mode_.control_mode != SubMode_T::EXPLR_NAV_AB) {
    RCLCPP_INFO(
      this->get_logger(),
      "current mode is %d, must be in EXPLR_NAV_AB mode.",
      mode_.control_mode);
    res->success = false;
    return;
  }
  if (req->length % 2 != 0) {
    RCLCPP_INFO(this->get_logger(), "req->length % 2 != 0.");
    res->success = false;
    return;
  }

  // req->data[] (row, col)
  int width = 0, height = 0, row_trans = INT_MAX, col_trans = INT_MAX;
  for (size_t i = 0; i < req->length; i += 2) {
    height = std::max(req->data[i], height);
    width = std::max(req->data[i + 1], width);
    row_trans = std::min(req->data[i], row_trans);
    col_trans = std::min(req->data[i + 1], col_trans);
  }
  height++;
  width++;
  nav_msgs::msg::MapMetaData map_info;
  map_info = req->info;
  map_info.width = col_trans;
  map_info.height = row_trans;

  cv::Mat res_area = cv::Mat::zeros(height, width, CV_8U);
  size_t len = req->length / 2;
  cv::Point * pts = new cv::Point[len];
  for (size_t i = 0; i < len * 2; i += 2) {
    pts[i / 2] =
      cv::Point(req->data[i + 1] - col_trans, req->data[i] - row_trans);
  }

  cv::fillConvexPoly(res_area, pts, len, cv::Scalar(100));
  res_areas_.push_back(std::make_pair(map_info, res_area));
  res->success = true;
}

CallbackReturn_T Interactive::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Interactive Configuring");
  audio_speaker_.m_player_client =
    rclcpp_action::create_client<AudioPlay>(this, "audio_play");
  dog_pose_pub_ = this->create_publisher<motion_msgs::msg::SE3Pose>(
    "dog_pose", rclcpp::SystemDefaultsQoS());
  target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", rclcpp::SystemDefaultsQoS());
  restricted_grid_map_pub_ =
    this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map_ch", rclcpp::SystemDefaultsQoS());
  caution_pub_ = this->create_publisher<automation_msgs::msg::Caution>(
    "nav_status", rclcpp::SystemDefaultsQoS());
  // timer_ = this->create_wall_timer(50ms, std::bind(&Interactive::pubMsg, this));

  grid_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map_ori", rclcpp::SystemDefaultsQoS(),
    std::bind(&Interactive::gridMapCallback, this, std::placeholders::_1));
  relocalization_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "relocalizationState", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &Interactive::relocalizationStateCallback, this,
      std::placeholders::_1));
  move_base_status_sub_ =
    this->create_subscription<automation_msgs::msg::NavStatus>(
    "move_base_status", rclcpp::SystemDefaultsQoS(),
    std::bind(
      &Interactive::moveBaseStatusCallback, this,
      std::placeholders::_1));
  tracking_status_sub_ =
    this->create_subscription<automation_msgs::msg::TrackingStatus>(
    "tracking_status", rclcpp::SensorDataQoS(),
    std::bind(
      &Interactive::trackingStatusCallback, this,
      std::placeholders::_1));
  slam_status_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "slam_error_status", rclcpp::SensorDataQoS(), std::bind(
      &Interactive::slamStatusCallback,
      this, std::placeholders::_1));

  callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  get_mode_ = this->create_service<automation_msgs::srv::NavMode>(
    "nav_mode",
    std::bind(
      &Interactive::getModeCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_services_default, callback_group_);
  get_target_ = this->create_service<automation_msgs::srv::Target>(
    "nav_target",
    std::bind(
      &Interactive::getTargetCallback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3),
    rmw_qos_profile_services_default, callback_group_);
  get_restricted_area_ =
    this->create_service<automation_msgs::srv::RestrictedArea>(
    "nav_restricted_area",
    std::bind(
      &Interactive::getRestrictedAreaCallback, this,
      std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3),
    rmw_qos_profile_services_default, callback_group_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  caution_.robot_mode = Caution_T::NAV_NORMAL;
  caution_.error_type = Caution_T::NO_ERROR;

  RCLCPP_INFO(get_logger(), "%s Configured.", get_name());
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T Interactive::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Interactive Activating");
  try {
    modeServiceCall("camera/enable", true);
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "realsense enable error...\n");
  }
  last_audio_play_time_ = std::chrono::system_clock::now();
  last_tracking_status_time_ = std::chrono::system_clock::now();
  last_move_base_status_time_ = std::chrono::system_clock::now();
  publisher_thread_ = std::make_shared<std::thread>(
    &Interactive::pubMsg, this);
  thread_flag_ = true;
  dog_pose_pub_->on_activate();
  target_pub_->on_activate();
  restricted_grid_map_pub_->on_activate();
  caution_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "%s Activated.", get_name());
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T Interactive::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Interactive Deactivating");
  try {
    modeServiceCall("camera/enable", false);
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "realsense enable error...\n");
  }
  thread_flag_ = false;
  publisher_thread_->join();

  mode_.control_mode = SubMode_T::NULL_SUBMODE;
  relocalization_state_ = false;
  dog_pose_pub_->on_deactivate();
  target_pub_->on_deactivate();
  restricted_grid_map_pub_->on_deactivate();
  caution_pub_->on_deactivate();
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T Interactive::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Interactive Cleaning up");
  publisher_thread_.reset();
  mode_.control_mode = SubMode_T::NULL_SUBMODE;
  relocalization_state_ = false;
  dog_pose_pub_.reset();
  target_pub_.reset();
  restricted_grid_map_pub_.reset();
  caution_pub_.reset();
  RCLCPP_INFO(get_logger(), "%s Completed Cleaning up", get_name());
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T Interactive::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Interactive Shutting down");
  return CallbackReturn_T::SUCCESS;
}

bool Interactive::ultrasonicSwitch(bool open)
{
  std::string service_name = "ultrasonic_server";

  auto client = this->create_client<ception_msgs::srv::SensorDetectionNode>(
    "obstacle_detection");

  int i = 0;
  while (!client->wait_for_service(std::chrono::seconds(1)) && ++i < 5) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "client interrupted while waiting for service to appear.");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(), "waiting for %s %d seconds",
      service_name.c_str(), i);
  }

  if (i >= 5) {
    RCLCPP_ERROR(this->get_logger(), "Open ultrasonic failed.");
    return false;
  }

  auto request =
    std::make_shared<ception_msgs::srv::SensorDetectionNode::Request>();
  if (open) {
    request->command =
      ception_msgs::srv::SensorDetectionNode::Request::ENABLE_ALL;
  } else {
    request->command =
      ception_msgs::srv::SensorDetectionNode::Request::DISABLE_ALL;
  }

  bool ok = false;
  using ServiceResponseFuture =
    rclcpp::Client<ception_msgs::srv::SensorDetectionNode>::SharedFuture;
  auto response_received_callback = [&](ServiceResponseFuture future) {
      auto result = future.get();
      if (!result->success) {
        RCLCPP_ERROR(
          this->get_logger(),
          " %s call failed, success %d, clientcount %d",
          service_name.c_str(), static_cast<int>(result->success),
          static_cast<int>(result->clientcount));
      } else {
        RCLCPP_INFO(
          this->get_logger(),
          "%s call success, success %d, clientcount %d",
          service_name.c_str(), static_cast<int>(result->success),
          static_cast<int>(result->clientcount));
        ok = true;
      }
    };
  auto future_result =
    client->async_send_request(request, response_received_callback);

  int j = 0;
  while (!ok && j++ < 100) {  // wait for 10s
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (j % 10 == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "wait for " + service_name + "'s result: %d s", j / 10);
    }
  }

  if (!ok) {
    RCLCPP_ERROR(this->get_logger(), "ultrasonic_server call failed");
    return false;
  }
  return true;
}

}  // namespace interactive

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::executor::ExecutorArgs(), 2);
  auto node = std::make_shared<interactive::Interactive>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
