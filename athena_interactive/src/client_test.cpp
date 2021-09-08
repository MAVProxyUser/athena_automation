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

#include <athena_utils/lifecycle_node.hpp>
#include <automation_msgs/srv/nav_mode.hpp>
#include <automation_msgs/srv/restricted_area.hpp>
#include <automation_msgs/srv/target.hpp>
#include <motion_msgs/action/change_gait.hpp>
#include <motion_msgs/action/change_mode.hpp>
#include <motion_msgs/msg/gait.hpp>
#include <motion_msgs/msg/mode.hpp>
#include <motion_msgs/msg/se3_velocity_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using ChangeMode_T = motion_msgs::action::ChangeMode;
using SubMode_T = automation_msgs::srv::NavMode_Request;
using Gait_T = motion_msgs::msg::Gait;
using ChangeGait_T = motion_msgs::action::ChangeGait;

class DogMotionNode : public rclcpp::Node
{
public:
  DogMotionNode()
  : Node("DogMotion")
  {
    auto callback = [this](std_msgs::msg::Int32::SharedPtr msg) {
        Command = msg->data;
        dog_thread = std::make_shared<std::thread>(
          &DogMotionNode::dog_command_callback, this);
        dog_thread->detach();
        // this->dog_command_callback(msg->data);
      };
    dog_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "dog_command", rclcpp::SystemDefaultsQoS(), callback);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr dog_sub_;
  std::shared_ptr<std::thread> dog_thread;
  int Command;

private:
  void dog_command_callback()
  {
    printf("Dog command get %d.\n", Command);
    if (Command == 100) {
      // checkout mode
      auto mode_goal = ChangeMode_T::Goal();
      motion_msgs::msg::Mode mode;
      // mode.control_mode = motion_msgs::msg::Mode::MODE_TRACK;
      mode.control_mode = motion_msgs::msg::Mode::MODE_MANUAL;
      // mode.mode_type = motion_msgs::msg::Mode::TRACK_F;
      mode_goal.modestamped = mode;

      auto mode_client_ =
        rclcpp_action::create_client<ChangeMode_T>(this, "checkout_mode");
      auto mode_goal_handle = mode_client_->async_send_goal(mode_goal);
      auto mode_result = mode_client_->async_get_result(mode_goal_handle.get());
      mode_result.wait_for(std::chrono::seconds(10));
      if (mode_goal_handle.get()->is_result_aware()) {
        if (mode_result.get().result->succeed) {
          printf("Result of mode_checkout: success\n");
        } else {
          printf("Result of mode_checkout: failed\n");
        }
      }

      // checkout pattern
      Gait_T goal_gait;
      goal_gait.timestamp = this->get_clock()->now();
      goal_gait.gait = Gait_T::GAIT_TROT;  // gait switching
      auto goal = ChangeGait_T::Goal();
      goal.motivation = athena_utils::MODE_TRIG;
      goal.gaitstamped = goal_gait;

      auto pattern_client_ =
        rclcpp_action::create_client<ChangeGait_T>(this, "checkout_gait");
      auto pattern_goal_handle = pattern_client_->async_send_goal(goal);
      auto pattern_result =
        pattern_client_->async_get_result(pattern_goal_handle.get());
      pattern_result.wait_for(std::chrono::seconds(20));
      if (pattern_goal_handle.get()->is_result_aware()) {
        if (pattern_result.get().result->succeed) {
          printf("Result of checkout_gait: success\n");
        } else {
          printf("Result of checkout_gait: failed\n");
        }
      }
      printf("motion end\n");
    } else if (Command == 1) {
      // checkout mode
      auto mode_goal = ChangeMode_T::Goal();
      motion_msgs::msg::Mode mode;
      // mode.control_mode = motion_msgs::msg::Mode::MODE_TRACK;
      mode.control_mode = motion_msgs::msg::Mode::MODE_EXPLOR;
      mode.mode_type = motion_msgs::msg::Mode::EXPLR_MAP_N;
      mode_goal.modestamped = mode;

      auto mode_client_ =
        rclcpp_action::create_client<ChangeMode_T>(this, "checkout_mode");
      auto mode_goal_handle = mode_client_->async_send_goal(mode_goal);
      auto mode_result = mode_client_->async_get_result(mode_goal_handle.get());
      mode_result.wait_for(std::chrono::seconds(10));
      if (mode_goal_handle.get()->is_result_aware()) {
        if (mode_result.get().result->succeed) {
          printf("Result of mode_checkout: success\n");
        } else {
          printf("Result of mode_checkout: failed\n");
        }
      }

      // checkout pattern
      Gait_T goal_gait;
      goal_gait.timestamp = this->get_clock()->now();
      goal_gait.gait = Gait_T::GAIT_TROT;  // gait switching
      auto goal = ChangeGait_T::Goal();
      goal.motivation = athena_utils::MODE_TRIG;
      goal.gaitstamped = goal_gait;

      auto pattern_client_ =
        rclcpp_action::create_client<ChangeGait_T>(this, "checkout_gait");
      auto pattern_goal_handle = pattern_client_->async_send_goal(goal);
      auto pattern_result =
        pattern_client_->async_get_result(pattern_goal_handle.get());
      pattern_result.wait_for(std::chrono::seconds(20));
      if (pattern_goal_handle.get()->is_result_aware()) {
        if (pattern_result.get().result->succeed) {
          printf("Result of checkout_gait: success\n");
        } else {
          printf("Result of checkout_gait: failed\n");
        }
      }
      printf("motion end\n");
    } else if (Command == 2) {
      // checkout mode
      auto mode_goal = ChangeMode_T::Goal();
      motion_msgs::msg::Mode mode;
      mode.control_mode = motion_msgs::msg::Mode::MODE_EXPLOR;
      mode.mode_type = motion_msgs::msg::Mode::EXPLR_NAV_AB;
      mode_goal.modestamped = mode;

      auto mode_client_ =
        rclcpp_action::create_client<ChangeMode_T>(this, "checkout_mode");
      auto mode_goal_handle = mode_client_->async_send_goal(mode_goal);
      auto mode_result = mode_client_->async_get_result(mode_goal_handle.get());
      mode_result.wait_for(std::chrono::seconds(10));
      if (mode_goal_handle.get()->is_result_aware()) {
        if (mode_result.get().result->succeed) {
          printf("Result of mode_checkout: success\n");
        } else {
          printf("Result of mode_checkout: failed\n");
        }
      }

      // checkout pattern
      Gait_T goal_gait;
      goal_gait.timestamp = this->get_clock()->now();
      goal_gait.gait = Gait_T::GAIT_TROT;  // gait switching
      auto goal = ChangeGait_T::Goal();
      goal.motivation = athena_utils::MODE_TRIG;
      goal.gaitstamped = goal_gait;

      auto pattern_client_ =
        rclcpp_action::create_client<ChangeGait_T>(this, "checkout_gait");
      auto pattern_goal_handle = pattern_client_->async_send_goal(goal);
      auto pattern_result =
        pattern_client_->async_get_result(pattern_goal_handle.get());
      pattern_result.wait_for(std::chrono::seconds(20));
      if (pattern_goal_handle.get()->is_result_aware()) {
        if (pattern_result.get().result->succeed) {
          printf("Result of checkout_gait: success\n");
        } else {
          printf("Result of checkout_gait: failed\n");
        }
      }
      printf("motion end\n");
    } else if (Command == 3) {
      // checkout mode
      auto mode_goal = ChangeMode_T::Goal();
      motion_msgs::msg::Mode mode;
      mode.control_mode = motion_msgs::msg::Mode::MODE_TRACK;
      mode.mode_type = motion_msgs::msg::Mode::TRACK_F;
      mode_goal.modestamped = mode;

      auto mode_client_ =
        rclcpp_action::create_client<ChangeMode_T>(this, "checkout_mode");
      auto mode_goal_handle = mode_client_->async_send_goal(mode_goal);
      auto mode_result = mode_client_->async_get_result(mode_goal_handle.get());
      mode_result.wait_for(std::chrono::seconds(10));
      if (mode_goal_handle.get()->is_result_aware()) {
        if (mode_result.get().result->succeed) {
          printf("Result of mode_checkout: success\n");
        } else {
          printf("Result of mode_checkout: failed\n");
        }
      }

      // checkout pattern
      Gait_T goal_gait;
      goal_gait.timestamp = this->get_clock()->now();
      goal_gait.gait = Gait_T::GAIT_TROT;  // gait switching
      auto goal = ChangeGait_T::Goal();
      goal.motivation = athena_utils::MODE_TRIG;
      goal.gaitstamped = goal_gait;

      auto pattern_client_ =
        rclcpp_action::create_client<ChangeGait_T>(this, "checkout_gait");
      auto pattern_goal_handle = pattern_client_->async_send_goal(goal);
      auto pattern_result =
        pattern_client_->async_get_result(pattern_goal_handle.get());
      pattern_result.wait_for(std::chrono::seconds(20));
      if (pattern_goal_handle.get()->is_result_aware()) {
        if (pattern_result.get().result->succeed) {
          printf("Result of checkout_gait: success\n");
        } else {
          printf("Result of checkout_gait: failed\n");
        }
      }
      printf("motion end\n");
    } else if (Command == 0) {
      // checkout mode
      auto mode_goal = ChangeMode_T::Goal();
      motion_msgs::msg::Mode mode;
      // mode.control_mode = motion_msgs::msg::Mode::MODE_TRACK;
      mode.control_mode = motion_msgs::msg::Mode::MODE_DEFAULT;
      // mode.mode_type = motion_msgs::msg::Mode::TRACK_F;

      mode_goal.modestamped = mode;

      auto mode_client_ =
        rclcpp_action::create_client<ChangeMode_T>(this, "checkout_mode");
      auto mode_goal_handle = mode_client_->async_send_goal(mode_goal);
      auto mode_result = mode_client_->async_get_result(mode_goal_handle.get());
      mode_result.wait_for(std::chrono::seconds(10));
      if (mode_goal_handle.get()->is_result_aware()) {
        if (mode_result.get().result->succeed) {
          printf("Result of mode_checkout: success\n");
        } else {
          printf("Result of mode_checkout: failed\n");
        }
      }
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DogMotionNode>();
  RCLCPP_INFO(node->get_logger(), "usage:");
  RCLCPP_INFO(node->get_logger(), "\t 1: MAP; 2: AB; 3:TRACK; 100:MANUAL");
  std::string cmd =
    "ros2 topic pub --once /dog_command std_msgs/msg/Int32 \"{data: 2}\"";
  RCLCPP_INFO(node->get_logger(), "\t %s", cmd.c_str());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
