// Copyright (c) 2021 XIAOMI Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__SELECT_MODE_CONDITION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__SELECT_MODE_CONDITION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <athena_interfaces/srv/nav_mode.hpp>
#include "behaviortree_cpp_v3/condition_node.h"

namespace nav2_behavior_tree
{

class SelectModeCondition : public BT::ConditionNode
{
  using SubMode_T = athena_interfaces::srv::NavMode_Request;
public:
  SelectModeCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  SelectModeCondition() = delete;

  ~SelectModeCondition() override;

  BT::NodeStatus tick() override;

  void getModeCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<athena_interfaces::srv::NavMode::Request> req,
    std::shared_ptr<athena_interfaces::srv::NavMode::Response> res);

  static BT::PortsList providedPorts()
  {
    return {
      /*
      BT::InputPort<std::string>("mode_server", std::string("get_control_mode"), "query control mode"),
      BT::InputPort<std::string>("nav_mode", std::string("none"), ""),
      */
    };
  }

protected:
  void cleanup()
  {}

private:

  bool setControllerTrackingMode (bool enable);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Service< athena_interfaces::srv::NavMode>::SharedPtr get_mode_;
  rclcpp::Client< rcl_interfaces::srv::ListParameters>::SharedPtr client_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONDITION__SELECT_MODE_CONDITION_HPP_
