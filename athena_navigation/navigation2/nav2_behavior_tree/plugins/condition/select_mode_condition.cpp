// Copyright (c) 2019 Intel Corporation
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

#include <string>
#include <memory>
#include "nav2_behavior_tree/plugins/condition/select_mode_condition.hpp"

namespace nav2_behavior_tree
{

SelectModeCondition::SelectModeCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  std::string server_name = "get_control_mode";
  //getInput("get_control_mode", server_name);
  //node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  node_ = std::make_shared< rclcpp::Node>("select_mode_node");
  get_mode_ = node_->create_service<athena_interfaces::srv::NavMode>(
    "get_mode",
    std::bind(
      &SelectModeCondition::getModeCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  auto spin_func = [&]() {
    rclcpp::spin(node_->get_node_base_interface());
  };

  std::thread spin_thread(spin_func);
  spin_thread.detach();
}

SelectModeCondition::~SelectModeCondition()
{
  cleanup();
}

BT::NodeStatus SelectModeCondition::tick()
{
  return BT::NodeStatus::SUCCESS;
}

void
SelectModeCondition::getModeCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<athena_interfaces::srv::NavMode::Request> req,
  std::shared_ptr<athena_interfaces::srv::NavMode::Response> res)
{
  res->success = false;
  switch(req->control_mode){
    case SubMode_T::EXPLR_NAV_AB:
      RCLCPP_INFO(node_->get_logger(), "Change mode to navigating");
      config().blackboard->set< std::string>("nav_mode", "navigating");
      setControllerTrackingMode(false);
      break;
    case SubMode_T::TRACK_F:
    case SubMode_T::TRACK_S:
      RCLCPP_INFO(node_->get_logger(), "Change mode to tracking");
      setControllerTrackingMode(true);
      config().blackboard->set< std::string>("nav_mode", "tracking");
      break;
    case SubMode_T::EXPLR_MAP_UPDATE:
    case SubMode_T::EXPLR_MAP_NEW:
    case SubMode_T::MODE_STOP:
      RCLCPP_INFO(node_->get_logger(), "Stop navigation");
      config().blackboard->set< std::string>("nav_mode", "none");
      break;
    default :
      return;
  }

  res->success = true;
}

bool
SelectModeCondition::setControllerTrackingMode (bool enable) {
    auto client = node_->create_client< rcl_interfaces::srv::SetParameters>("controller_server/set_parameters");
    client->wait_for_service();

    auto request = std::make_shared< rcl_interfaces::srv::SetParameters::Request>();
    rclcpp::Parameter tracking_mode("PurePersuit.tracking_mode", enable);
    request->parameters.push_back(tracking_mode.to_parameter_msg());

    auto future = client->async_send_request(request);
    //rclcpp::spin_until_future_complete(node_, future);
    
    //RCLCPP_INFO(node_->get_logger(), "Change controller tracking_mode %i",
    //    future.get()->results.front().successful);

    return true;
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SelectModeCondition>("SelectMode");
}
