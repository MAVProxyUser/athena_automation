// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2018 Simbe Robotics
// Copyright (c) 2019 Samsung Research America
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

// Navigation Strategy based on:
// Brock, O. and Oussama K. (1999). High-Speed Navigation Using
// the Global Dynamic Window Approach. IEEE.
// https://cs.stanford.edu/group/manips/publications/pdfs/Brock_1999_ICRA.pdf

// #define BENCHMARK_TESTING
#include "limited_navfn_planner/limited_navfn_planner.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "builtin_interfaces/msg/duration.hpp"
#include "limited_navfn_planner/limited_navfn.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace lnavfn_planner
{

LNavfnPlanner::LNavfnPlanner()
: tf_(nullptr), costmap_(nullptr)
{
}

LNavfnPlanner::~LNavfnPlanner()
{
  RCLCPP_INFO(
    node_->get_logger(), "Destroying plugin %s of type LNavfnPlanner",
    name_.c_str());
}

void
LNavfnPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  RCLCPP_INFO(
    node_->get_logger(), "Configuring plugin %s of type LNavfnPlanner",
    name_.c_str());

  // Initialize parameters
  // Declare this plugin's parameters
  declare_parameter_if_not_declared(node_, name + ".tolerance", rclcpp::ParameterValue(0.5));
  node_->get_parameter(name + ".tolerance", tolerance_);
  declare_parameter_if_not_declared(node_, name + ".use_astar", rclcpp::ParameterValue(false));
  node_->get_parameter(name + ".use_astar", use_astar_);
  declare_parameter_if_not_declared(node_, name + ".allow_unknown", rclcpp::ParameterValue(true));
  node_->get_parameter(name + ".allow_unknown", allow_unknown_);
  declare_parameter_if_not_declared(node_, name + ".free_radius", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name + ".free_radius", free_radius_);
  declare_parameter_if_not_declared(node_, name + ".search_radius", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name + ".search_radius", search_radius_);
  declare_parameter_if_not_declared(node_, name + ".tracking_distance", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name + ".tracking_distance", tracking_dist_);
  declare_parameter_if_not_declared(node_, name + ".visualize", rclcpp::ParameterValue(false));
  node_->get_parameter(name + ".visualize", visualize_);

  // Create a planner based on the new costmap size
  planner_ = std::make_unique<LNavFn>(
    costmap_->getSizeInCellsX(),
    costmap_->getSizeInCellsY());

  // Setup callback for changes to parameters.
  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_->get_node_base_interface(),
    node_->get_node_topics_interface(),
    node_->get_node_graph_interface(),
    node_->get_node_services_interface());

  parameter_event_sub_ = parameters_client_->on_parameter_event(
    std::bind(&LNavfnPlanner::on_parameter_event_callback, this, _1));

  std::string topic_name = name_ + "/costmap";
  auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name, custom_qos);
}

void
LNavfnPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type LNavfnPlanner",
    name_.c_str());
  costmap_pub_->on_activate();
}

void
LNavfnPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type LNavfnPlanner",
    name_.c_str());
  costmap_pub_->on_deactivate();
}

void
LNavfnPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "Cleaning up plugin %s of type LNavfnPlanner",
    name_.c_str());
  planner_.reset();
  costmap_pub_.reset();
}

nav_msgs::msg::Path LNavfnPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
#ifdef BENCHMARK_TESTING
  steady_clock::time_point a = steady_clock::now();
#endif

  // Update planner based on the new costmap size
  if (isPlannerOutOfDate()) {
    planner_->setNavArr(
      costmap_->getSizeInCellsX(),
      costmap_->getSizeInCellsY());
  }

  nav_msgs::msg::Path path;

  if (!makePlan(start.pose, goal.pose, tolerance_, path)) {
    RCLCPP_WARN(
      node_->get_logger(), "%s: failed to create plan with "
      "tolerance %.2f.", name_.c_str(), tolerance_);
  }

#ifdef BENCHMARK_TESTING
  steady_clock::time_point b = steady_clock::now();
  duration<double> time_span = duration_cast<duration<double>>(b - a);
  std::cout << "It took " << time_span.count() * 1000 <<
    " milliseconds with " << num_iterations << " iterations." << std::endl;
#endif

  return path;
}

bool
LNavfnPlanner::isPlannerOutOfDate()
{
  if (!planner_.get() ||
    planner_->nx != static_cast<int>(costmap_->getSizeInCellsX()) ||
    planner_->ny != static_cast<int>(costmap_->getSizeInCellsY()))
  {
    return true;
  }
  return false;
}

void
LNavfnPlanner::setSearchArea (int *start, int *goal) {
    int nx = costmap_->getSizeInCellsX();
    int ny = costmap_->getSizeInCellsY();
    if (search_radius_ <= 0.0) {
        memset((void*)planner_->limited, false, nx * ny * sizeof(bool));
        return;
    }

    int cell_search_radius = costmap_->cellDistance(search_radius_);
    double diff_x = goal[0] - start[0];
    double diff_y = goal[1] - start[1];
    int steps = std::max(std::abs(diff_x), std::abs(diff_y));

    double step_x = diff_x / steps;
    double step_y = diff_y / steps;

    // mark cells, a distance less than search_radius, that used for searching
    bool *pending = new bool[nx * ny];
    memset((void*)pending, 0, nx * ny * sizeof(bool));
    int x, y;
    for (int i = 0; i < steps; ++i) {
        x = start[0] + i * step_x;
        y = start[1] + i * step_y;

        int min_x = std::max< int>(0, x - cell_search_radius);
        int min_y = std::max< int>(0, y - cell_search_radius);
        int max_x = std::min< int>(nx - 1, x + cell_search_radius);
        int max_y = std::min< int>(ny - 1, y + cell_search_radius);
        for (int i = min_x; i < max_x; ++i) {
            for (int j = min_y; j < max_y; ++j) {
                if (std::hypot(i - x, j - y) >= cell_search_radius) continue;
                int idx = i + j * nx;
                if (pending[idx] == true) continue;
                planner_->setLimit(idx);
                pending[idx] = true;
            }
        }
    }
    if (pending) delete [] pending;
}

void
LNavfnPlanner::setFreeArea (int *goal) {
    unsigned int cell_free_radius = costmap_->cellDistance(free_radius_);
    int min_cell_x = std::max< int>(1, goal[0] - cell_free_radius);
    int min_cell_y = std::max< int>(1, goal[1] - cell_free_radius);
    int max_cell_x = std::min< int>(costmap_->getSizeInCellsX() - 2, goal[0] + cell_free_radius);
    int max_cell_y = std::min< int>(costmap_->getSizeInCellsY() - 2, goal[1] + cell_free_radius);

    int nx = costmap_->getSizeInCellsX();
    for (int i = min_cell_x; i < max_cell_x; ++i) {
        for (int j = min_cell_y; j < max_cell_y; ++j) {
            if (std::hypot(i - goal[0], j - goal[1]) >= cell_free_radius) continue;
            int idx = i + j * nx;
            planner_->setCost(idx);
        }
    }
}

void
LNavfnPlanner::visualize () {
    if (node_->count_subscribers(costmap_pub_->get_topic_name()) > 0) {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.frame_id = global_frame_;
        msg.header.stamp = node_->now();
        msg.info.resolution = costmap_->getResolution();
        msg.info.width = costmap_->getSizeInCellsX();
        msg.info.height = costmap_->getSizeInCellsY();
        msg.info.origin.position.x = costmap_->getOriginX();
        msg.info.origin.position.y = costmap_->getOriginY();
        msg.info.origin.position.z = 0.001;
        msg.info.origin.orientation.w = 1;

        int ns = costmap_->getSizeInCellsX() * costmap_->getSizeInCellsY();
        msg.data.resize(ns);

        COSTTYPE *cm = planner_->costarr;
        bool *lm = planner_->limited;
        for (int i = 0; i < ns; ++i, ++cm, ++lm) {
            if (*lm) {
                msg.data[i] = 100;
                continue;
            }
            auto v = *cm;
            int mc;
            if (v == 255) {
                mc = -1;
            } else if (v == 254){
                mc = 100;
            } else if (v == 253) {
                mc = 99;
            } else {
                mc = (1 + (97 * ((v - COST_NEUTRAL) / COST_FACTOR - 1)) / 251);
            }
            msg.data[i] = mc;
        }
        costmap_pub_->publish(msg);
    }

}

bool
LNavfnPlanner::makePlan(
  const geometry_msgs::msg::Pose & start,
  const geometry_msgs::msg::Pose & goal, double tolerance,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  plan.header.stamp = node_->now();
  plan.header.frame_id = global_frame_;

  // TODO(orduno): add checks for start and goal reference frame -- should be in global frame

  double wx = start.position.x;
  double wy = start.position.y;

  RCLCPP_DEBUG(
    node_->get_logger(), "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
    start.position.x, start.position.y, goal.position.x, goal.position.y);

  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Cannot create a plan: the robot's start position is off the global"
      " costmap. Planning will always fail, are you sure"
      " the robot has been properly localized?");
    return false;
  }

  // clear the starting cell within the costmap because we know it can't be an obstacle
  clearRobotCell(mx, my);

  {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

      // make sure to resize the underlying array that Navfn uses
      planner_->setNavArr(
        costmap_->getSizeInCellsX(),
        costmap_->getSizeInCellsY());

      planner_->setCostmap(costmap_->getCharMap(), true, allow_unknown_);

      lock.unlock();
  }

  int map_start[2];
  map_start[0] = mx;
  map_start[1] = my;

  wx = goal.position.x;
  wy = goal.position.y;

  if (!worldToMap(wx, wy, mx, my)) {
	  // lookup the edge goal btw start and goal
	  {
          double res = costmap_->getResolution();
		  double dx = goal.position.x - start.position.x;
		  double dy = goal.position.y - start.position.y;
		  double up_ratio = 1.0, down_ratio = 0.0;
		  while (true) {
			  double cur_ratio = (up_ratio + down_ratio) * 0.5;
			  double tmp_x = goal.position.x - dx * cur_ratio;
			  double tmp_y = goal.position.y - dy * cur_ratio;
			  if(!worldToMap(tmp_x, tmp_y, mx, my))
				  down_ratio = cur_ratio;
			  else
				  up_ratio = cur_ratio;
			  if (up_ratio - down_ratio < res / std::hypot(dx, dy)) break;
		  }

		  wx -= dx * up_ratio;
		  wy -= dy * up_ratio;
		  worldToMap(wx, wy, mx, my);

          // edge of costarr will be printed as COST_OBS in LNavFn's setupNavFn 
          if ((int)mx == 0) wx += res;
          else if ((int)mx == planner_->nx - 1) wx -= res;
          if ((int)my == 0) wy += res;
          else if ((int)my == planner_->ny - 1) wy -= res;
	  }
	  if (!worldToMap(wx, wy, mx, my)) {
		  RCLCPP_WARN(
					  node_->get_logger(),
					  "The goal sent to the planner is off the global costmap."
					  " Planning will always fail to this goal.");
		  return false;
	  }
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  // TODO(orduno): Explain why we are providing 'map_goal' to setStart().
  //               Same for setGoal, seems reversed. Computing backwards?

  setSearchArea(map_start, map_goal);
  setFreeArea(map_goal);

  if (visualize_) {
      visualize();
  }

  planner_->setStart(map_goal);
  planner_->setGoal(map_start);
  if (use_astar_) {
    planner_->calcNavFnAstar();
  } else {
    planner_->calcNavFnDijkstra(true);
  }

  double resolution = costmap_->getResolution();
  geometry_msgs::msg::Pose p, best_pose;

  bool found_legal = false;

  p.position.x = wx;
  p.position.y = wy;
  p.orientation = goal.orientation;
  double potential = getPointPotential(p.position);
  if (potential < POT_HIGH) {
    // Goal is reachable by itself
    best_pose = p;
    found_legal = true;
    RCLCPP_INFO(node_->get_logger(), "%i found_legal: %i", __LINE__, found_legal);
  } else {
    // Goal is not reachable. Trying to find nearest to the goal
    // reachable point within its tolerance region
    double best_sdist = std::numeric_limits<double>::max();

    p.position.y = goal.position.y - tolerance;
    while (p.position.y <= goal.position.y + tolerance) {
      p.position.x = goal.position.x - tolerance;
      while (p.position.x <= goal.position.x + tolerance) {
        potential = getPointPotential(p.position);
        double sdist = squared_distance(p, goal);
        if (potential < POT_HIGH && sdist < best_sdist) {
          best_sdist = sdist;
          best_pose = p;
          found_legal = true;
        }
        p.position.x += resolution;
      }
      p.position.y += resolution;
    }
    RCLCPP_INFO(node_->get_logger(), "%i found_legal: %i", __LINE__, found_legal);
  }

  if (found_legal) {
    // extract the plan
    if (getPlanFromPotential(best_pose, plan)) {
      smoothApproachToGoal(best_pose, plan);
      RCLCPP_INFO(node_->get_logger(), "plan poses size: %i", plan.poses.size());
    } else {
      RCLCPP_ERROR(
        node_->get_logger(),
        "Failed to create a plan from potential when a legal"
        " potential was found. This shouldn't happen.");
    }
  }
  RCLCPP_INFO(node_->get_logger(), "plan poses size: %i", plan.poses.size());

  if (tracking_dist_ > 0.0 && !plan.poses.empty()) {
      // find pose with a distance greater thean tracking_dist_;
      auto erase_it = std::find_if(
            std::rbegin(plan.poses), std::rend(plan.poses), 
            [&](const auto &pose) {
                double diff_x = pose.pose.position.x - goal.position.x;
                double diff_y = pose.pose.position.y - goal.position.y;
                return std::hypot(diff_x, diff_y) >= tracking_dist_;
            });
      plan.poses.erase(erase_it.base(), plan.poses.end());
      if (plan.poses.empty()) {
          plan.poses.push_back(geometry_msgs::msg::PoseStamped());
          plan.poses.back().pose = start;
      }
      double dx = goal.position.x - plan.poses.back().pose.position.x;
      double dy = goal.position.y - plan.poses.back().pose.position.y;
      dy = (dy == 0) ? 1e-3 : dy;
      double yaw = std::atan2(dy, dx);
      tf2::Quaternion q;
      q.setRPY(0.f, 0.f, yaw);
      plan.poses.back().pose.orientation = tf2::toMsg(q);
  }
  RCLCPP_INFO(node_->get_logger(), "plan poses size: %i", plan.poses.size());

  return !plan.poses.empty();
}

void
LNavfnPlanner::smoothApproachToGoal(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // Replace the last pose of the computed path if it's actually further away
  // to the second to last pose than the goal pose.
  if (plan.poses.size() >= 2) {
    auto second_to_last_pose = plan.poses.end()[-2];
    auto last_pose = plan.poses.back();
    if (
      squared_distance(last_pose.pose, second_to_last_pose.pose) >
      squared_distance(goal, second_to_last_pose.pose))
    {
      plan.poses.back().pose = goal;
      return;
    }
  }
  geometry_msgs::msg::PoseStamped goal_copy;
  goal_copy.pose = goal;
  plan.poses.push_back(goal_copy);
}

bool
LNavfnPlanner::getPlanFromPotential(
  const geometry_msgs::msg::Pose & goal,
  nav_msgs::msg::Path & plan)
{
  // clear the plan, just in case
  plan.poses.clear();

  // Goal should be in global frame
  double wx = goal.position.x;
  double wy = goal.position.y;

  // the potential has already been computed, so we won't update our copy of the costmap
  unsigned int mx, my;
  if (!worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "The goal sent to the lnavfn planner is off the global costmap."
      " Planning will always fail to this goal.");
    return false;
  }

  int map_goal[2];
  map_goal[0] = mx;
  map_goal[1] = my;

  planner_->setStart(map_goal);

  const int & max_cycles = (costmap_->getSizeInCellsX() >= costmap_->getSizeInCellsY()) ?
    (costmap_->getSizeInCellsX() * 4) : (costmap_->getSizeInCellsY() * 4);

  int path_len = planner_->calcPath(max_cycles);
  if (path_len == 0) {
    return false;
  }

  auto cost = planner_->getLastPathCost();
  RCLCPP_DEBUG(node_->get_logger(), "Path found, %d steps, %f cost\n", path_len, cost);

  // extract the plan
  float * x = planner_->getPathX();
  float * y = planner_->getPathY();
  int len = planner_->getPathLen();

  for (int i = len - 1; i >= 0; --i) {
    // convert the plan to world coordinates
    double world_x, world_y;
    mapToWorld(x[i], y[i], world_x, world_y);

    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = world_x;
    pose.pose.position.y = world_y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    plan.poses.push_back(pose);
  }

  return !plan.poses.empty();
}

double
LNavfnPlanner::getPointPotential(const geometry_msgs::msg::Point & world_point)
{
  unsigned int mx, my;
  if (!worldToMap(world_point.x, world_point.y, mx, my)) {
    return std::numeric_limits<double>::max();
  }

  unsigned int index = my * planner_->nx + mx;
  return planner_->potarr[index];
}

// bool
// LNavfnPlanner::validPointPotential(const geometry_msgs::msg::Point & world_point)
// {
//   return validPointPotential(world_point, tolerance_);
// }

// bool
// LNavfnPlanner::validPointPotential(
//   const geometry_msgs::msg::Point & world_point, double tolerance)
// {
//   const double resolution = costmap_->getResolution();

//   geometry_msgs::msg::Point p = world_point;
//   double potential = getPointPotential(p);
//   if (potential < POT_HIGH) {
//     // world_point is reachable by itself
//     return true;
//   } else {
//     // world_point, is not reachable. Trying to find any
//     // reachable point within its tolerance region
//     p.y = world_point.y - tolerance;
//     while (p.y <= world_point.y + tolerance) {
//       p.x = world_point.x - tolerance;
//       while (p.x <= world_point.x + tolerance) {
//         potential = getPointPotential(p);
//         if (potential < POT_HIGH) {
//           return true;
//         }
//         p.x += resolution;
//       }
//       p.y += resolution;
//     }
//   }

//   return false;
// }

bool
LNavfnPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my)
{
  if (wx < costmap_->getOriginX() || wy < costmap_->getOriginY()) {
    return false;
  }

  mx = static_cast<int>(
    std::round((wx - costmap_->getOriginX()) / costmap_->getResolution()));
  my = static_cast<int>(
    std::round((wy - costmap_->getOriginY()) / costmap_->getResolution()));

  if (mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY()) {
    return true;
  }

  RCLCPP_ERROR(
    node_->get_logger(), "worldToMap failed: mx,my: %d,%d, size_x,size_y: %d,%d", mx, my,
    costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());

  return false;
}

void
LNavfnPlanner::mapToWorld(double mx, double my, double & wx, double & wy)
{
  wx = costmap_->getOriginX() + mx * costmap_->getResolution();
  wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

void
LNavfnPlanner::clearRobotCell(unsigned int mx, unsigned int my)
{
  // TODO(orduno): check usage of this function, might instead be a request to
  //               world_model / map server
  costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

void
LNavfnPlanner::on_parameter_event_callback(
  const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  for (auto & changed_parameter : event->changed_parameters) {
    const auto & type = changed_parameter.value.type;
    const auto & name = changed_parameter.name;
    const auto & value = changed_parameter.value;

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == name_ + ".tolerance") {
        tolerance_ = value.double_value;
      } else if (name == name_ + ".free_radius") {
        free_radius_ = value.double_value;
      } else if (name == name_ + ".search_radius") {
        search_radius_ = value.double_value;
      } else if (name == name_ + ".tracking_distance") {
        tracking_dist_ = value.double_value;
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == name_ + ".use_astar") {
        use_astar_ = value.bool_value;
      } else if (name == name_ + ".allow_unknown") {
        allow_unknown_ = value.bool_value;
      } else if (name == name_ + ".visualize") {
        visualize_ = value.bool_value;
      }
    }
  }
}

}  // namespace lnavfn_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lnavfn_planner::LNavfnPlanner, nav2_core::GlobalPlanner)
