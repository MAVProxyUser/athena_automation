#include "dm_planner/distance_map_planner_node.h"
#include <nav2_util/node_utils.hpp>

using namespace std::chrono_literals;
using nav2_util::declare_parameter_if_not_declared;

namespace dm_planner {

DMPlanner::DMPlanner () 
    : node_(nullptr), tf_(nullptr), costmap_(nullptr)
{}

DMPlanner::~DMPlanner () {
    RCLCPP_INFO(
        node_->get_logger(), "Destroying plugin %s of type DMPlanner",
        name_.c_str());
}

void 
DMPlanner::configure (
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
    node_ = parent;
    tf_ = tf;
    name_ = name;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    double free_radius, search_radius;

    declare_parameter_if_not_declared(node_, name + ".visualize", rclcpp::ParameterValue(false));
    declare_parameter_if_not_declared(node_, name + ".free_radius", rclcpp::ParameterValue(0.));
    declare_parameter_if_not_declared(node_, name + ".search_radius", rclcpp::ParameterValue(1.));
    declare_parameter_if_not_declared(node_, name + ".tracking_distance", rclcpp::ParameterValue(1.));

    node_->get_parameter(name + ".visualize", visualize_);
    node_->get_parameter(name + ".free_radius", free_radius);
    node_->get_parameter(name + ".search_radius", search_radius);
    node_->get_parameter(name + ".tracking_distance", tracking_dist_);

    std::string topic_name = name_ + "/costmap";
    auto custom_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    costmap_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>( topic_name, custom_qos);

    dmp_ = std::make_shared< dmp::DMPlanner>();
    dmp_->freeGoal(free_radius);
    dmp_->setSearchRadius(search_radius);

    set_parameters_callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&DMPlanner::parametersCallback, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult
DMPlanner::parametersCallback( const std::vector< rclcpp::Parameter> params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto param : params) {
        if (param.get_name() == name_ + ".visualize" &&
            param.get_type() == rclcpp::ParameterType::PARAMETER_BOOL) {
            visualize_ = param.as_bool();
            RCLCPP_INFO(node_->get_logger(), "Parameter %s changed: %i", param.get_name().c_str(), param.as_bool());
        }
        if (param.get_name() == name_ + ".free_radius" &&
            param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            double free_radius = param.as_double();
            dmp_->freeGoal(free_radius);
            RCLCPP_INFO(node_->get_logger(), "Parameter %s changed: %f", param.get_name().c_str(), free_radius);
        }
        if (param.get_name() == name_ + ".search_radius" &&
            param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            double search_radius = param.as_double();
            dmp_->setSearchRadius(search_radius);
            RCLCPP_INFO(node_->get_logger(), "Parameter %s changed: %f", param.get_name().c_str(), search_radius);
        }
        if (param.get_name() == name_ + ".tracking_distance" &&
            param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
            tracking_dist_ = param.as_double();
            RCLCPP_INFO(node_->get_logger(), "Parameter %s changed: %f", param.get_name().c_str(), tracking_dist_);
        }
    }
    return result;
}

void 
DMPlanner::cleanup () {
    costmap_pub_.reset();
}

void 
DMPlanner::activate () {
    RCLCPP_INFO(
        node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
        name_.c_str());
    costmap_pub_->on_activate();
}

void 
DMPlanner::deactivate () {
    RCLCPP_INFO(
        node_->get_logger(), "Cleaning plugin %s of type NavfnPlanner",
        name_.c_str());
    costmap_pub_->on_deactivate();
}

nav_msgs::msg::Path 
DMPlanner::createPlan (
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) {

    nav_msgs::msg::Path path;

    if (!makePlan(start.pose, goal.pose, path)) {
        RCLCPP_WARN( node_->get_logger(), "%s: failed to create plan" , name_.c_str());
    }

    return path;
}

bool 
DMPlanner::makePlan (
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Pose & goal,
    nav_msgs::msg::Path & plan) {
    plan.poses.clear();
    plan.header.stamp = node_->now();
    plan.header.frame_id = global_frame_;

    double res = costmap_->getResolution();

    double wx = start.position.x;
    double wy = start.position.y;

    RCLCPP_DEBUG(
        node_->get_logger(), "Making plan from (%.2f,%.2f) to (%.2f,%.2f)",
        start.position.x, start.position.y, goal.position.x, goal.position.y);

    unsigned int mx, my;
    if (!costmap_->worldToMap(wx, wy, mx, my)) {
        RCLCPP_WARN(
            node_->get_logger(),
            "Cannot create a plan: the robot's start position is off the global"
            " costmap. Planning will always fail, are you sure"
            " the robot has been properly localized?");
        return false;
    }

    clearRobotCell(mx, my);

    wx = goal.position.x;
    wy = goal.position.y;

    if (!costmap_->worldToMap(wx, wy, mx, my)) {
        // lookup the edge goal btw start and goal
        {
            double dx = goal.position.x - start.position.x;
            double dy = goal.position.y - start.position.y;
            double up_ratio = 1.0, down_ratio = 0.0;
            while (true) {
                double cur_ratio = (up_ratio + down_ratio) * 0.5;
                double tmp_x = goal.position.x - dx * cur_ratio;
                double tmp_y = goal.position.y - dy * cur_ratio;
                if(!costmap_->worldToMap(tmp_x, tmp_y, mx, my)) 
                    down_ratio = cur_ratio;
                else 
                    up_ratio = cur_ratio;
                if (up_ratio - down_ratio < res / std::hypot(dx, dy)) break;
            }

            wx -= dx * (up_ratio - down_ratio) * 0.5;
            wy -= dy * (up_ratio - down_ratio) * 0.5;
            double n_steps = std::max(std::round(dx / res), std::round(dy / res));
            for (int i = 0; i <= n_steps; ++i) {
                wx -= dx / n_steps;
                wy -= dy / n_steps;
                if (costmap_->worldToMap(wx, wy, mx, my)) break;
            }
        }
        if (!costmap_->worldToMap(wx, wy, mx, my)) {
            RCLCPP_WARN(
                node_->get_logger(),
                "The goal sent to the planner is off the global costmap."
                " Planning will always fail to this goal.");
            return false;
        }
    }

    {
        std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));

        dmp::Position origin(costmap_->getOriginX(), costmap_->getOriginY());
        dmp::Size size(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
        dmp_->setMap(costmap_->getCharMap(), origin, size, res);

        lock.unlock();
    }

    dmp::Position start_p(start.position.x, start.position.y);
    dmp::Position goal_p(goal.position.x, goal.position.y);
    dmp::Position real_p(wx, wy);
    std::vector< dmp::Position> ps;
    ps.push_back(start_p);
    ps.push_back(real_p);
    dmp_->computePath(start_p, real_p, ps, false);

    if (visualize_) visualize();

    int status = dmp_->status();
    if (status != 0) {
        std::string err = "";
        if (status == -1) err = "cannot find a valid plan.";
        if (status == 1) err = "start pose is not free.";
        if (status == 2) err = "goal pose is not free.";
        RCLCPP_ERROR( node_->get_logger(), err);
        return !plan.poses.empty();
    }
    auto path = dmp_->getPath();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    for (const auto &it : path) {
        if (!plan.poses.empty() && (goal_p - it).norm() < tracking_dist_) break;
        pose.pose.position.x = it.x();
        pose.pose.position.y = it.y();
        plan.poses.push_back(pose);
    }

    // final pose towards to the goal
    double dx = goal.position.x - plan.poses.back().pose.position.x;
    double dy = goal.position.y - plan.poses.back().pose.position.y;
    dy = (dy == 0) ? 1e-3 : dy;
    double yaw = std::atan2(dy, dx);
    tf2::Quaternion q;
    q.setRPY(0.f, 0.f, yaw);
    plan.poses.back().pose.orientation = tf2::toMsg(q);

    return !plan.poses.empty();
}

void
DMPlanner::clearRobotCell(uint8_t mx, uint8_t my) {
    costmap_->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
}

uint8_t
DMPlanner::getCostValue (double x, double y) {
    unsigned int mx, my;
    if (!costmap_->worldToMap(x, y, mx, my)) {
        return nav2_costmap_2d::LETHAL_OBSTACLE;
    }
    return costmap_->getCost(mx, my);
}

void
DMPlanner::visualize () {
    if (node_->count_subscribers(costmap_pub_->get_topic_name()) > 0) {
        auto cost_map = dmp_->getMap();
        auto res = cost_map->getRes();
        auto size = cost_map->getSize();
        auto origin = cost_map->getOrigin();
        auto data = cost_map->getMap();

        nav_msgs::msg::OccupancyGrid msg;
        msg.header.frame_id = global_frame_;
        msg.header.stamp = node_->now();
        msg.info.resolution = res;
        msg.info.width = size.x();
        msg.info.height = size.y();
        msg.info.origin.position.x = origin.x();
        msg.info.origin.position.y = origin.y();
        msg.info.origin.position.z = 0.001;
        msg.info.origin.orientation.w = 1;

        int ns = size.x() * size.y();
        msg.data.resize(ns);

        for (int i = 0; i < ns; ++i) {
            auto v = data[i];
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
} // dm_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dm_planner::DMPlanner, nav2_core::GlobalPlanner)
