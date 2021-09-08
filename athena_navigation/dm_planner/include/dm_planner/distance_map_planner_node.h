#pragma once

#include "dm_planner/distance_map_planner.h"

#include <nav_msgs/msg/path.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace dm_planner {

class DMPlanner : public nav2_core::GlobalPlanner
{
public:
    DMPlanner ();
    ~DMPlanner ();

    // plugin configure
    void configure (
        rclcpp_lifecycle::LifecycleNode::SharedPtr parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    // plugin cleanup
    void cleanup () override;

    // plugin activate
    void activate () override;

    // plugin deactivate
    void deactivate () override;


    // plugin create path
    nav_msgs::msg::Path createPlan (
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal) override;

protected:
    // Compute a plan given start and goal poses, provided in global world frame.
    bool makePlan (
        const geometry_msgs::msg::Pose & start,
        const geometry_msgs::msg::Pose & goal,
        nav_msgs::msg::Path & plan);

    void clearRobotCell(uint8_t mx, uint8_t my);

    inline double squared_distance (
		const geometry_msgs::msg::Pose & p1,
        const geometry_msgs::msg::Pose & p2)
    {
        double dx = p1.position.x - p2.position.x;
        double dy = p1.position.y - p2.position.y;
        return dx * dx + dy * dy;
    }

    uint8_t getCostValue (double x, double y);

    rcl_interfaces::msg::SetParametersResult
    parametersCallback(const std::vector< rclcpp::Parameter> params);

    void visualize ();
    bool visualize_;

    std::string global_frame_, name_;
    nav2_util::LifecycleNode::SharedPtr node_;
    std::shared_ptr< tf2_ros::Buffer> tf_;
    nav2_costmap_2d::Costmap2D *costmap_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_handle_;

    std::shared_ptr< dmp::DMPlanner> dmp_;
    double tracking_dist_;
};

} // dm_planner
