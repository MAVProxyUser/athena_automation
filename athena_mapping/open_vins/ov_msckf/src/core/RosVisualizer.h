/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef OV_MSCKF_ROSVISUALIZER_H
#define OV_MSCKF_ROSVISUALIZER_H



#include <Eigen/Core>
#include <Eigen/Dense>
//#include <geometry_msgs/TransformStamped.h>
//#include <tf/transform_datatypes.h>

#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <Eigen/src/Geometry/Translation.h>
#include <tf2/time.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/async_buffer_interface.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>
#include <memory>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/detail/camera_info__struct.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud2__struct.hpp>
#include <sensor_msgs/msg/detail/point_cloud__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/qos.hpp>
#include <cv_bridge/cv_bridge.h>
#include <boost/filesystem.hpp>
#include <athena_utils/lifecycle_node.hpp>

#include "VioManager.h"
#include "ov_core/utils/dataset_reader.h"
#include "sim/Simulator.h"

namespace ov_msckf {

/**
 * @brief Helper class that will publish results onto the ROS framework.
 *
 * Also save to file the current total state and covariance along with the
 * groundtruth if we are simulating. We visualize the following things:
 * - State of the system on TF, pose message, and path
 * - Image of our tracker
 * - Our different features (SLAM, MSCKF, ARUCO)
 * - Groundtruth trajectory if we have it
 */
class RosVisualizer {
 public:
  /**
   * @brief Default constructor
   * @param nh ROS node handler
   * @param app Core estimator manager
   * @param sim Simulator if we are simulating
   */
  RosVisualizer(athena_utils::LifecycleNode *node, std::shared_ptr<VioManager> app,
                std::shared_ptr<Simulator> sim = nullptr);

  /**
   * @brief Will visualize the system if we have new things
   */
  void visualize();

  /**
   * @brief Will publish our odometry message for the current timestep.
   * This will take the current state estimate and get the propagated pose to
   * the desired time. This can be used to get pose estimates on systems which
   * require high frequency pose estimates.
   */
  void visualize_odometry(double timestamp);

  /**
   * @brief After the run has ended, print results
   */
  void visualize_final();

  void broadcast_T(const std::string& frame_id, const std::string& child_frame_id, 
                   const Eigen::Matrix4d& trans, const double timestamp);
  
  bool visualize_active();
  bool visualize_deactive();

 protected:
  /// Publish the current state
  void publish_state();

  /// Publish the active tracking image
  void publish_images(double timestamp);

  /// Publish current features
  void publish_features(double timestamp);

  /// Core application of the filter system
  std::shared_ptr<VioManager> _app;

  /// Simulator (is nullptr if we are not sim'ing)
  std::shared_ptr<Simulator> _sim;

  /// Publish loop-closure information of current pose and active track information
  //void publish_loopclosure_information();

  /// Save current estimate state and groundtruth including calibration
  //void sim_save_total_state_to_file();

  void listen_tf(athena_utils::LifecycleNode *node);
  bool is_close;
  Eigen::Matrix4d get_T_imu_curr();
  Eigen::Matrix4d get_T_base_curr();
  Eigen::Matrix4d matrix_inverse_by_cv(const Eigen::Matrix4d& T_in);
  geometry_msgs::msg::PoseWithCovarianceStamped eigen_to_posewithcovstamped(const Eigen::Matrix4d& trans);
  geometry_msgs::msg::PoseStamped eigen_to_posestamped(const Eigen::Matrix4d& trans); 

  // Our publishers
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pub_poseimu;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_pathimu;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_msckf;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_slam;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_aruco;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_points_sim;

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr pub_tracks;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBr;
  std::unique_ptr<tf2_ros::TransformBroadcaster> slam_broadcaster_;

  // For path viz
  vector<geometry_msgs::msg::PoseStamped> poses_imu;

  // Start and end timestamps
  bool start_time_set = false;
  boost::posix_time::ptime rT1, rT2;

  // Last timestamp we visualized at
  double last_visualization_timestamp = 0;

  // For path viz
  unsigned int poses_seq_gt = 0;
  vector<geometry_msgs::msg::PoseStamped> poses_gt;
  bool publish_tf_ = true;

  std::map<std::string, std::string> frame_ids_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  // Files and if we should save total state
  bool save_total_state;
  std::ofstream of_state_est, of_state_std, of_state_gt;

  long int odom_fusion_broadcast_cnt_;
  Eigen::Matrix4d T_base_lost_;
  Eigen::Matrix4d T_foot_lost_;
  Eigen::Matrix4d delta_T_;
  Eigen::Matrix4d T_foot_curr_;
  Eigen::Matrix4d T_base_curr_;
  Eigen::Matrix4d T_fusion_curr_;
  Eigen::Matrix4d T_imu_cam_;
  Eigen::Matrix4d T_cam_base_;
  Eigen::Matrix4d T_chassis_global_;
  bool track_bad_;
  bool track_lost_edge_;
  double lost_time_;
  int fusion_odom_state_;
  double wait_vio_reset_time_;
  bool is_initial_;
  double last_track_bad_time_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_path_fusion_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr pub_path_foot_;
  unsigned int pose_fusion_seq_num_, pose_foot_seq_num_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_fusion_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_foot_;
  std::shared_ptr<std::thread> fusion_thread;
};

}  // namespace ov_msckf

#endif  // OV_MSCKF_ROSVISUALIZER_H
