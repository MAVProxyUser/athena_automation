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
#include "RosVisualizer.h"
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
#include <iostream>
#include <fstream>

using namespace ov_msckf;

RosVisualizer::RosVisualizer(athena_utils::LifecycleNode *node,
                             std::shared_ptr<VioManager> app,
                             std::shared_ptr<Simulator> sim)
    : _app(app), _sim(sim) {
  
  // setup frame_id
  // TODO (yangsheng): parameterization
  frame_ids_.emplace("odom", "odom");
  frame_ids_.emplace("imu", "camera_imu_optical_frame");
  frame_ids_.emplace("base_link", "base_footprint");
  frame_ids_.emplace("cam0", "camera_infra1_optical_frame");

  // Setup our transform broadcaster
  mTfBr.reset(new tf2_ros::TransformBroadcaster(*node));
  slam_broadcaster_.reset(new tf2_ros::TransformBroadcaster(*node));

  // Setup transform broadcaster and listener
  tf_broadcaster_.reset(new tf2_ros::TransformBroadcaster(*node));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Setup pose and path publisher
  pub_poseimu =
      node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "ov_msckf/poseimu", 2);
  RCLCPP_INFO(node->get_logger(), "Publishing: %s",
              pub_poseimu->get_topic_name());
  pub_pathimu =
      node->create_publisher<nav_msgs::msg::Path>("ov_msckf/pathimu", 2);
  RCLCPP_INFO(node->get_logger(), "Publishing: %s",
              pub_pathimu->get_topic_name());

  // 3D points publishing
  //pub_points_msckf = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      //"ov_msckf/points_msckf", 2);
  //RCLCPP_INFO(node->get_logger(), "Publishing: %s",
              //pub_points_msckf->get_topic_name());
  //pub_points_slam = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      //"ov_msckf/points_slam", 2);
  //RCLCPP_INFO(node->get_logger(), "Publishing: %s",
              //pub_points_msckf->get_topic_name());
  //pub_points_aruco = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      //"ov_msckf/points_aruco", 2);
  //RCLCPP_INFO(node->get_logger(), "Publishing: %s",
              //pub_points_aruco->get_topic_name());
  //pub_points_sim = node->create_publisher<sensor_msgs::msg::PointCloud2>(
      //"ov_msckf/points_sim", 2);
  //RCLCPP_INFO(node->get_logger(), "Publishing: %s",
              //pub_points_sim->get_topic_name());

  // Our tracking image
  pub_tracks =
      node->create_publisher<sensor_msgs::msg::Image>("ov_msckf/trackhist", 2);
  RCLCPP_INFO(node->get_logger(), "Publishing: %s",
              pub_tracks->get_topic_name());

  // option to enable publishing of global to IMU transformation
#if 0 //error occur when reset viz, tmp close it
  node->declare_parameter<bool>("publish_tf", true);
  node->get_parameter("publish_tf", publish_tf_);
#endif

  odom_fusion_broadcast_cnt_ = 0;

  T_base_lost_ = Eigen::Matrix4d::Identity();
  T_foot_lost_ = Eigen::Matrix4d::Identity();
  delta_T_ = Eigen::Matrix4d::Identity();
  T_foot_curr_ = Eigen::Matrix4d::Identity();
  T_base_curr_ = Eigen::Matrix4d::Identity();
  T_fusion_curr_ = Eigen::Matrix4d::Identity();
  T_imu_cam_ <<   1.0, 0.0, 0.0, -0.0302,
                  0.0, 1.0, 0.0, 0.0074,
                  0.0, 0.0, 1.0, 0.0160,
                  0.0, 0.0, 0.0, 1.0;
  T_cam_base_ <<  0.0, -1.0, 0.0, 0.048,
                  0.0, 0.0, -1.0, 0.048,
                  1.0, 0.0, 0.0, -0.387,
                  0.0, 0.0, 0.0, 1.0;
  T_chassis_global_ <<  0.0, 1.0, 0.0, 0.0,
                     -1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 1.0, 0.0,
                     0.0, 0.0, 0.0, 1.0;
  track_bad_ = false;
  track_lost_edge_ = false;
  is_initial_ = true;
  lost_time_ = 0.0;
  fusion_odom_state_ = 0;  // 0:vio, 1:vio->foot, 2:foot, 3:foot->vio
  wait_vio_reset_time_ = 5.0;  // after 5s vio reset is stable
  pose_fusion_seq_num_ = 0;
  pose_foot_seq_num_ = 0;
  last_track_bad_time_ = 0.0;
  fusion_thread = std::make_shared<std::thread>(&RosVisualizer::listen_tf, this, node);
  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  
  pub_path_fusion_ = node->create_publisher<nav_msgs::msg::Path>("ov_msckf/odomfusion", 2);
  pub_path_foot_ = node->create_publisher<nav_msgs::msg::Path>("ov_msckf/odomfoot", 2);
  is_close = false;
}

bool RosVisualizer::visualize_active() {
  pub_poseimu->on_activate();
  pub_pathimu->on_activate(); 
  //pub_points_msckf->on_activate();
  //pub_points_slam->on_activate();
  //pub_points_aruco->on_activate();
  //pub_points_sim->on_activate();
  pub_tracks->on_activate();
  pub_path_fusion_->on_activate();
  pub_path_foot_->on_activate();
  return true;
}

bool RosVisualizer::visualize_deactive() {
  is_close = true;
  fusion_thread->join();
  pub_poseimu->on_deactivate();
  pub_pathimu->on_deactivate(); 
  //pub_points_msckf->on_deactivate();
  //pub_points_slam->on_deactivate();
  //pub_points_aruco->on_deactivate();
  //pub_points_sim->on_deactivate();
  pub_tracks->on_deactivate();
  pub_path_fusion_->on_deactivate();
  pub_path_foot_->on_deactivate();
  return true;
}

void RosVisualizer::visualize() {
  // Return if we have already visualized
  if (last_visualization_timestamp == _app->get_state()->_timestamp) return;
  last_visualization_timestamp = _app->get_state()->_timestamp;

  // Start timing
  boost::posix_time::ptime rT0_1, rT0_2;
  rT0_1 = boost::posix_time::microsec_clock::local_time();

  // publish current image
  //publish_images(last_visualization_timestamp);

  // Return if we have not inited
  if (!_app->initialized()) return;

  // Save the start time of this dataset
  if (!start_time_set) {
    rT1 = boost::posix_time::microsec_clock::local_time();
    start_time_set = true;
  }

  // publish state
  publish_state();

  // publish points
  //publish_features(last_visualization_timestamp);

#if 0
  // Print how much time it took to publish / displaying things
  rT0_2 = boost::posix_time::microsec_clock::local_time();
  double time_total = (rT0_2 - rT0_1).total_microseconds() * 1e-6;
  printf(BLUE "[TIME]: %.4f seconds for visualization\n" RESET, time_total);
#endif
}

void RosVisualizer::visualize_final() {
  // Final time offset value
  if (_app->get_state()->_options.do_calib_camera_timeoffset) {
    printf(REDPURPLE "camera-imu timeoffset = %.5f\n\n" RESET,
           _app->get_state()->_calib_dt_CAMtoIMU->value()(0));
  }

  // Final camera intrinsics
  if (_app->get_state()->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < _app->get_state()->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = _app->get_state()->_cam_intrinsics.at(i);
      printf(REDPURPLE "cam%d intrinsics:\n" RESET, (int)i);
      printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n" RESET, calib->value()(0),
             calib->value()(1), calib->value()(2), calib->value()(3));
      printf(REDPURPLE "%.5f,%.5f,%.5f,%.5f\n\n" RESET, calib->value()(4),
             calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Final camera extrinsics
  if (_app->get_state()->_options.do_calib_camera_pose) {
    for (int i = 0; i < _app->get_state()->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = _app->get_state()->_calib_IMUtoCAM.at(i);
      Eigen::Matrix4d T_CtoI = Eigen::Matrix4d::Identity();
      T_CtoI.block(0, 0, 3, 3) = quat_2_Rot(calib->quat()).transpose();
      T_CtoI.block(0, 3, 3, 1) = -T_CtoI.block(0, 0, 3, 3) * calib->pos();
      printf(REDPURPLE "T_C%dtoI:\n" RESET, i);
      printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(0, 0),
             T_CtoI(0, 1), T_CtoI(0, 2), T_CtoI(0, 3));
      printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(1, 0),
             T_CtoI(1, 1), T_CtoI(1, 2), T_CtoI(1, 3));
      printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f,\n" RESET, T_CtoI(2, 0),
             T_CtoI(2, 1), T_CtoI(2, 2), T_CtoI(2, 3));
      printf(REDPURPLE "%.3f,%.3f,%.3f,%.3f\n\n" RESET, T_CtoI(3, 0),
             T_CtoI(3, 1), T_CtoI(3, 2), T_CtoI(3, 3));
    }
  }

  // Print the total time
  rT2 = boost::posix_time::microsec_clock::local_time();
  printf(REDPURPLE "TIME: %.3f seconds\n\n" RESET,
         (rT2 - rT1).total_microseconds() * 1e-6);
}

void RosVisualizer::publish_state() {
  // Get the current state
  std::shared_ptr<State> state = _app->get_state();

  // YS: !!!
  // We want to publish in the IMU clock frame
  // The timestamp in the state will be the last camera time
  double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
  double timestamp_inI = state->_timestamp + t_ItoC;
  tf2::TimePoint tf2_tp = tf2::timeFromSec(timestamp_inI);

  // Create pose of IMU (note we use the bag time)
  geometry_msgs::msg::PoseWithCovarianceStamped poseIinM;
  poseIinM.header.stamp = tf2_ros::toMsg(tf2_tp);
  poseIinM.header.frame_id = frame_ids_["odom"];
  poseIinM.pose.pose.orientation.x = state->_imu->quat()(0);
  poseIinM.pose.pose.orientation.y = state->_imu->quat()(1);
  poseIinM.pose.pose.orientation.z = state->_imu->quat()(2);
  poseIinM.pose.pose.orientation.w = state->_imu->quat()(3);
  poseIinM.pose.pose.position.x = state->_imu->pos()(0);
  poseIinM.pose.pose.position.y = state->_imu->pos()(1);
  poseIinM.pose.pose.position.z = state->_imu->pos()(2);

  // Finally set the covariance in the message (in the order position then
  // orientation as per ros convention)
  std::vector<std::shared_ptr<Type>> statevars;
  statevars.push_back(state->_imu->pose()->p());
  statevars.push_back(state->_imu->pose()->q());
  Eigen::Matrix<double, 6, 6> covariance_posori =
      StateHelper::get_marginal_covariance(_app->get_state(), statevars);
  for (int r = 0; r < 6; r++) {
    for (int c = 0; c < 6; c++) {
      poseIinM.pose.covariance[6 * r + c] = covariance_posori(r, c);
    }
  }
  pub_poseimu->publish(poseIinM);

  //=========================================================
  //=========================================================

  // Append to our pose vector
  geometry_msgs::msg::PoseStamped posetemp;
  posetemp.header = poseIinM.header;
  posetemp.pose = poseIinM.pose.pose;
  poses_imu.push_back(posetemp);

  // Create our path (imu)
  // NOTE: We downsample the number of poses as needed to prevent rviz crashes
  // NOTE: https://github.com/ros-visualization/rviz/issues/1107
  nav_msgs::msg::Path arrIMU;
  arrIMU.header.stamp = tf2_ros::toMsg(tf2_tp);
  arrIMU.header.frame_id = frame_ids_["odom"];
  for (size_t i = 0; i < poses_imu.size();
       i += std::floor(poses_imu.size() / 16384.0) + 1) {
    arrIMU.poses.push_back(poses_imu.at(i));
  }
  pub_pathimu->publish(arrIMU);
    if (is_initial_)
        is_initial_ = false;

  // Publish our transform on TF
  // NOTE: since we use JPL we have an implicit conversion to Hamilton when we publish
  // NOTE: a rotation from GtoI in JPL has the same xyzw as a ItoG Hamilton rotation
 // tf::StampedTransform trans;
 // trans.stamp_ = tf2_ros::toMsg(tf2_tp);
 // trans.frame_id_ = "global";
 // trans.child_frame_id_ = "imu";
 // tf::Quaternion quat(state->_imu->quat()(0),state->_imu->quat()(1),state->_imu->quat()(2),state->_imu->quat()(3));
 // trans.setRotation(quat);
 // tf::Vector3 orig(state->_imu->pos()(0),state->_imu->pos()(1),state->_imu->pos()(2));
 // trans.setOrigin(orig);
 // if(publish_global2imu_tf) {
 //     mTfBr->sendTransform(trans);
 // }

#if 0
  // YS: 只用于显示，应该还好
  if (publish_tf_) {
    const auto &base_link = frame_ids_["base_link"];
    const auto &imu_link = frame_ids_["imu"];
    if (tf_buffer_->canTransform(base_link, imu_link, tf2_tp,
                                 tf2::durationFromSec(0.2))) {
      auto Tbi = tf2::transformToEigen(
          tf_buffer_->lookupTransform(base_link, imu_link, tf2_tp));
      auto q = state->_imu->quat();
      auto p = state->_imu->pos();
      Eigen::Affine3d Twi =
          Eigen::Translation3d(p) * Eigen::Quaterniond(q(3), q(0), q(1), q(2));
      Eigen::Affine3d Twb = Twi * Tbi.inverse();
      // Publish our transform on TF
      // NOTE: since we use JPL we have an implicit conversion to Hamilton when
      // we publish NOTE: a rotation from GtoI in JPL has the same xyzw as a
      // ItoG Hamilton rotation
      geometry_msgs::msg::TransformStamped trans = tf2::eigenToTransform(Twb);
      trans.header.stamp = tf2_ros::toMsg(tf2_tp);
      trans.header.frame_id = frame_ids_["odom"];
      trans.child_frame_id = frame_ids_["base_link"];
      tf_broadcaster_->sendTransform(trans);
    }
  }
#endif
}

void RosVisualizer::publish_images(double timestamp) {
  // Check if we have subscribers
  if (pub_tracks->get_subscription_count() == 0) return;

  // Get our image of history tracks
  cv::Mat img_history = _app->get_historical_viz_image();

  // Create our message
  std_msgs::msg::Header header;
  header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp));
  header.frame_id = frame_ids_["cam0"];
  sensor_msgs::msg::Image::SharedPtr exl_msg =
      cv_bridge::CvImage(header, "bgr8", img_history).toImageMsg();
  // Publish
  pub_tracks->publish(*exl_msg);
}

#if 0
void RosVisualizer::publish_features(double timestamp) {
  // Check if we have subscribers
  if (pub_points_msckf->get_subscription_count() == 0 &&
      pub_points_slam->get_subscription_count() == 0 &&
      pub_points_aruco->get_subscription_count() == 0 &&
      pub_points_sim->get_subscription_count() == 0)
    return;

  // Get our good features
  std::vector<Eigen::Vector3d> feats_msckf = _app->get_good_features_MSCKF();

  // Declare message and sizes
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame_ids_["odom"];
  cloud.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp));
  cloud.width = 3 * feats_msckf.size();
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false;  // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(3 * feats_msckf.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");

  // Fill our iterators
  for (const auto &pt : feats_msckf) {
    *out_x = pt(0);
    ++out_x;
    *out_y = pt(1);
    ++out_y;
    *out_z = pt(2);
    ++out_z;
  }

  // Publish
  pub_points_msckf->publish(cloud);

  //====================================================================
  //====================================================================

  // Get our good features
  std::vector<Eigen::Vector3d> feats_slam = _app->get_features_SLAM();

  // Declare message and sizes
  sensor_msgs::msg::PointCloud2 cloud_SLAM;
  cloud_SLAM.header.frame_id = frame_ids_["odom"];
  cloud_SLAM.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp));
  cloud_SLAM.width = 3 * feats_slam.size();
  cloud_SLAM.height = 1;
  cloud_SLAM.is_bigendian = false;
  cloud_SLAM.is_dense = false;  // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier_SLAM(cloud_SLAM);
  modifier_SLAM.setPointCloud2FieldsByString(1, "xyz");
  modifier_SLAM.resize(3 * feats_slam.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x_SLAM(cloud_SLAM, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y_SLAM(cloud_SLAM, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z_SLAM(cloud_SLAM, "z");

  // Fill our iterators
  for (const auto &pt : feats_slam) {
    *out_x_SLAM = pt(0);
    ++out_x_SLAM;
    *out_y_SLAM = pt(1);
    ++out_y_SLAM;
    *out_z_SLAM = pt(2);
    ++out_z_SLAM;
  }

  // Publish
  pub_points_slam->publish(cloud_SLAM);

  //====================================================================
  //====================================================================

  // Get our good features
  std::vector<Eigen::Vector3d> feats_aruco = _app->get_features_ARUCO();

  // Declare message and sizes
  sensor_msgs::msg::PointCloud2 cloud_ARUCO;
  cloud_ARUCO.header.frame_id = frame_ids_["odom"];
  cloud_ARUCO.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp));
  cloud_ARUCO.width = 3 * feats_aruco.size();
  cloud_ARUCO.height = 1;
  cloud_ARUCO.is_bigendian = false;
  cloud_ARUCO.is_dense = false;  // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier_ARUCO(cloud_ARUCO);
  modifier_ARUCO.setPointCloud2FieldsByString(1, "xyz");
  modifier_ARUCO.resize(3 * feats_aruco.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x_ARUCO(cloud_ARUCO, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y_ARUCO(cloud_ARUCO, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z_ARUCO(cloud_ARUCO, "z");

  // Fill our iterators
  for (const auto &pt : feats_aruco) {
    *out_x_ARUCO = pt(0);
    ++out_x_ARUCO;
    *out_y_ARUCO = pt(1);
    ++out_y_ARUCO;
    *out_z_ARUCO = pt(2);
    ++out_z_ARUCO;
  }

  // Publish
  pub_points_aruco->publish(cloud_ARUCO);

  //====================================================================
  //====================================================================

  // Skip the rest of we are not doing simulation
  if (_sim == nullptr) return;

  // Get our good features
  std::unordered_map<size_t, Eigen::Vector3d> feats_sim = _sim->get_map();

  // Declare message and sizes
  sensor_msgs::msg::PointCloud2 cloud_SIM;
  cloud_SIM.header.frame_id = frame_ids_["odom"];
  cloud_SIM.header.stamp = tf2_ros::toMsg(tf2::timeFromSec(timestamp));
  cloud_SIM.width = 3 * feats_sim.size();
  cloud_SIM.height = 1;
  cloud_SIM.is_bigendian = false;
  cloud_SIM.is_dense = false;  // there may be invalid points

  // Setup pointcloud fields
  sensor_msgs::PointCloud2Modifier modifier_SIM(cloud_SIM);
  modifier_SIM.setPointCloud2FieldsByString(1, "xyz");
  modifier_SIM.resize(3 * feats_sim.size());

  // Iterators
  sensor_msgs::PointCloud2Iterator<float> out_x_SIM(cloud_SIM, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y_SIM(cloud_SIM, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z_SIM(cloud_SIM, "z");

  // Fill our iterators
  for (const auto &pt : feats_sim) {
    *out_x_SIM = pt.second(0);
    ++out_x_SIM;
    *out_y_SIM = pt.second(1);
    ++out_y_SIM;
    *out_z_SIM = pt.second(2);
    ++out_z_SIM;
  }

  // Publish
  pub_points_sim->publish(cloud_SIM);
}
#endif

void RosVisualizer::listen_tf(athena_utils::LifecycleNode *node) {
    std::unique_ptr<tf2_ros::Buffer> tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    std::unique_ptr<tf2_ros::TransformListener> tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
    std::ofstream tf_stream;
    std::string tf_path = "output_tf.txt";
    //tf_stream.open(tf_path.c_str(), ios::app);
    while (rclcpp::ok() && !is_close){
        // Get the current state
        // We want to publish in the IMU clock frame
        // The timestamp in the state will be the last camera time
        std::shared_ptr<State> state = _app->get_state();
        double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
        double timestamp_inI = state->_timestamp + t_ItoC;
        tf2::TimePoint tf2_tp = tf2::timeFromSec(timestamp_inI);

        geometry_msgs::msg::TransformStamped transformStamped;
        if(tf_buffer->canTransform("odom", "base_footprint", tf2_tp, tf2::durationFromSec(0.2))){

            Eigen::Isometry3d eigen_transform = tf2::transformToEigen(tf_buffer->lookupTransform(
				             "odom", "base_footprint", tf2_tp, tf2::durationFromSec(0.2)));
            //tf::StampedTransform stamped_transform;
            //tf::transformStampedMsgToTF(transformStamped, stamped_transform);
            //Eigen::Isometry3d eigen_transform;
            //tf::transformTFToEigen(stamped_transform, eigen_transform);
            T_foot_curr_ = eigen_transform.matrix();
	          //std::cout << "T_foot_curr_: " << std::endl << T_foot_curr_ << std::endl;

            track_lost_edge_ = _app->get_track_lost_egde();
            double time_now = timestamp_inI;//ros::Time::now().toSec();
            if (track_lost_edge_) {  // vio->foot
                if (fusion_odom_state_ == 0) {
                    // fusion_odom_state_ = 1;
                    std::cout << "vio->foot" << std::endl;
                    track_lost_edge_ = false;
                    T_base_lost_ = T_fusion_curr_;
                    T_foot_lost_ = T_foot_curr_;
                    lost_time_ = timestamp_inI;//ros::Time::now().toSec();
                }
            } else {
                if (lost_time_ == 0.0) {  // vio
                    fusion_odom_state_ = 0;
                    std::cout << "vio" << std::endl;
                    T_base_curr_ = get_T_base_curr();
                    Eigen::Matrix4d inv_T_base_lost = matrix_inverse_by_cv(T_base_lost_);
                    delta_T_ = inv_T_base_lost * T_base_curr_;
                    T_fusion_curr_ = T_foot_lost_ * delta_T_;
                } else {
                    track_bad_ = _app->get_track_bad();
                    track_bad_ = true; // tmp change, close foot-vio 
                    last_track_bad_time_ = (track_bad_) ? timestamp_inI : last_track_bad_time_;//ros::Time::now().toSec();
                    double delta_lose_time = time_now - lost_time_;
                    double delta_bad_time = time_now - last_track_bad_time_;
                    if (delta_lose_time <= wait_vio_reset_time_ || delta_bad_time < wait_vio_reset_time_) {  // foot
                        fusion_odom_state_ = 2;
                        std::cout << "foot" << std::endl;
                        Eigen::Matrix4d inv_T_foot_lost = matrix_inverse_by_cv(T_foot_lost_);
                        delta_T_ = inv_T_foot_lost * T_foot_curr_;
                        T_fusion_curr_ = T_base_lost_ * delta_T_;
                    } else {  // foot->vio
                        fusion_odom_state_ = 3;
                        std::cout << "foot->vio" << std::endl;
                        T_base_curr_ = get_T_base_curr();
                        T_base_lost_ = T_base_curr_;
                        T_foot_lost_ = T_fusion_curr_;
                        lost_time_ = 0.0;
                    }
                }
            }
	          //std::cout << "T_fusion_curr_: " << std::endl << T_fusion_curr_ << std::endl;
            Eigen::Matrix3d rot = T_fusion_curr_.block(0,0,3,3);
            Eigen::Quaterniond Qua(rot);
            //tf_stream << std::setprecision(7) << Qua.x() << " " << Qua.y() << " " << Qua.z() << " " << Qua.w() << " " <<
                      //T_fusion_curr_(0,3) << " " << T_fusion_curr_(1,3) << " " << T_fusion_curr_(2,3) << std::endl;
            broadcast_T("odom", "base_fusion", T_fusion_curr_, timestamp_inI);
#if 0
            if (!is_initial_) {
		            geometry_msgs::msg::PoseStamped pose_fusion_stamped = eigen_to_posestamped(T_fusion_curr_);
                pose_fusion_stamped.header.stamp = tf2_ros::toMsg(tf2_tp);//ros::Time::now();
                //pose_fusion_stamped.header.seq = pose_fusion_seq_num_;
                pose_fusion_stamped.header.frame_id = "odom";

                poses_fusion_.push_back(pose_fusion_stamped);

                nav_msgs::msg::Path path_fusion;
                path_fusion.header.stamp = tf2_ros::toMsg(tf2_tp);//ros::Time::now();
                //path_fusion.header.seq = pose_fusion_seq_num_;
                path_fusion.header.frame_id = "odom";
                for (size_t i = 0; i < poses_fusion_.size(); 
                    i += std::floor(poses_fusion_.size() / 16384.0) + 1) {
                    path_fusion.poses.push_back(poses_fusion_.at(i));
                }
                // std::cout << "path_fusion size: " << path_fusion.poses.size() << std::endl;
                pub_path_fusion_->publish(path_fusion);
            
                pose_fusion_seq_num_++;
            

                geometry_msgs::msg::PoseStamped pose_foot_stamped = eigen_to_posestamped(T_foot_curr_);
                pose_foot_stamped.header.stamp = tf2_ros::toMsg(tf2_tp);//ros::Time::now();
                //pose_foot_stamped.header.seq = pose_foot_seq_num_;
                pose_foot_stamped.header.frame_id = "odom";
                poses_foot_.push_back(pose_foot_stamped);

		            nav_msgs::msg::Path path_foot;
                path_foot.header.stamp = tf2_ros::toMsg(tf2_tp);//ros::Time::now();
                //path_foot.header.seq = pose_foot_seq_num_;
                path_foot.header.frame_id = "odom";
                for (size_t i = 0; i < poses_foot_.size(); 
                    i += std::floor(poses_foot_.size() / 16384.0) + 1) {
                    path_foot.poses.push_back(poses_foot_.at(i));
                }
                pub_path_foot_->publish(path_foot);
                pose_foot_seq_num_++;
            }
#endif
            
        }
	      else {
	        std::cout << "sleep once for no transform listen" << std::endl;
        }
	      usleep(5e4);
    }
    //tf_stream.close();
}

void RosVisualizer::broadcast_T(
    const std::string& frame_id, const std::string& child_frame_id, const Eigen::Matrix4d& trans, const double timestamp) {
    Eigen::Matrix4d trans1= trans;
    Eigen::Matrix3d rotation = trans1.block(0, 0, 3, 3);
    Eigen::Vector3d translation = trans1.block(0, 3, 3, 1);
    Eigen::AngleAxisd rot_vec(rotation);
    Eigen::Isometry3d trans_iso = Eigen::Isometry3d::Identity();
    trans_iso.rotate(rot_vec);
    trans_iso.pretranslate(translation);
    geometry_msgs::msg::TransformStamped transformStamped = tf2::eigenToTransform(trans_iso);
    tf2::TimePoint tf2_tp = tf2::timeFromSec(timestamp);
    transformStamped.header.stamp = tf2_ros::toMsg(tf2_tp);// ros::Time::now();
    transformStamped.header.frame_id = frame_id;
    transformStamped.child_frame_id = child_frame_id;
    slam_broadcaster_->sendTransform(transformStamped);
}

Eigen::Matrix4d RosVisualizer::get_T_imu_curr() {
    std::shared_ptr<State> state = _app->get_state();
    Eigen::Quaterniond quat(state->_imu->quat()(3), state->_imu->quat()(0), state->_imu->quat()(1), state->_imu->quat()(2));
    Eigen::Vector3d orig(state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));
    //std::cout << "quat: " << quat.w() << " "  << quat.x() << " " << quat.y() << " " << quat.z() << std::endl; 
    //std::cout << "trans: " << orig[0] << " "  <<  orig[1] << " " << orig[2] << std::endl; 
    
    //tf::Quaternion quat(state->_imu->quat()(0), state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3));
    //tf::Vector3 orig(state->_imu->pos()(0), state->_imu->pos()(1), state->_imu->pos()(2));
    //tf::StampedTransform trans;
    //trans.setRotation(quat);
    //trans.setOrigin(orig);

    Eigen::Isometry3d iso_transform = Eigen::Isometry3d::Identity();
    iso_transform.rotate(Eigen::AngleAxisd(quat));
    iso_transform.pretranslate(orig);
    //tf::transformTFToEigen(trans, iso_transform);
    Eigen::Matrix4d T_odom_imu = iso_transform.matrix();
    //std::cout << "t_2: " << std::endl << T_odom_imu << std::endl;
    return T_odom_imu;
}

Eigen::Matrix4d RosVisualizer::get_T_base_curr() {
    Eigen::Matrix4d T_imu_curr = get_T_imu_curr();
    Eigen::Matrix4d T_cam_curr = T_imu_curr * T_imu_cam_;
    Eigen::Matrix4d T_base_curr = T_chassis_global_ * T_cam_curr * T_cam_base_;
    //std::cout << "T_base_curr: " << std::endl << T_base_curr << std::endl;
    return T_base_curr;
}

Eigen::Matrix4d RosVisualizer::matrix_inverse_by_cv(const Eigen::Matrix4d& T_in) {
    cv::Mat cv_T_in = (cv::Mat_<double>(4, 4) << 
        T_in(0,0), T_in(0,1), T_in(0,2), T_in(0,3), 
        T_in(1,0), T_in(1,1), T_in(1,2), T_in(1,3), 
        T_in(2,0), T_in(2,1), T_in(2,2), T_in(2,3), 
        T_in(3,0), T_in(3,1), T_in(3,2), T_in(3,3));
    cv::Mat cv_inv_T_in = cv_T_in.inv();
    Eigen::Matrix4d inv_T_in;
    inv_T_in <<     
        cv_inv_T_in.at<double>(0, 0), cv_inv_T_in.at<double>(0, 1), 
            cv_inv_T_in.at<double>(0, 2), cv_inv_T_in.at<double>(0, 3), 
        cv_inv_T_in.at<double>(1, 0), cv_inv_T_in.at<double>(1, 1), 
            cv_inv_T_in.at<double>(1, 2), cv_inv_T_in.at<double>(1, 3), 
        cv_inv_T_in.at<double>(2, 0), cv_inv_T_in.at<double>(2, 1), 
            cv_inv_T_in.at<double>(2, 2), cv_inv_T_in.at<double>(2, 3), 
        cv_inv_T_in.at<double>(3, 0), cv_inv_T_in.at<double>(3, 1), 
            cv_inv_T_in.at<double>(3, 2), cv_inv_T_in.at<double>(3, 3);
    return inv_T_in;
}

geometry_msgs::msg::PoseWithCovarianceStamped RosVisualizer::eigen_to_posewithcovstamped(
    const Eigen::Matrix4d& trans) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose1;
    geometry_msgs::msg::PoseStamped pose2 = eigen_to_posestamped(trans);
    pose1.pose.pose = pose2.pose;
    
    for(int r = 0; r < 6; r++) {
        for(int c = 0; c < 6; c++) {
            pose1.pose.covariance[6*r+c] = 0.0;
        }
    }
    return pose1;
}

geometry_msgs::msg::PoseStamped RosVisualizer::eigen_to_posestamped(
    const Eigen::Matrix4d& trans) {
    Eigen::Matrix3d Rin = trans.block(0, 0, 3, 3);
    Eigen::Quaterniond qin(Rin);
    Eigen::Vector3d tin = trans.block(0, 3, 3, 1);
    geometry_msgs::msg::PoseStamped pose1;
    pose1.pose.orientation.x = qin.x();
    pose1.pose.orientation.y = qin.y();
    pose1.pose.orientation.z = qin.z();
    pose1.pose.orientation.w = qin.w();
    pose1.pose.position.x = tin(0);
    pose1.pose.position.y = tin(1);
    pose1.pose.position.z = tin(2);
    return pose1;
}

