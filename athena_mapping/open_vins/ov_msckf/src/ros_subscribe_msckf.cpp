/*
 * OpenVINS: An Open Platform for Visual-Inertial Research * Copyright (C) 2019 Patrick Geneva
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

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <memory>

#include "core/RosVisualizer.h"
#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "ov_core/utils/dataset_reader.h"
#include "ov_core/utils/sensor_data.h"
#include "utils/parse_ros.h"

using namespace ov_msckf;
using CallbackReturn_T = athena_utils::CallbackReturn;

class OvMsckf : public athena_utils::LifecycleNode {
 public:
  OvMsckf();
  ~OvMsckf() {
    if (viz) viz->visualize_final();
  }
  VioManagerOptions params;
  rclcpp::Node::SharedPtr image_node;

 private:
  typedef rclcpp::Subscription<sensor_msgs::msg::Image>::ConstSharedPtr
      ImageSub;
  typedef message_filters::Subscriber<sensor_msgs::msg::Image> ImageSyncSub;
  // YS: change from ApproximateTime to ExactTime
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::msg::Image,
                                                    sensor_msgs::msg::Image>
      ImageSyncPolicy;
  typedef message_filters::Synchronizer<ImageSyncPolicy> ImageSync;
  // Callback functions
  void callback_inertial(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void callback_monocular(const sensor_msgs::msg::Image::ConstSharedPtr msg0,
                          int cam_id0);
  void callback_stereo(const sensor_msgs::msg::Image::ConstSharedPtr msg0,
                       const sensor_msgs::msg::Image::ConstSharedPtr msg1,
                       int cam_id0, int cam_id1);
  void main_process();

  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr subimu_;
  std::vector<std::unique_ptr<ImageSyncSub>> sync_subs_cam_;
  std::vector<std::unique_ptr<ImageSync>> sync_cam_;
  std::vector<ImageSub> mono_cams_;

  std::shared_ptr<VioManager> sys;
  std::shared_ptr<RosVisualizer> viz;
  std::shared_ptr<std::thread> main_thread;
  bool is_imu;
  bool is_close;

 protected:
  CallbackReturn_T on_configure(const rclcpp_lifecycle::State&) override;
  CallbackReturn_T on_activate(const rclcpp_lifecycle::State&) override;
  CallbackReturn_T on_deactivate(const rclcpp_lifecycle::State&) override;
  CallbackReturn_T on_cleanup(const rclcpp_lifecycle::State&) override;
  CallbackReturn_T on_shutdown(const rclcpp_lifecycle::State&) override;

};

CallbackReturn_T OvMsckf::on_configure(const rclcpp_lifecycle::State&){
  RCLCPP_INFO(this->get_logger(), "ovmsckf configuring");
  params = parse_ros_nodehandler(this);
  rclcpp::SensorDataQoS sub_qos;
  sub_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  subimu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", sub_qos,
      std::bind(&OvMsckf::callback_inertial, this, std::placeholders::_1));

  std::vector<int> added_cam_ids;
  // Logic for sync stereo subscriber
  // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
  for (const auto &pair : params.stereo_pairs) {
    RCLCPP_INFO(this->get_logger(), "subscrigbing to stereo camera");
    // Read in the topics
    std::string cam_topic0 = "topic_camera" + std::to_string(pair.first);
    std::string cam_topic1 = "topic_camera" + std::to_string(pair.second);
    // Create sync filter (they have unique pointers internally, so we have to
    // use move logic here...)
    auto image_sub0 = std::make_unique<ImageSyncSub>(image_node, cam_topic0, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    auto image_sub1 = std::make_unique<ImageSyncSub>(image_node, cam_topic1, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    auto sync = std::make_unique<ImageSync>(ImageSyncPolicy(10), *image_sub0,
                                            *image_sub1);

    std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr,
                       const sensor_msgs::msg::Image::ConstSharedPtr)>
        cb = std::bind(&OvMsckf::callback_stereo, this, std::placeholders::_1,
                       std::placeholders::_2, pair.first, pair.second);
    sync->registerCallback(
        std::bind(cb, std::placeholders::_1, std::placeholders::_2));
    // Append to our vector of subscribers
    added_cam_ids.push_back(pair.first);
    added_cam_ids.push_back(pair.second);
    sync_cam_.push_back(std::move(sync));
    sync_subs_cam_.push_back(std::move(image_sub0));
    sync_subs_cam_.push_back(std::move(image_sub1));
    RCLCPP_INFO(this->get_logger(), "subscrigbing to camera (stereo): %s",
                cam_topic0.c_str());
    RCLCPP_INFO(this->get_logger(), "subscrigbing to camera (stereo): %s",
                cam_topic1.c_str());
    RCLCPP_INFO(this->get_logger(), "ovmsckf configured");
  }

  // Now we should add any non-stereo callbacks here
  for (int i = 0; i < params.state_options.num_cameras; i++) {
    // Skip if already have been added
    if (std::find(added_cam_ids.begin(), added_cam_ids.end(), i) !=
        added_cam_ids.end())
      continue;
    // read in the topic
    std::string cam_topic = "topic_camera" + std::to_string(i);
    std::function<void(const sensor_msgs::msg::Image::ConstSharedPtr)> cb;
    cb =
        std::bind(&OvMsckf::callback_monocular, this, std::placeholders::_1, i);
    // create subscriber
    auto mono_sub =
        image_node->create_subscription<sensor_msgs::msg::Image>(cam_topic, sub_qos, cb);
    mono_cams_.push_back(mono_sub);
    RCLCPP_INFO(this->get_logger(), "subscribing to cam (mono): %s",
                cam_topic.c_str());
  }
  is_imu = false;
  is_close = true;
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T OvMsckf::on_activate(const rclcpp_lifecycle::State&){
  RCLCPP_INFO(this->get_logger(), "ovmsckf Activating");
  sys = std::make_shared<VioManager>(params);
  viz = std::make_shared<RosVisualizer>(this, sys);
  main_thread = std::make_shared<std::thread>(&OvMsckf::main_process, this);
  viz->visualize_active();
  is_close = false;
  RCLCPP_INFO(this->get_logger(), "ovmsckf Activated");
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T OvMsckf::on_deactivate(const rclcpp_lifecycle::State&){
  RCLCPP_INFO(this->get_logger(), "ovmsckf Deactivating");
  if(!is_close){
    is_close = true;
    viz->visualize_deactive();
    main_thread->join();
    viz->visualize_final();
    viz.reset();
    sys.reset();
  }
  RCLCPP_INFO(this->get_logger(), "ovmsckf Deactivated");
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T OvMsckf::on_cleanup(const rclcpp_lifecycle::State&){
  RCLCPP_INFO(this->get_logger(), "ovmsckf on_cleanup");
  return CallbackReturn_T::SUCCESS;
}

CallbackReturn_T OvMsckf::on_shutdown(const rclcpp_lifecycle::State&){
  RCLCPP_INFO(this->get_logger(), "ovmsckf on_shutdown");
  return CallbackReturn_T::SUCCESS;
}

// Main function
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::executor::ExecutorArgs(), 2);
  auto node = std::make_shared<OvMsckf>();
  executor.add_node(node->get_node_base_interface());
  executor.add_node(node->image_node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}

OvMsckf::OvMsckf(): athena_utils::LifecycleNode("ov_msckf") {
  image_node = std::make_shared<rclcpp::Node>("ov_msckf_img");
}

void OvMsckf::main_process(){
  while(rclcpp::ok() && !is_close){
    if(is_imu){
      // send it to our VIO system
      sys->imu_process();
      is_imu = false;
      viz->visualize();
    }
    else{
      usleep(1000);
    }
  }
}

void OvMsckf::callback_inertial(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg) {
  if(is_close)
    return;
  // convert into correct format
  ov_core::ImuData message;
  message.timestamp = rclcpp::Time(msg->header.stamp).seconds();
  message.wm << msg->angular_velocity.x, msg->angular_velocity.y,
      msg->angular_velocity.z;
  message.am << msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z;
  
  // send it to our VIO system
  sys->feed_measurement_imu(message);
  is_imu = true;
  //viz->visualize();
}


int image_id = 0;
void OvMsckf::callback_stereo(
    const sensor_msgs::msg::Image::ConstSharedPtr msg0,
    const sensor_msgs::msg::Image::ConstSharedPtr msg1, int cam_id0,
    int cam_id1) {
  if(is_close)
    return;
  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr0;
  try {
    cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr1;
  try {
    cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = rclcpp::Time(cv_ptr0->header.stamp).seconds();
  
  //RCLCPP_INFO(this->get_logger(), "image time: %lf", message.timestamp);

  message.sensor_ids.push_back(cam_id0);
  message.sensor_ids.push_back(cam_id1);
  message.images.push_back(cv_ptr0->image.clone());
  message.images.push_back(cv_ptr1->image.clone());
  //std::string image_save_path_1 = "/home/mi/slam_ws/image/left_" + std::to_string(image_id) + ".jpg"; 
  //std::string image_save_path_2 = "/home/mi/slam_ws/image/right_" + std::to_string(image_id) + ".jpg"; 
  //cv::imwrite( image_save_path_1, cv_ptr0->image.clone() );
  //cv::imwrite( image_save_path_2, cv_ptr1->image.clone() );
  image_id++;

  // send it to our VIO system
  sys->feed_measurement_camera(message);
}

void OvMsckf::callback_monocular(
    const sensor_msgs::msg::Image::ConstSharedPtr msg0, int cam_id0) {

  RCLCPP_INFO(this->get_logger(), "monocular come %d", is_close);
  if(is_close)
    return;
  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::RGB8);
  } catch (cv_bridge::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return;
  }

  // Create the measurement
  ov_core::CameraData message;
  message.timestamp = rclcpp::Time(cv_ptr->header.stamp).seconds();
  message.sensor_ids.push_back(cam_id0);
  cv::Mat img_gray = cv_ptr->image.clone();
  message.convert_color2gray(img_gray);
  message.images.push_back(img_gray);
  RCLCPP_INFO(this->get_logger(), "ovmsckf img");
  cv::imshow("show_img",img_gray);
  cv::waitKey(-1);

  // send it to our VIO system
  sys->feed_measurement_camera(message);
}
