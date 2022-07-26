// Copyright (c) 2021 Xiaomi Corporation
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

#ifndef OBJECT_TRACKING__TRANSFORM_HPP_
#define OBJECT_TRACKING__TRANSFORM_HPP_

#include <stdio.h>
#include <string>
#include <iostream>
#include <algorithm>
#include <future>

#include "eigen3/Eigen/Dense"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"

#include "sensor_msgs/msg/camera_info.hpp"

namespace object_tracking
{

class Transform
{
public:
  explicit Transform(
    const std::string & file_path, const cv::Mat & rot, const cv::Mat & trans,
    bool stereo_mode = true);
  ~Transform();

  // Select camera type(realsense camera or stereo camera)
  cv::Mat depth_to_ai(
    const cv::Mat & depth_image, const sensor_msgs::msg::CameraInfo & camera_info,
    const float & rows_scale, const float & cols_scale);
  cv::Mat ai_camera_param_;

private:
  // Transform realsense depth info to AI camera
  cv::Mat realsense_depth_to_ai(
    const cv::Mat & depth_image,
    const sensor_msgs::msg::CameraInfo & camera_info,
    const float & rows_scale, const float & cols_scale);

  // Transform stereo camera info to AI camera
  cv::Mat stereo_depth_to_ai(
    const cv::Mat & depth_image, const sensor_msgs::msg::CameraInfo,
    const float & rows_scale, const float & cols_scale);

  // Transform depth info to AI camera image
  void depth_to_ai(cv::Mat & ai_depth, double depth, int r, int c);

  // Transform stereo info to AI camera image
  void stereo_depth_to_ai(cv::Mat & ai_depth, double depth, int r, int c);

  // Get calibration params
  // camera_AI.yaml : AI camera intrnsic
  // extrinsic_ColorAI.yaml : extrinsic(realsense color camera -> AI camera)
  // extrinsic_Left.yaml : extrinsic(stereo left -> AI camera)
  void load_parameters(const std::string & file_path);

  // Get camera Intrisic(K, D, xi, imageSize)
  // Camera model : omnidirect model
  void load_intrinsic(
    const std::string & file_name, cv::Mat & K, cv::Mat & D, cv::Mat & Xi,
    cv::Size & img_size);

  // Get camera extrinsic(rotation, translation)
  void load_extrinsic(const std::string & file_name, cv::Mat & rot, cv::Mat & trans);

private:
  cv::Mat aiT_;
  cv::Mat aiR_;
  cv::Mat aiD_;
  cv::Mat aiK_;
  cv::Mat aiXi_;

  cv::Mat laiT_;
  cv::Mat laiR_;

  cv::Mat depth2color_r_;
  cv::Mat depth2color_t_;

  cv::Size ai_img_size_;

  double fx_;
  double fy_;
  double cx_;
  double cy_;

  bool stereo_mode_;
};

}  // namespace object_tracking

#endif  // OBJECT_TRACKING__TRANSFORM_HPP_
