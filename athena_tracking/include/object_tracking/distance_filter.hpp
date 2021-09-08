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

#ifndef OBJECT_TRACKING__DISTANCE_FILTER_HPP_
#define OBJECT_TRACKING__DISTANCE_FILTER_HPP_

#include "opencv2/video/tracking.hpp"

namespace object_tracking
{

class DistanceFilter
{
public:
  DistanceFilter();
  ~DistanceFilter();

  void init(const cv::Point3f & pose);
  cv::Point3f predict(const float & delta);
  void correct(const cv::Point3f & pose);

  bool initialized_;
  cv::KalmanFilter filter_;

private:
  void init();
  void set_interval(const float & delta);

  int state_num_;
  int measure_num_;

  cv::Mat state_;
  cv::Mat process_noise_;
  cv::Mat measurement_;
};

}  // namespace object_tracking

#endif  // OBJECT_TRACKING__DISTANCE_FILTER_HPP_
