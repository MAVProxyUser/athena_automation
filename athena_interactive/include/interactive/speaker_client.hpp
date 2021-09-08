// Copyright 2021 Xiaomi Inc
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

#ifndef INTERACTIVE__SPEAKER_CLIENT_HPP_
#define INTERACTIVE__SPEAKER_CLIENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>

#include "interaction_msgs/action/audio_play.hpp"

namespace interactive
{
typedef unsigned int uint16;
using AudioPlay = interaction_msgs::action::AudioPlay;
using GoalHandleAudio = rclcpp_action::ClientGoalHandle<AudioPlay>;
class SpeakerClient
{
public:
  static SpeakerClient & getInstance();
  bool sendGoal(uint16 name_id, uint16 user_id);

  SpeakerClient();
  ~SpeakerClient();

  void goalResponseCallback(std::shared_future<GoalHandleAudio::SharedPtr> future);
  void feedbackCallback(
    GoalHandleAudio::SharedPtr,
    const std::shared_ptr<const AudioPlay::Feedback> feedback);
  void resultCallback(const GoalHandleAudio::WrappedResult & result);

  rclcpp_action::Client<AudioPlay>::SharedPtr m_player_client;
};

}  // namespace interactive

#endif  // INTERACTIVE__SPEAKER_CLIENT_HPP_
