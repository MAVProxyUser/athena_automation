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

#define LOG_TAG "NavSpeaker"

#include <memory>

#include "interactive/speaker_client.hpp"

namespace interactive
{

SpeakerClient & SpeakerClient::getInstance()
{
  static SpeakerClient s_instance;

  return s_instance;
}

SpeakerClient::SpeakerClient()
{
  //    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "Create");
  //    m_player_client =  rclcpp_action::create_client<AudioPlay>(this,
  //    "audio_play"); RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "%p end",
  //    m_player_client);
}

SpeakerClient::~SpeakerClient()
{
  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "Destroy");
}

bool SpeakerClient::sendGoal(uint16 name_id, uint16 user_id)
{
  using namespace std::placeholders;

  if (!m_player_client->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(
      rclcpp::get_logger(LOG_TAG),
      "Audio server not available after waiting");
    return false;
  }

  auto goal_msg = interaction_msgs::action::AudioPlay::Goal();
  goal_msg.order.name.id = name_id;
  goal_msg.order.user.id = user_id;

  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "Send audio play request");

  auto send_goal_options = rclcpp_action::Client<
    interaction_msgs::action::AudioPlay>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&SpeakerClient::goalResponseCallback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&SpeakerClient::feedbackCallback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&SpeakerClient::resultCallback, this, _1);
  m_player_client->async_send_goal(goal_msg, send_goal_options);

  return true;
}

void SpeakerClient::goalResponseCallback(
  std::shared_future<GoalHandleAudio::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger(LOG_TAG),
      "Goal accepted by server, waiting for result");
  }
}

void SpeakerClient::feedbackCallback(
  GoalHandleAudio::SharedPtr,
  const std::shared_ptr<const AudioPlay::Feedback> feedback)
{
  RCLCPP_INFO(rclcpp::get_logger(LOG_TAG), "status: %u", feedback->feed.status);
}

void SpeakerClient::resultCallback(
  const GoalHandleAudio::WrappedResult & result)
{
  RCLCPP_INFO(
    rclcpp::get_logger(LOG_TAG), "result: %u",
    result.result->result.error);
}

}  // namespace interactive
