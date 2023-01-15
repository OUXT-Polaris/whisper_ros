// Copyright (c) 2023 OUXT Polaris
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

#ifndef WHISPER_ROS__WHISPER_ROS_COMPONENT_HPP_
#define WHISPER_ROS__WHISPER_ROS_COMPONENT_HPP_

#include <whisper.h>

#include <audio_common_msgs/msg/audio_data.hpp>
#include <audio_common_msgs/msg/audio_info.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <whisper_parameters.hpp>
#include <whisper_ros/audio_buffer.hpp>
#include <whisper_ros/visibility_control.hpp>

namespace whisper_ros
{
class WhisperRosComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit WhisperRosComponent(const rclcpp::NodeOptions &);

private:
  auto findModel() const -> std::optional<std::string>;
  auto checkLanguage() const -> bool;
  auto getPromptTokens(whisper_context *) const -> std::vector<whisper_token>;
  auto audioDataCallback(const audio_common_msgs::msg::AudioData::SharedPtr) -> void;
  auto audioInfoCallback(const audio_common_msgs::msg::AudioInfo::SharedPtr) -> void;
  auto runInference(const whisper_ros_node::Params &, const std::vector<whisper_token> &) const
    -> void;
  auto getFullParameters(const whisper_ros_node::Params, const std::vector<whisper_token> &) const
    -> whisper_full_params;
  const whisper_ros_node::Params parameters_;
  AudioBuffer buffer_;
};

struct whisper_print_user_data
{
  const whisper_ros_node::Params * params;
  const std::vector<std::vector<float>> * pcmf32s;
};
}  // namespace whisper_ros

#endif  // WHISPER_ROS__WHISPER_ROS_COMPONENT_HPP_
