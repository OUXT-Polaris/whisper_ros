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

#ifndef WHISPER_ROS__AUDIO_BUFFER_HPP_
#define WHISPER_ROS__AUDIO_BUFFER_HPP_

#include <audio_common_msgs/msg/audio_data.hpp>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace whisper_ros
{
class AudioBuffer
{
public:
  explicit AudioBuffer(size_t buffer_length, const rclcpp::Clock::SharedPtr clock);
  const size_t buffer_length;
  auto append(const audio_common_msgs::msg::AudioData::SharedPtr msg) -> std::vector<uint8_t>;
  auto getTimeStamp() const -> rclcpp::Time;

private:
  const rclcpp::Clock::SharedPtr clock_;
  std::vector<uint8_t> raw_data_;
  rclcpp::Time timestamp_;
};
}  // namespace whisper_ros

#endif  // WHISPER_ROS__AUDIO_BUFFER_HPP_
