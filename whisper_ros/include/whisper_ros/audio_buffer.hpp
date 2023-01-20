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
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

namespace whisper_ros
{
auto toFloat32Mono(const std::vector<int16_t> & values, size_t num_channels)
  -> std::optional<std::vector<float>>;
auto toFloat32Stero(const std::vector<int16_t> & values, size_t num_channels)
  -> std::optional<std::vector<std::vector<float>>>;
auto toInt16(const std::vector<uint8_t> & values) -> std::optional<std::vector<int16_t>>;

struct ModulatedData
{
  const std::vector<int16_t> pcm16;               // int16 PCM
  const std::vector<float> pcmf32;                // mono-channel F32 PCM
  const std::vector<std::vector<float>> pcmf32s;  // stereo-channel F32 PCM
  explicit ModulatedData(
    const std::vector<int16_t> & pcm16, const std::vector<float> & pcmf32,
    const std::vector<std::vector<float>> & pcmf32s);
};

class AudioBuffer
{
public:
  explicit AudioBuffer(
    const size_t buffer_length, const rclcpp::Clock::SharedPtr clock, bool diarize);
  const size_t buffer_length;
  const bool diarize;
  auto append(const audio_common_msgs::msg::AudioData::SharedPtr msg) -> std::vector<uint8_t>;
  auto getTimeStamp() const -> rclcpp::Time;
  auto modulate(size_t num_channels) -> std::optional<ModulatedData>;

private:
  const rclcpp::Clock::SharedPtr clock_;
  std::vector<uint8_t> raw_data_;
  rclcpp::Time timestamp_;
  std::mutex mtx_;
};
}  // namespace whisper_ros

#endif  // WHISPER_ROS__AUDIO_BUFFER_HPP_
