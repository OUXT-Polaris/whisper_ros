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

#include <whisper_ros/audio_buffer.hpp>
#define DR_WAV_IMPLEMENTATION
#include <rclcpp/rclcpp.hpp>

#include "dr_wav.h"

namespace whisper_ros
{

ModulatedData::ModulatedData(
  const std::vector<int16_t> & pcm16, const std::vector<float> & pcmf32,
  const std::vector<std::vector<float>> & pcmf32s)
: pcm16(pcm16), pcmf32(pcmf32), pcmf32s(pcmf32s)
{
}

AudioBuffer::AudioBuffer(
  const size_t buffer_length, const rclcpp::Clock::SharedPtr clock, const bool diarize)
: buffer_length(buffer_length), diarize(diarize), clock_(clock)
{
  raw_data_.reserve(buffer_length);
}

auto AudioBuffer::append(const audio_common_msgs::msg::AudioData::SharedPtr msg)
  -> std::vector<uint8_t>
{
  timestamp_ = clock_->now();
  if (msg->data.size() >= buffer_length) {
    raw_data_.clear();
    raw_data_.insert(
      raw_data_.end(), msg->data.begin() + static_cast<size_t>(msg->data.size() - buffer_length),
      msg->data.end());
  } else {
    int delete_size = static_cast<int>(raw_data_.size()) + static_cast<int>(msg->data.size()) -
                      static_cast<int>(buffer_length);
    if (delete_size >= 0) {
      raw_data_.erase(raw_data_.begin(), raw_data_.begin() + delete_size);
    }
    raw_data_.insert(raw_data_.end(), msg->data.begin(), msg->data.end());
  }
  return raw_data_;
}

auto AudioBuffer::getTimeStamp() const -> rclcpp::Time { return timestamp_; }

auto AudioBuffer::modulate(size_t num_channels) const -> std::optional<ModulatedData>
{
  const auto pcm16 = toInt16(raw_data_);
  if (!pcm16) {
    return std::nullopt;
  }
  const auto pcmf32 = toFloat32Mono(pcm16.value(), num_channels);
  if (!pcmf32) {
    return std::nullopt;
  }
  if (diarize) {
    const auto pcmf32s = toFloat32Stero(pcm16.value(), num_channels);
    if (!pcmf32s) {
      return std::nullopt;
    }
    return ModulatedData(pcm16.value(), pcmf32.value(), pcmf32s.value());
  } else {
    return ModulatedData(pcm16.value(), pcmf32.value(), std::vector<std::vector<float>>());
  }
}

auto toFloat32Mono(const std::vector<int16_t> & values, size_t num_channels)
  -> std::optional<std::vector<float>>
{
  std::vector<float> ret;
  switch (num_channels) {
    case 1:
      std::transform(values.begin(), values.end(), std::back_inserter(ret), [](const int16_t x) {
        return static_cast<float>(x) / 32768.0f;
      });
      return ret;
    case 2:
      if (values.size() % 2 == 0) {
        for (size_t i = 0; i < values.size() / 2; i++) {
          ret[i] = static_cast<float>(values[2 * i] + values[2 * i + 1]) / 65536.0f;
        }
        return ret;
      } else {
        return std::nullopt;
      }
    default:
      return std::nullopt;
  }
}

auto toFloat32Stero(const std::vector<int16_t> & values, size_t num_channels)
  -> std::optional<std::vector<std::vector<float>>>
{
  if (num_channels != 2) {
    return std::nullopt;
  }
  if (values.size() % 2 != 0) {
    return std::nullopt;
  }
  const size_t n = values.size() / 2;
  std::vector<std::vector<float>> ret;
  ret.resize(2);
  ret[0].resize(n);
  ret[1].resize(n);
  for (uint64_t i = 0; i < n; i++) {
    ret[0][i] = float(values[2 * i]) / 32768.0f;
    ret[1][i] = float(values[2 * i + 1]) / 32768.0f;
  }
  return ret;
}

auto toInt16(const std::vector<uint8_t> & values) -> std::optional<std::vector<int16_t>>
{
  if (values.size() % 2 == 0) {
    std::vector<int16_t> ret;
    size_t n = static_cast<size_t>(values.size() / 2);
    ret.resize(n);
    memcpy(ret.data(), values.data(), sizeof(int16_t) * n);
    return ret;
  } else {
    return std::nullopt;
  }
}

}  // namespace whisper_ros
