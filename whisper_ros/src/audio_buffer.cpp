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
#include <dr_wav.h>

#include <rclcpp/rclcpp.hpp>

namespace whisper_ros
{
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

auto AudioBuffer::modulate() const -> std::optional<ModulatedData>
{
  drwav wav;
  if (drwav_init_memory(&wav, raw_data_.data(), raw_data_.size(), nullptr) == false) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("audio_buffer"), "Something wrong happens while reading wav data.");
    return std::nullopt;
  }
  if (wav.channels != 1 && wav.channels != 2) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("audio_buffer"), "Wav data must be mono or stereo.");
    return std::nullopt;
  }
  if (diarize && wav.channels != 2) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("audio_buffer"),
      "If you want to use stereo diarize, wav data must be mono or stereo.");
    return std::nullopt;
  }
  const uint64_t n = raw_data_.empty() ? wav.totalPCMFrameCount
                                       : raw_data_.size() / (wav.channels * wav.bitsPerSample / 8);
  std::vector<int16_t> pcm16;
  pcm16.resize(n * wav.channels);
  drwav_read_pcm_frames_s16(&wav, n, pcm16.data());
  drwav_uninit(&wav);

  // convert to mono, float
  std::vector<float> pcmf32;                // mono-channel F32 PCM
  std::vector<std::vector<float>> pcmf32s;  // stereo-channel F32 PCM
  pcmf32.resize(n);
  if (wav.channels == 1) {
    for (uint64_t i = 0; i < n; i++) {
      pcmf32[i] = float(pcm16[i]) / 32768.0f;
    }
  } else {
    for (uint64_t i = 0; i < n; i++) {
      pcmf32[i] = float(pcm16[2 * i] + pcm16[2 * i + 1]) / 65536.0f;
    }
  }

  if (diarize) {
    // convert to stereo, float
    pcmf32s.resize(2);

    pcmf32s[0].resize(n);
    pcmf32s[1].resize(n);
    for (uint64_t i = 0; i < n; i++) {
      pcmf32s[0][i] = float(pcm16[2 * i]) / 32768.0f;
      pcmf32s[1][i] = float(pcm16[2 * i + 1]) / 32768.0f;
    }
  }
  return ModulatedData{pcm16, pcmf32, pcmf32s};
}

}  // namespace whisper_ros
