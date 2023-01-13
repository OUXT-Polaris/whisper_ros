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
namespace whisper_ros
{
AudioBuffer::AudioBuffer(size_t buffer_length, const rclcpp::Clock::SharedPtr clock)
: buffer_length(buffer_length), clock_(clock)
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
}  // namespace whisper_ros
