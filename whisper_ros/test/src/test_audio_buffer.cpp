
// Copyright (c) 2013 OUXT Polaris
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

/**
 * @file test_quaternion_operation.cpp
 * @author Masaya Kataoka ms.kataoka@gmail.com
 * @brief test code for Quaternion Operation
 * @version 0.1
 * @date 2019-04-21
 *
 * @copyright Copyright (c) 2019
 *
 */

// headers in Google Test
#include <gtest/gtest.h>

#include <memory>
#include <whisper_ros/audio_buffer.hpp>

TEST(TestSuite, append)
{
  whisper_ros::AudioBuffer buffer(100, std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME));
  auto data = std::make_shared<audio_common_msgs::msg::AudioData>();
  data->data = std::vector<uint8_t>(100, 1);
  {
    const auto values = buffer.append(data);
    EXPECT_EQ(static_cast<int>(values.size()), 100);
    for (const auto & value : values) {
      EXPECT_EQ(static_cast<int>(value), 1);
    }
  }
  data->data = std::vector<uint8_t>(100, 2);
  {
    const auto values = buffer.append(data);
    EXPECT_EQ(static_cast<int>(values.size()), 100);
    for (const auto & value : values) {
      EXPECT_EQ(static_cast<int>(value), 2);
    }
  }
  data->data = std::vector<uint8_t>(30, 3);
  {
    const auto values = buffer.append(data);
    EXPECT_EQ(static_cast<int>(values.size()), 100);
    for (size_t i = 0; i < 100; i++) {
      if (i <= 69) {
        EXPECT_EQ(static_cast<int>(values[i]), 2);
      } else {
        EXPECT_EQ(static_cast<int>(values[i]), 3);
      }
    }
  }
  data->data = std::vector<uint8_t>(150, 4);
  {
    const auto values = buffer.append(data);
    EXPECT_EQ(static_cast<int>(values.size()), 100);
    for (const auto & value : values) {
      EXPECT_EQ(static_cast<int>(value), 4);
    }
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
