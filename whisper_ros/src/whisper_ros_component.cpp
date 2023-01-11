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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <whisper_ros/whisper_ros_component.hpp>

namespace whisper_ros
{
WhisperRosComponent::WhisperRosComponent(const rclcpp::NodeOptions & options)
: Node("whisper_ros_node", options),
  parameters_(whisper_ros_node::ParamListener(get_node_parameters_interface()).get_params())
{
  if (!checkLanguage()) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Invalida language : " << parameters_.language << " specofied.");
    return;
  }
}

bool WhisperRosComponent::checkLanguage() const
{
  if (parameters_.language == "auto" && whisper_lang_id(parameters_.language.c_str()) == -1) {
    return false;
  }
  return true;
}

std::optional<std::string> WhisperRosComponent::findModel() const
{
  std::string model_path =
    ament_index_cpp::get_package_share_directory("whisper_cpp_vendor") + "/share/models";
  return model_path;
}
}  // namespace whisper_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_ros::WhisperRosComponent)
