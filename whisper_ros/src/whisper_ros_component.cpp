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
#include <filesystem>
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
  struct whisper_context * ctx;
  if (const auto model_path = findModel()) {
    ctx = whisper_init(model_path.value().c_str());
  } else {
    return;
  }
  if (ctx == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "error: failed to initialize whisper context");
    return;
  }
  const auto prompt_tokens = getPromptTokens(ctx);
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
  std::string model_path = ament_index_cpp::get_package_share_directory("whisper_cpp_vendor") +
                           "/share/models/ggml-" + parameters_.model_type + ".bin";
  if (std::filesystem::exists(model_path)) {
    return model_path;
  }
  return std::nullopt;
}

std::vector<whisper_token> WhisperRosComponent::getPromptTokens(whisper_context * ctx) const
{
  std::vector<whisper_token> prompt_tokens;
  if (!parameters_.prompt.empty()) {
    prompt_tokens.resize(1024);
    prompt_tokens.resize(whisper_tokenize(
      ctx, parameters_.prompt.c_str(), prompt_tokens.data(), prompt_tokens.size()));
    RCLCPP_INFO(get_logger(), "initial prompt: '%s'\n", parameters_.prompt.c_str());
    RCLCPP_INFO(get_logger(), "initial tokens: [ ");
    for (int i = 0; i < (int)prompt_tokens.size(); ++i) {
      RCLCPP_INFO(get_logger(), "%d ", prompt_tokens[i]);
    }
  }
  return prompt_tokens;
}
}  // namespace whisper_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_ros::WhisperRosComponent)
