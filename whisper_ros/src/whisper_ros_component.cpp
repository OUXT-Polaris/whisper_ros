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
  parameters_(whisper_ros_node::ParamListener(get_node_parameters_interface()).get_params()),
  buffer_(parameters_.audio_buffer_length, get_clock(), parameters_.diarize)
{
  if (!checkLanguage()) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Invalida language : " << parameters_.language << " specofied.");
    return;
  }
  struct whisper_context * ctx;
  if (const auto model_path = findModel()) {
    ctx = whisper_init_from_file(model_path.value().c_str());
  } else {
    return;
  }
  if (ctx == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "error: failed to initialize whisper context");
    return;
  }
  const auto prompt_tokens = getPromptTokens(ctx);
}

auto WhisperRosComponent::checkLanguage() const -> bool
{
  if (parameters_.language == "auto" && whisper_lang_id(parameters_.language.c_str()) == -1) {
    return false;
  }
  return true;
}

auto WhisperRosComponent::findModel() const -> std::optional<std::string>
{
  std::string model_path = ament_index_cpp::get_package_share_directory("whisper_cpp_vendor") +
                           "/share/models/ggml-" + parameters_.model_type + ".bin";
  if (std::filesystem::exists(model_path)) {
    return model_path;
  }
  return std::nullopt;
}

auto WhisperRosComponent::getPromptTokens(whisper_context * ctx) const -> std::vector<whisper_token>
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

auto WhisperRosComponent::audioDataCallback(const audio_common_msgs::msg::AudioData::SharedPtr msg)
  -> void
{
  buffer_.append(msg);
}

auto WhisperRosComponent::audioInfoCallback(const audio_common_msgs::msg::AudioInfo::SharedPtr msg)
  -> void
{
  if (msg->sample_rate != WHISPER_SAMPLE_RATE) {
    RCLCPP_ERROR_STREAM(get_logger(), "Sampling rate should be " << WHISPER_SAMPLE_RATE << " Hz");
  }
  if (msg->bitrate != 16) {
    RCLCPP_ERROR_STREAM(get_logger(), "Bitrate should be 16");
  }
}

/**
 * @brief 
 * 
 * @param params 
 * @param tokens
 * @sa https://github.com/ggerganov/whisper.cpp/blob/00ea21668b7db98e0530324c0bc1bff53df6995c/examples/main/main.cpp#L665-L686 
 */
auto WhisperRosComponent::runInference(
  const whisper_ros_node::Params & params, const std::vector<whisper_token> & tokens) const -> void
{
  const auto full_params = getFullParameters(params, tokens);
  buffer_.modulate();
  // whisper_print_user_data user_data = {&params, &pcmf32s};
}

/**
 * @brief get full paramter for whisper algorithm
 * @param parameters paramters for whisper_ros node
 * @return whisper_full_params 
 * @sa https://github.com/ggerganov/whisper.cpp/blob/8738427dd60bda894df1ff3c12317cca2e960016/examples/main/main.cpp#L631-L668
 */
auto WhisperRosComponent::getFullParameters(
  const whisper_ros_node::Params params, const std::vector<whisper_token> & prompt_tokens) const
  -> whisper_full_params
{
  whisper_full_params wparams = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
  wparams.strategy = params.beam_size > 1 ? WHISPER_SAMPLING_BEAM_SEARCH : WHISPER_SAMPLING_GREEDY;
  wparams.print_realtime = false;
  wparams.print_progress = false;
  wparams.print_timestamps = false;
  wparams.print_special = false;
  wparams.translate = params.translate;
  wparams.language = params.language.c_str();
  wparams.n_threads = params.n_threads;
  wparams.n_max_text_ctx = params.max_context >= 0 ? params.max_context : wparams.n_max_text_ctx;
  wparams.offset_ms = params.offset_t_ms;
  wparams.duration_ms = 0;
  wparams.token_timestamps = true;
  wparams.thold_pt = params.word_thold;
  wparams.entropy_thold = params.entropy_thold;
  wparams.logprob_thold = params.logprob_thold;
  wparams.max_len = params.max_len == 0 ? 60 : params.max_len;
  wparams.speed_up = params.speed_up;
  wparams.greedy.best_of = params.best_of;
  wparams.beam_search.beam_size = params.beam_size;
  wparams.temperature_inc = -1;
  wparams.prompt_tokens = prompt_tokens.empty() ? nullptr : prompt_tokens.data();
  wparams.prompt_n_tokens = prompt_tokens.empty() ? 0 : prompt_tokens.size();
  // whisper_print_user_data user_data = {&params, &pcmf32s};
  return wparams;
}
}  // namespace whisper_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_ros::WhisperRosComponent)
