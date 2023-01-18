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
  if (const auto model_path = findModel()) {
    ctx_ = whisper_init_from_file(model_path.value().c_str());
  } else {
    return;
  }
  if (ctx_ == nullptr) {
    RCLCPP_ERROR_STREAM(get_logger(), "error: failed to initialize whisper context");
    return;
  }
  // rclcpp::QoS durable_qos{1};
  // durable_qos.transient_local();
  print_segment_callback_pointer_ = &WhisperRosComponent::whisper_print_segment_callback;
}

WhisperRosComponent::~WhisperRosComponent() { whisper_free(ctx_); }

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

auto WhisperRosComponent::getPromptTokens() const -> std::vector<whisper_token>
{
  std::vector<whisper_token> prompt_tokens;
  if (!parameters_.prompt.empty()) {
    prompt_tokens.resize(1024);
    prompt_tokens.resize(whisper_tokenize(
      ctx_, parameters_.prompt.c_str(), prompt_tokens.data(), prompt_tokens.size()));
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
  runInference(parameters_, getPromptTokens());
  // runInference(parameters_, );
}

auto WhisperRosComponent::audioInfoCallback(const audio_common_msgs::msg::AudioInfo::SharedPtr msg)
  -> void
{
  if (msg->sample_rate != WHISPER_SAMPLE_RATE) {
    RCLCPP_ERROR_STREAM(get_logger(), "Sampling rate should be " << WHISPER_SAMPLE_RATE << " Hz");
  }
  if (msg->sample_format != "S16LE") {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Sample format should be S16LE, current value is " << msg->sample_format);
  }
  if (msg->coding_format == "wave") {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Coding format should be wave, current format is " << msg->coding_format);
  }
  if (msg->channels == 1 or msg->channels == 2) {
    audio_info_ = msg;
  } else {
    RCLCPP_ERROR_STREAM(
      get_logger(), "Channels should be 1 or 2, current value is " << msg->channels);
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
  const whisper_ros_node::Params & params, const std::vector<whisper_token> & tokens) -> void
{
  if (!audio_info_) {
    RCLCPP_WARN_STREAM(get_logger(), "Audio info topic does not subscribed yet.");
  }
  const auto data = buffer_.modulate(audio_info_.value()->channels);
  if (!data) {
    RCLCPP_WARN_STREAM(get_logger(), "Failed to modulate audio data.");
  }
  whisper_print_user_data user_data = {&params, &data.value().pcmf32s};
  auto full_params = getFullParameters(params, tokens);
  full_params.new_segment_callback =
    (whisper_new_segment_callback)(this->print_segment_callback_pointer_);
  full_params.new_segment_callback_user_data = &user_data;
  {
    static bool is_aborted = false;  // NOTE: this should be atomic to avoid data race

    full_params.encoder_begin_callback = [](struct whisper_context * /*ctx*/, void * user_data) {
      bool is_aborted = *(bool *)user_data;
      return !is_aborted;
    };
    full_params.encoder_begin_callback_user_data = &is_aborted;
  }
  if (
    whisper_full_parallel(
      ctx_, full_params, data.value().pcmf32.data(), data.value().pcmf32.size(),
      params.n_processors) != 0) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to process audio.");
  }
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

auto WhisperRosComponent::whisper_print_segment_callback(
  whisper_context * ctx, int n_new, void * user_data) -> void
{
  const auto & params = *((whisper_print_user_data *)user_data)->params;
  const auto & pcmf32s = *((whisper_print_user_data *)user_data)->pcmf32s;
  const int n_segments = whisper_full_n_segments(ctx);
}
}  // namespace whisper_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_ros::WhisperRosComponent)
