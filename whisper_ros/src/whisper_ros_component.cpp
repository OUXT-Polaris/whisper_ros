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
#define DR_WAV_IMPLEMENTATION
#include <dr_wav.h>

namespace whisper_ros
{
WhisperRosComponent::WhisperRosComponent(const rclcpp::NodeOptions & options)
: Node("whisper_ros_node", options),
  parameters_(whisper_ros_node::ParamListener(get_node_parameters_interface()).get_params()),
  buffer_(parameters_.audio_buffer_length, get_clock())
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

void WhisperRosComponent::audioDataCallback(const audio_common_msgs::msg::AudioData::SharedPtr msg)
{
  const auto wav_data = buffer_.append(msg);
  drwav wav;
  if (drwav_init_memory(&wav, wav_data.data(), wav_data.size(), nullptr) == false) {
    RCLCPP_ERROR_STREAM(get_logger(), "Something wrong happens while reading wav data.");
    return;
  }
  if (wav.channels != 1 && wav.channels != 2) {
    RCLCPP_ERROR_STREAM(get_logger(), "Wav data must be mono or stereo.");
    return;
  }
  if (parameters_.diarize && wav.channels != 2) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "If you want to use stereo diarize, wav data must be mono or stereo.");
    return;
  }
  const uint64_t n = wav_data.empty() ? wav.totalPCMFrameCount
                                      : wav_data.size() / (wav.channels * wav.bitsPerSample / 8);
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

  if (parameters_.diarize) {
    // convert to stereo, float
    pcmf32s.resize(2);

    pcmf32s[0].resize(n);
    pcmf32s[1].resize(n);
    for (uint64_t i = 0; i < n; i++) {
      pcmf32s[0][i] = float(pcm16[2 * i]) / 32768.0f;
      pcmf32s[1][i] = float(pcm16[2 * i + 1]) / 32768.0f;
    }
  }
}

void WhisperRosComponent::audioInfoCallback(const audio_common_msgs::msg::AudioInfo::SharedPtr msg)
{
  if (msg->sample_rate != WHISPER_SAMPLE_RATE) {
    RCLCPP_ERROR_STREAM(get_logger(), "Sampling rate should be " << WHISPER_SAMPLE_RATE << " Hz");
  }
  if (msg->bitrate != 16) {
    RCLCPP_ERROR_STREAM(get_logger(), "Bitrate should be 16");
  }
}

/**
 * @brief get full paramter for whisper algorithm
 * @param parameters paramters for whisper_ros node
 * @return whisper_full_params 
 * @sa https://github.com/ggerganov/whisper.cpp/blob/8738427dd60bda894df1ff3c12317cca2e960016/examples/main/main.cpp#L631-L668
 */
whisper_full_params WhisperRosComponent::getFullParameters(
  const whisper_ros_node::Params params) const
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
  return wparams;
}
}  // namespace whisper_ros

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(whisper_ros::WhisperRosComponent)
