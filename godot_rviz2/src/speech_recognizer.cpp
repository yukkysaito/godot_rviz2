//
//  Copyright 2026 Yukihiro Saito. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "speech_recognizer.hpp"

#include <algorithm>
#include <thread>

#ifdef GODOT_RVIZ2_WHISPER
#include "whisper.h"
#endif

void SpeechRecognizer::_bind_methods()
{
  ClassDB::bind_static_method("SpeechRecognizer", D_METHOD("is_available"), &SpeechRecognizer::is_available);
  ClassDB::bind_method(
    D_METHOD("load_model", "path", "vad_path"), &SpeechRecognizer::load_model, DEFVAL(String()));
  ClassDB::bind_method(D_METHOD("is_loaded"), &SpeechRecognizer::is_loaded);
  ClassDB::bind_method(D_METHOD("start", "samples", "language"), &SpeechRecognizer::start);
  ClassDB::bind_method(D_METHOD("is_running"), &SpeechRecognizer::is_running);
  ClassDB::bind_method(D_METHOD("is_done"), &SpeechRecognizer::is_done);
  ClassDB::bind_method(D_METHOD("take_result"), &SpeechRecognizer::take_result);
}

bool SpeechRecognizer::is_available()
{
#ifdef GODOT_RVIZ2_WHISPER
  return true;
#else
  return false;
#endif
}

SpeechRecognizer::~SpeechRecognizer()
{
#ifdef GODOT_RVIZ2_WHISPER
  // The task (declared last) is destroyed first and waits for a running recognition
  if (task_.is_running()) {
    while (task_.is_running()) std::this_thread::yield();
  }
  if (context_) whisper_free(context_);
#endif
}

bool SpeechRecognizer::load_model(const String & path, const String & vad_path)
{
#ifdef GODOT_RVIZ2_WHISPER
  if (context_ || task_.is_running()) return context_ != nullptr;
  whisper_context_params params = whisper_context_default_params();
  params.use_gpu = false;  // keep the GPU for Autoware and the rendering
  context_ = whisper_init_from_file_with_params(path.utf8().get_data(), params);
  vad_path_ = vad_path.utf8().get_data();
  return context_ != nullptr;
#else
  (void)path;
  (void)vad_path;
  return false;
#endif
}

bool SpeechRecognizer::start(const PackedFloat32Array & samples, const String & language)
{
#ifdef GODOT_RVIZ2_WHISPER
  if (!context_ || samples.is_empty()) return false;
  std::vector<float> pcm(samples.ptr(), samples.ptr() + samples.size());
  language_ = language.utf8().get_data();
  return task_.start([this, pcm = std::move(pcm)]() {
    whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
    params.language = language_.c_str();
    params.suppress_nst = true;  // no "(music)" and such for non-speech
    if (!vad_path_.empty()) {
      params.vad = true;
      params.vad_model_path = vad_path_.c_str();
      params.vad_params.threshold = 0.5f;
      params.vad_params.min_speech_duration_ms = 200;
    }
    params.n_threads = std::max(1, std::min(4, int(std::thread::hardware_concurrency()) / 2));
    params.no_context = true;
    params.single_segment = true;
    // Encode only as much as the utterance needs (whisper pads to 30 s otherwise), for short
    // commands much faster: 50 frames per second, plus a margin
    params.audio_ctx = std::min(1500, int(pcm.size() / 16000.0 * 50.0) + 64);
    params.print_progress = false;
    params.print_realtime = false;
    params.print_timestamps = false;
    params.print_special = false;
    Result result;
    if (whisper_full(context_, params, pcm.data(), int(pcm.size())) == 0) {
      float p_sum = 0.0f;
      int tokens = 0;
      const whisper_token eot = whisper_token_eot(context_);
      for (int i = 0; i < whisper_full_n_segments(context_); ++i) {
        result.text += whisper_full_get_segment_text(context_, i);
        if (i == 0) result.no_speech = whisper_full_get_segment_no_speech_prob(context_, i);
        for (int j = 0; j < whisper_full_n_tokens(context_, i); ++j) {
          if (whisper_full_get_token_id(context_, i, j) >= eot) continue;  // special tokens
          p_sum += whisper_full_get_token_p(context_, i, j);
          ++tokens;
        }
      }
      result.confidence = tokens > 0 ? p_sum / tokens : 0.0f;
    }
    return result;
  });
#else
  (void)samples;
  (void)language;
  return false;
#endif
}

Dictionary SpeechRecognizer::take_result()
{
  const Result result = task_.take();
  Dictionary dict;
  dict["text"] = String::utf8(result.text.c_str());
  dict["no_speech"] = result.no_speech;
  dict["confidence"] = result.confidence;
  return dict;
}
