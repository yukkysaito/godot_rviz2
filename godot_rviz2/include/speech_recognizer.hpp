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

#pragma once

#include "async_task.hpp"
#include "core/object/ref_counted.h"
#include "core/variant/variant.h"

#include <string>
#include <vector>

struct whisper_context;

/**
 * @class SpeechRecognizer
 * @brief Speech to text with whisper.cpp, on a worker thread (for the Orb voice copilot).
 *
 * load_model() once, then start() an utterance (mono 16 kHz samples) and poll is_done() / take_text()
 * from the main thread. Without whisper.cpp (built with scripts/build_whisper.sh), is_available()
 * is false and nothing is recognized.
 */
class SpeechRecognizer : public RefCounted
{
  GDCLASS(SpeechRecognizer, RefCounted);

public:
  SpeechRecognizer() = default;
  ~SpeechRecognizer();

  static bool is_available();

  /**
   * @brief Loads a ggml whisper model (e.g. ggml-small-q5_1.bin) and, optionally, a Silero VAD
   * model: with it only the parts with speech are recognized, and sound without speech gives no
   * text (whisper alone "hears" words in noise). False if the whisper model cannot be loaded.
   */
  bool load_model(const String & path, const String & vad_path = String());
  bool is_loaded() const { return context_ != nullptr; }

  /**
   * @brief Starts recognizing `samples` (mono, 16 kHz) in `language` ("ja"). False while a
   * recognition runs. (No prompt: whisper tends to repeat a prompt for noise, which made false
   * commands.)
   */
  bool start(const PackedFloat32Array & samples, const String & language);
  bool is_running() const { return task_.is_running(); }
  bool is_done() const { return task_.is_done(); }
  /**
   * @brief The result: {"text": String, "no_speech": float (probability that it was no speech),
   * "confidence": float (mean probability of the recognized tokens)}
   */
  Dictionary take_result();

protected:
  static void _bind_methods();

private:
  whisper_context * context_ = nullptr;
  std::string language_;
  std::string vad_path_;
  struct Result
  {
    std::string text;
    float no_speech = 1.0f;
    float confidence = 0.0f;
  };
  AsyncTask<Result> task_;  // last: it waits for a running job on destruction
};
