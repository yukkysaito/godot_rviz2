#!/bin/bash
# Builds whisper.cpp (speech recognition for the Orb voice copilot) as static libraries, and gets
# the models: kotoba-whisper (trained for Japanese, about 540 MB) and Silero VAD (finds the speech,
# about 1 MB). The godot_rviz2 module links whisper.cpp when it is built here; without it the
# copilot is disabled.
#   scripts/build_whisper.sh
set -e
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WHISPER="$ROOT/godot_rviz2/thirdparty/whisper.cpp"

git -C "$ROOT" submodule update --init godot_rviz2/thirdparty/whisper.cpp
cmake -S "$WHISPER" -B "$WHISPER/build" -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DGGML_NATIVE=OFF -DGGML_OPENMP=OFF \
  -DWHISPER_BUILD_TESTS=OFF -DWHISPER_BUILD_EXAMPLES=OFF -DWHISPER_BUILD_SERVER=OFF
cmake --build "$WHISPER/build" -j"$(nproc)" --target whisper

get() {  # file url
  [ -f "$ROOT/models/$1" ] || curl -L -o "$ROOT/models/$1" "$2"
}
mkdir -p "$ROOT/models"
get ggml-kotoba-whisper-v2.0-q5_0.bin \
  https://huggingface.co/kotoba-tech/kotoba-whisper-v2.0-ggml/resolve/main/ggml-kotoba-whisper-v2.0-q5_0.bin
get ggml-silero-v6.2.0.bin https://huggingface.co/ggml-org/whisper-vad/resolve/main/ggml-silero-v6.2.0.bin
echo "whisper.cpp built; models in $ROOT/models"
