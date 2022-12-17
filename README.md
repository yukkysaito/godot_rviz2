# godot_rviz2 (ROS2 humble)

[![](http://img.youtube.com/vi/LPzkEC5hBMo/0.jpg)](https://www.youtube.com/watch?v=LPzkEC5hBMo)
[![](http://img.youtube.com/vi/7udy3QDXQBk/0.jpg)](https://www.youtube.com/watch?v=7udy3QDXQBk)
[![](http://img.youtube.com/vi/r8NtqiF3JNg/0.jpg)](https://www.youtube.com/watch?v=r8NtqiF3JNg)

## Environment

- Ubuntu 22.04
- GPU
- Autoware (step1 in How to use)

## How to use

1. Autoware build following [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

1. Build godot from source
   1. install pre-requirements following [here](https://docs.godotengine.org/en/stable/development/compiling/compiling_for_x11.html#distro-specific-one-liners) by godot.

      ```bash
      sudo apt-get install build-essential scons pkg-config libx11-dev libxcursor-dev libxinerama-dev libgl1-mesa-dev libglu-dev libasound2-dev libpulse-dev libudev-dev libxi-dev libxrandr-dev yasm
      ```

   1. clone godot_rviz2

      ```bash
      git clone https://github.com/yukkysaito/godot_rviz2.git --recursive
      ```

   1. build godot with autoware and ros2 components.

      ```bash
      cd godot_rviz2/godot
      source <autoware_path>/install/setup.bash
      scons platform=linux -j8 custom_modules=../godot_rviz2
      ```

1. Run Autoware and Godot
   1. Run Autoware
   1. Run Godot

      ```bash
      ./bin/godot.x11.tools.64
      ```
   1. Click run button on top right on Godot GUI.

## Key Config

| key   | action           |
| ----- | ---------------- |
| Space | Show menu        |
| Tab   | Switch view mode |

## How to customize

1. Run Godot and import project from `godot_rviz2/godot-project`

   ```bash
   ./bin/godot.x11.tools.64
   ```

1. Edit on Godot GUI editor

1. Click run button on top right on Godot GUI.

## Reference

- [font](https://github.com/adobe-fonts/source-code-pro)
- [steering icon](https://icooon-mono.com/13897-%E3%83%8F%E3%83%B3%E3%83%89%E3%83%AB%E3%82%A2%E3%82%A4%E3%82%B3%E3%83%B31/)
- [meter icon](https://icooon-mono.com/13350-%E3%83%A1%E3%83%BC%E3%82%BF%E3%83%BC%E3%82%A2%E3%82%A4%E3%82%B3%E3%83%B37/)
