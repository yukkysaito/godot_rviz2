# godot_rviz2 (ROS2 humble)

[![](https://github.com/user-attachments/assets/437755d1-6c20-449f-8da2-0ba3e8c42669)](https://youtu.be/mqZM1AkbA5A?si=EQV2JB2tUFsGdfVd)


## Environment

- Ubuntu 22.04
- GPU
- Autoware (step1 in How to use)
- Internet connection (optional): the mini map shows OpenStreetMap tiles when online, and the vector map otherwise

## How to use

1. Autoware build following [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

1. Build the godot_rviz2 GDExtension (the ROS 2 / Autoware classes Godot uses)
   1. install GeographicLib, used to place the map on OpenStreetMap (usually installed with Autoware).

      ```bash
      sudo apt-get install libgeographic-dev
      ```

   1. clone godot_rviz2

      ```bash
      git clone https://github.com/yukkysaito/godot_rviz2.git --recursive
      ```

   1. build it against Autoware. The library is put in `godot-project/addons/godot_rviz2/bin/`.

      ```bash
      cd godot_rviz2
      source <autoware_path>/install/setup.bash
      colcon build --packages-select godot_rviz2 --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```

1. Download the official Godot editor of the version the extension is built for (the one godot-cpp is pinned to). It prints the editor's path, used as `<godot>` below.

   ```bash
   scripts/get_godot.sh
   ```

1. Import the project once (the first import also registers the extension; errors about missing classes in it are expected)

   ```bash
   source <autoware_path>/install/setup.bash
   <godot> --path godot-project --editor --quit
   ```

1. Run Autoware and Godot
   1. Run Autoware
   1. Run Godot with the project (with Autoware's environment, which the extension needs)

      ```bash
      source <autoware_path>/install/setup.bash
      <godot> --path godot-project
      ```

      To edit the project instead, add `--editor` and click the run button on the top right.

## Key Config

| key   | action           |
| ----- | ---------------- |
| Tab   | Switch view mode |

## How to run from ROS2 launch

1. Create godot_rviz2.py and modify paths in `cmd`.

```python
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    godot_rviz2 = ExecuteProcess(
        cmd=[[
            '<godot> --path ~/workspace/godot_rviz2/godot-project'
        ]],
        shell=True
    )
    return LaunchDescription([godot_rviz2])
```

2. Run ros2 command.

```bash
ros2 launch <path>/godot_rviz2.py
```

## How to Export Binary

1. Install the export templates: in the editor, Editor > Manage Export Templates > Download and Install.
1. Export with the `Linux/X11` preset (Project > Export). The exported binary needs `libgodot_rviz2.linux.x86_64.so` next to it, which Godot copies there.
1. Run

   ```bash
   source <autoware_path>/install/setup.bash
   ./sample.x86_64
   ```

## Videos
[![](http://img.youtube.com/vi/LPzkEC5hBMo/0.jpg)](https://www.youtube.com/watch?v=LPzkEC5hBMo)
[![](http://img.youtube.com/vi/7udy3QDXQBk/0.jpg)](https://www.youtube.com/watch?v=7udy3QDXQBk)
[![](http://img.youtube.com/vi/r8NtqiF3JNg/0.jpg)](https://www.youtube.com/watch?v=r8NtqiF3JNg)


## Reference

- [font](https://github.com/adobe-fonts/source-code-pro)
- [steering icon](https://icooon-mono.com/13897-%E3%83%8F%E3%83%B3%E3%83%89%E3%83%AB%E3%82%A2%E3%82%A4%E3%82%B3%E3%83%B31/)
- [meter icon](https://icooon-mono.com/13350-%E3%83%A1%E3%83%BC%E3%82%BF%E3%83%BC%E3%82%A2%E3%82%A4%E3%82%B3%E3%83%B37/)
- [vehicle 3D model](https://github.com/tier4/AWSIM/tree/v1.0.1/Assets/AWSIM/Models/Vehicles/Lexus%20RX450h%202015)
