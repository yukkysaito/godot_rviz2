# godot_rviz2 (ROS2 humble)

[![](https://github.com/user-attachments/assets/437755d1-6c20-449f-8da2-0ba3e8c42669)](https://youtu.be/mqZM1AkbA5A?si=EQV2JB2tUFsGdfVd)

A 3D viewer for [Autoware](https://github.com/autowarefoundation/autoware) made with [Godot](https://godotengine.org/). The ROS 2 / Autoware side is a GDExtension (`godot_rviz2/`, C++), and the viewer is a Godot project (`godot-project/`).

## Environment

- Ubuntu 22.04, ROS 2 Humble
- A GPU with Vulkan
- Autoware (step 1 in How to use)
- Internet connection (optional): the mini map shows OpenStreetMap tiles when online, and the vector map otherwise

## How to use

1. Build Autoware following [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

1. Build the godot_rviz2 GDExtension (the ROS 2 / Autoware classes Godot uses)
   1. Install GeographicLib, used to place the map on OpenStreetMap (usually installed with Autoware).

      ```bash
      sudo apt-get install libgeographic-dev
      ```

   1. Clone godot_rviz2 with its submodule, [godot-cpp](https://github.com/godotengine/godot-cpp) (the C++ bindings the extension is built with). In a clone without it, run `git submodule update --init`.

      ```bash
      git clone https://github.com/yukkysaito/godot_rviz2.git --recursive
      ```

   1. Build it against Autoware. It is put in `godot-project/addons/godot_rviz2/`.

      ```bash
      cd godot_rviz2
      source <autoware_path>/install/setup.bash
      colcon build --packages-select godot_rviz2 --cmake-args -DCMAKE_BUILD_TYPE=Release
      ```

1. Download Godot itself, which runs the project: the official editor of the version the extension is built for (`GODOT_VERSION`), checked against the release's checksums. The script prints the editor's path, used as `<godot>` below.

   ```bash
   scripts/get_godot.sh
   ```

1. Import the project once.

   ```bash
   source <autoware_path>/install/setup.bash
   <godot> --path godot-project --editor --quit
   ```

1. Run Autoware and Godot
   1. Run Autoware
   1. Run Godot with the project, with Autoware's environment, which the extension needs. Use the same RMW implementation and DDS settings as Autoware (for example `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI`).

      ```bash
      source <autoware_path>/install/setup.bash
      <godot> --path godot-project
      ```

      To edit the project instead, add `--editor` and click the run button on the top right.

## Controls

| Input                                                      | Action                                                                           |
| ---------------------------------------------------------- | -------------------------------------------------------------------------------- |
| Tab                                                        | Switch the view (third person / bird's-eye)                                      |
| Left drag                                                  | Rotate the camera (it returns behind the vehicle after a while)                  |
| Mouse wheel                                                | Zoom                                                                             |
| One-finger drag / two-finger pinch                         | Rotate / zoom on a touch screen                                                  |
| Click the mini map                                         | Expand it to the whole route, and back                                           |
| Click the grabber on the left edge, or swipe from the edge | Open / close the menu (display, view, data and HUD settings); Esc also closes it |
| Double-click a slider in the menu                          | Reset it to its default                                                          |

## Settings

The menu saves the choices to `~/.local/share/godot/app_userdata/godot_rviz2/settings.cfg`. They are read in this order, and each layer overrides the previous one (see `godot-project/Core/Settings.gd`):

1. the defaults in `Core/Settings.gd`
1. `godot-project/Config/project.cfg` (optional), for a derived project to change the defaults
1. a preset from `godot-project/Config/Presets/`, chosen on the command line: `real_vehicle` or `jetson` (full screen, lighter anti-aliasing)
1. the user's `settings.cfg`
1. command-line options, which are not saved

```bash
<godot> --path godot-project -- --preset=real_vehicle
```

`-- --vehicle=<Name>` selects the vehicle model `godot-project/3DViewer/Vehicle/<Name>/<Name>.tres`.

## Mini game

Swipe from the right edge of the view to the left to open "Planner Challenge": you are the velocity planner of an automated vehicle. Drag up to accelerate, down to brake, and release to coast. Reach the goal fast while keeping the ride comfortable and safe, then stop on the line. You race a rival ghost, a rule-based planner whose time is the course's target. There is a daily course and random ones, and best scores are kept. Close the game with the button at the top right or with Esc.

The game is a simulation only: it never sends anything to ROS. While it is open, the viewer's 3D view is not drawn, but the viewer keeps receiving data.

The `real_vehicle` and `jetson` presets turn the game off with `[game] enabled=false`. You can also set this in `godot-project/Config/project.cfg`.

The game is self-contained in `godot-project/Game/`. To remove it, delete that folder and the `MiniGame` autoload line in `godot-project/project.godot`.

## How to run from ROS2 launch

1. Create godot_rviz2.py and modify the paths in `cmd`.

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

1. Run it with Autoware's environment.

   ```bash
   ros2 launch <path>/godot_rviz2.py
   ```

## How to Export Binary

1. Install the export templates: in the editor, Editor > Manage Export Templates > Download and Install.
1. Export with the `Linux/X11` preset (Project > Export). Turn on Embed PCK in the preset to get a single binary instead of a binary and a `.pck`. Godot copies `libgodot_rviz2.linux.x86_64.so` next to the binary, which needs it there.
1. Run it with Autoware's environment.

   ```bash
   source <autoware_path>/install/setup.bash
   ./sample.x86_64
   ```

## Performance measurement

`--perf` prints frame times and start-up timings, `--perf-duration=N` quits after N seconds with a summary, and `--perf-screenshot=T1,T2,...` saves screenshots at those times to `~/.local/share/godot/app_userdata/godot_rviz2/perf/` (see `godot-project/Debug/PerfMonitor.gd`).

```bash
<godot> --path godot-project -- --perf --perf-duration=60
```

## Videos
[![](http://img.youtube.com/vi/LPzkEC5hBMo/0.jpg)](https://www.youtube.com/watch?v=LPzkEC5hBMo)
[![](http://img.youtube.com/vi/7udy3QDXQBk/0.jpg)](https://www.youtube.com/watch?v=7udy3QDXQBk)
[![](http://img.youtube.com/vi/r8NtqiF3JNg/0.jpg)](https://www.youtube.com/watch?v=r8NtqiF3JNg)


## Reference

- fonts: [Barlow](https://github.com/jpt/barlow) (SIL Open Font License, `godot-project/UI/Fonts/OFL.txt`) and [Source Code Pro](https://github.com/adobe-fonts/source-code-pro)
- mini map tiles: © [OpenStreetMap](https://www.openstreetmap.org/copyright) contributors
- [vehicle 3D model](https://github.com/tier4/AWSIM/tree/v1.0.1/Assets/AWSIM/Models/Vehicles/Lexus%20RX450h%202015)
