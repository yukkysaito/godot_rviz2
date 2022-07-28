# godot_rviz2
[![](http://img.youtube.com/vi/7udy3QDXQBk/0.jpg)](https://www.youtube.com/watch?v=7udy3QDXQBk)
[![](http://img.youtube.com/vi/r8NtqiF3JNg/0.jpg)](https://www.youtube.com/watch?v=r8NtqiF3JNg)

## Environment
- Ubuntu 20.04
- GPU 
- Autoware (step1 in How to use )

## How to use
1. Autoware build following [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

2. Build godot from source
   1. install pre-requirements following [here](https://docs.godotengine.org/en/stable/development/compiling/compiling_for_x11.html#distro-specific-one-liners) by godot.
    
   2. clone godot_rviz2
   ```bash
   git clone https://github.com/yukkysaito/godot_rviz2.git
   ```

   3. build godot with autoware and ros2 components.
   ```bash
   cd godot_rviz2/godot
   source <autoware_path>/install/setup.bash
   scons platform=linux -j8 custom_modules=../godot_rviz2
   ```

3. Run Autoware and Godot
   1. Run Autoware

   2. Run Godot and import project from `godot_rviz2/godot-project`
    ```
    ./bin/godot.x11.tools.64
    ```

   3. Click run button on top right on Godot GUI.
