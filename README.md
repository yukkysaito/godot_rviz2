# godot_rviz2
[![](http://img.youtube.com/vi/7udy3QDXQBk/0.jpg)](https://www.youtube.com/watch?v=7udy3QDXQBk)
[![](http://img.youtube.com/vi/r8NtqiF3JNg/0.jpg)](https://www.youtube.com/watch?v=r8NtqiF3JNg)

## Environment
- Ubuntu 20.04
- GPU 
- Autoware (step1 in How to use )
- Godot 3.4.4 (step2 in How to use )

## How to use
1. Autoware build following [here](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).

2. Build godot from source
   1. clone godot
   ```bash
   git clone https://github.com/godotengine/godot
   ```

   2. change version
   ```bash
   cd godot
   git checkout 3.4.4-stable
   ```

   3. build following [here](https://docs.godotengine.org/en/stable/development/compiling/compiling_for_x11.html). Note in 3.4.4-stable, not x11 as platform
   ```bash
   scons platform=linux 
    ```

3. Build with ros2 component
   1. modify the path in `godot_rviz2/SCsub`. It is needed to include autoware custum msg type because build system is defferent between colcon(ROS2) and scon(Godot)
   ```
   autoware_include_path = "/home/yukky/workspace/tutorial/autoware"
   ```

   2. build with ros2 component. 
   ```bash
   # run in godot directory
   scons platform=linux -j8 custom_modules=<path>/godot_rviz2/godot_rviz2
   ```
   If you get error about console_bridge, put following command.
   ```
   sudo ln -s /opt/ros/galactic/lib/x86_64-linux-gnu/libconsole_bridge.so.1.0 /opt/ros/galactic/lib/libconsole_bridge.so
   ```

4. Run Autoware and Godot
   1. Run Autoware

   2. Run Godot and import project from `godot_rviz2/godot-project`
    ```
    ./bin/godot.x11.tools.64
    ```

   3. Click run button on top right on Godot GUI.
