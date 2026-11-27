# Cross build for arm64 (e.g. Jetson Orin) against a target root file system with ROS 2 and the
# Autoware packages installed in /opt/ros/humble:
#   cmake -S godot_rviz2 -B build-arm64 -DCMAKE_TOOLCHAIN_FILE=godot_rviz2/cmake/aarch64-linux-gnu.cmake \
#     -DSYSROOT=<sysroot>
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(SYSROOT "" CACHE PATH "Root file system of the target")
set(CMAKE_SYSROOT "${SYSROOT}")
set(CMAKE_FIND_ROOT_PATH "${SYSROOT}")
set(CMAKE_PREFIX_PATH /opt/ros/humble)  # in the sysroot
set(ENV{AMENT_PREFIX_PATH} "${SYSROOT}/opt/ros/humble")  # the ament resource index
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
