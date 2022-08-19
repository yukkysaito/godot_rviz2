cmake_minimum_required(VERSION 3.14)
project(godot_rviz2)

find_package(autoware_cmake REQUIRED)

# autoware_package()

# Find PCL Dependencies
find_package(PCL REQUIRED)

# Find Eigen Dependencies
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_subdirectory(godot-cpp)

include_directories(
    godot_rviz2/include
    ${catkin_INCLUDE_DIRS}
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    godot-cpp/include
    godot-cpp/include/core
    godot-cpp/include/gen
    godot-cpp/godot_headers)

# ament_auto_add_library(ego_pose SHARED
# godot_rviz2/src/ego_pose.cpp
# godot_rviz2/src/godot_rviz2.cpp
# godot_rviz2/src/util.cpp
# godot_rviz2/src/gdlibrary.cpp)
ament_auto_add_library(pointcloud SHARED
    godot_rviz2/src/pointcloud.cpp
    godot_rviz2/src/godot_rviz2.cpp
    godot_rviz2/src/util.cpp
    godot_rviz2/src/gdlibrary.cpp)

# target_link_libraries(ego_pose
# godot-cpp
# Eigen3::Eigen
# )
target_link_libraries(pointcloud
    godot-cpp
    Eigen3::Eigen
)

ament_auto_package()
