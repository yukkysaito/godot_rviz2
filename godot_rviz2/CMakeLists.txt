cmake_minimum_required(VERSION 3.16)
project(godot_rviz2 LANGUAGES CXX)

# The GDExtension that gives Godot the ROS 2 / Autoware classes (see register_types.cpp). It is
# built into godot-project/addons/godot_rviz2/: bin/ and the godot_rviz2.gdextension that loads it.

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# The Godot version (GODOT_VERSION at the repository root, which scripts/get_godot.sh downloads
# too): the extension is built against the API of its <major>.<minor>
file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/../GODOT_VERSION" GODOT_VERSION LIMIT_COUNT 1)
string(REGEX MATCH "^[0-9]+\\.[0-9]+" GODOT_API_VERSION "${GODOT_VERSION}")

# godot-cpp for that API, with exceptions (rclcpp and the libraries below throw them) and with the
# shared libstdc++ the ROS libraries use (a static copy breaks exceptions thrown across them)
set(GODOTCPP_API_VERSION "${GODOT_API_VERSION}" CACHE STRING "" FORCE)
set(GODOTCPP_TARGET template_release CACHE STRING "" FORCE)
set(GODOTCPP_DISABLE_EXCEPTIONS OFF CACHE BOOL "" FORCE)
set(GODOTCPP_USE_STATIC_CPP OFF CACHE BOOL "" FORCE)
add_subdirectory(thirdparty/godot-cpp EXCLUDE_FROM_ALL)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_map_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(autoware_planning_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_lanelet2_extension REQUIRED)
find_package(lanelet2_core REQUIRED)
find_package(Eigen3 REQUIRED)
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)  # FindGeographicLib.cmake (libgeographic-dev)
find_package(GeographicLib REQUIRED)

file(GLOB SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_library(${PROJECT_NAME} SHARED ${SOURCES} register_types.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
  include
  thirdparty
  ${GeographicLib_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
  sensor_msgs
  autoware_adapi_v1_msgs
  autoware_map_msgs
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_vehicle_msgs
  autoware_lanelet2_extension
  lanelet2_core)
target_link_libraries(${PROJECT_NAME} godot::cpp Eigen3::Eigen ${GeographicLib_LIBRARIES})

# libgodot_rviz2.linux.<x86_64|arm64>.so (the names godot_rviz2.gdextension lists). The ROS and
# Autoware libraries are found at run time through the environment (source setup.bash).
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(GODOT_ARCH arm64)
else()
  set(GODOT_ARCH ${CMAKE_SYSTEM_PROCESSOR})
endif()
set(GODOT_ADDON_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../godot-project/addons/godot_rviz2"
  CACHE PATH "Where the extension is put for the Godot project")
set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME "${PROJECT_NAME}.linux.${GODOT_ARCH}"
  LIBRARY_OUTPUT_DIRECTORY "${GODOT_ADDON_DIR}/bin"
  SKIP_BUILD_RPATH ON)
configure_file(godot_rviz2.gdextension.in "${GODOT_ADDON_DIR}/godot_rviz2.gdextension" @ONLY)

ament_package()
