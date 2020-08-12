cmake_minimum_required(VERSION 3.5)
project(mcl_3dl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7.2 REQUIRED
  COMPONENTS
    core
    filters
    kdtree
    io
)

find_package(geometry_msgs REQUIRED)
find_package(mcl_3dl_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(dependencies
  rclcpp

#  tf2
#  tf2_geometry_msgs
#  tf2_ros
#  tf2_sensor_msgs

#  diagnostic_updater
  geometry_msgs
  mcl_3dl_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

# Workaround for debian stretch build (https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894656)
list(REMOVE_ITEM PCL_LIBRARIES
  "vtkGUISupportQt"
  "vtkGUISupportQtOpenGL"
  "vtkGUISupportQtSQL"
  "vtkGUISupportQtWebkit"
  "vtkViewsQt"
  "vtkRenderingQt"
)

# Workaround for the bug in PCL<1.8.1 (https://github.com/PointCloudLibrary/pcl/issues/1406)
remove_definitions(-DDISABLE_LIBUSB-1.0)

# Workaround for PCL-1.7 problem on C++11 (https://github.com/PointCloudLibrary/pcl/issues/619)
# Debug build without optimization causes Segmentation fault.
# Set default build type to Release and add -O1 to CXX_FLAGS_DEBUG.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)


add_executable(map_loader_node
  src/map_loader.cpp
  src/map_loader_node.cpp
)

ament_target_dependencies(map_loader_node
  rclcpp
  sensor_msgs
)
target_link_libraries(map_loader_node
  ${PCL_LIBRARIES}
)

add_executable(mcl_3dl
  src/cloud_accum.cpp
  src/lidar_measurement_model_beam.cpp
  src/lidar_measurement_model_likelihood.cpp
  src/parameters.cpp

  # to avoid the undefined reference to 'main' error for now
  src/dummy.cpp
)
ament_target_dependencies(mcl_3dl
  rclcpp
  geometry_msgs
  mcl_3dl_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

target_link_libraries(mcl_3dl
  ${PCL_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
