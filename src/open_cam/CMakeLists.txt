cmake_minimum_required(VERSION 3.5)
project(open_cam)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(CUDA REQUIRED)
find_package(darknet_ros_msgs REQUIRED)


include_directories(
  ${CUDA_INCLUDE_DIRS}
  ${ZED_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(open_cam src/main.cpp)
ament_target_dependencies(open_cam rclcpp sensor_msgs darknet_ros_msgs OpenCV cv_bridge ZED CUDA)


install(TARGETS
  open_cam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  DESTINATION lib/${PROJECT_NAME}
  
)



ament_package()
