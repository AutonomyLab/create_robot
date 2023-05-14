cmake_minimum_required(VERSION 3.5)
project(create_driver)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(libcreate REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(create_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${libcreate_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
ament_target_dependencies(${PROJECT_NAME}
  create_msgs
  diagnostic_msgs
  diagnostic_updater
  geometry_msgs
  libcreate
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
)

if(BUILD_TESTING)
  # Skip copyright lint and uncrustify
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  # Set language to C++, otherwise cppcheck assumes header file with .h is C
  set(ament_cmake_cppcheck_LANGUAGE c++)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

ament_package()
