cmake_minimum_required(VERSION 3.5)
project(create_autonomy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
# find_package(ca_description EQUIRED)
# find_package(ca_driver REQUIRED)
find_package(ca_msgs REQUIRED)
# find_package(ca_tools REQUIRED)

ament_package()
