cmake_minimum_required(VERSION 3.5)
project(ca_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(joy REQUIRED)
find_package(teleop_twist_joy REQUIRED)

install(DIRECTORY config
  DESTINATION config
  )

ament_package()
