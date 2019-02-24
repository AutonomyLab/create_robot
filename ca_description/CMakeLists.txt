cmake_minimum_required(VERSION 3.5)
project(ca_description)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)

install(DIRECTORY urdf
  DESTINATION urdf
  )

install(DIRECTORY meshes
  DESTINATION meshes
  )

ament_package()
