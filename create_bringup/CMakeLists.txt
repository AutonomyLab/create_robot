cmake_minimum_required(VERSION 3.5)
project(create_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN ".svn" EXCLUDE
)

ament_package()
