cmake_minimum_required(VERSION 2.8.3)
project(create_driver)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
)

find_package(Boost REQUIRED system thread)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES libcreate create_driver
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp std_msgs tf
  DEPENDS system_lib
)

include(ExternalProject)
ExternalProject_Add(libcreate
  GIT_REPOSITORY https://github.com/AutonomyLab/libcreate.git
  GIT_TAG master
  PREFIX ${CATKIN_DEVEL_PREFIX}
  CONFIGURE_COMMAND cmake . 
  BUILD_COMMAND make
  INSTALL_COMMAND echo "No install" 
  #make install INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}/lib/create
  BUILD_IN_SOURCE 1
)

set(libcreate_PATH "${CATKIN_DEVEL_PREFIX}/src/libcreate/")

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${libcreate_PATH}/include
  include
)

link_directories(${libcreate_PATH})

add_executable(create_driver src/create_driver.cpp)

target_link_libraries(create_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  -lboost_system # Why need these two lines?
  -lboost_thread
  create
)
add_dependencies(create_driver libcreate)
