cmake_minimum_required(VERSION 2.8.3)
project(ca_bumper2pc)
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  nodelet 
  pluginlib 
  ca_msgs
  sensor_msgs 
  kobuki_msgs
)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ca_bumper2pc_nodelet
   CATKIN_DEPENDS roscpp nodelet pluginlib sensor_msgs ca_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(ca_bumper2pc_nodelet src/ca_bumper2pc.cpp)
add_dependencies(ca_bumper2pc_nodelet sensor_msgs_gencpp ca_msgs_gencpp)
target_link_libraries(ca_bumper2pc_nodelet ${catkin_LIBRARIES})

install(TARGETS ca_bumper2pc_nodelet
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
