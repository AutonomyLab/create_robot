cmake_minimum_required(VERSION 2.8.3)
project(ca_safety_controller)
find_package(catkin REQUIRED COMPONENTS roscpp nodelet pluginlib std_msgs geometry_msgs ca_msgs yocs_controllers ecl_threads)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ca_safety_controller_nodelet
   CATKIN_DEPENDS roscpp nodelet pluginlib std_msgs geometry_msgs ca_msgs yocs_controllers ecl_threads
)

include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_library(ca_safety_controller_nodelet src/nodelet.cpp)
add_dependencies(ca_safety_controller_nodelet ca_msgs_gencpp)
target_link_libraries(ca_safety_controller_nodelet ${catkin_LIBRARIES})

install(TARGETS ca_safety_controller_nodelet
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
