cmake_minimum_required(VERSION 2.8.3)
project(moveit_sandbox)

find_package(catkin REQUIRED COMPONENTS geometry_msgs
                                        moveit_ros_planning_interface)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(move_group_test src/move_group_test.cpp)

target_link_libraries(move_group_test ${catkin_LIBRARIES})
