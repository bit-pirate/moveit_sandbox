cmake_minimum_required(VERSION 2.8.3)
project(moveit_sandbox)

find_package(catkin REQUIRED COMPONENTS geometry_msgs
                                        moveit_msgs
                                        moveit_ros_planning_interface
                                        shape_msgs)

catkin_python_setup()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(move_group_test src/move_group_test.cpp)

target_link_libraries(move_group_test ${catkin_LIBRARIES})

install(TARGETS move_group_test
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})