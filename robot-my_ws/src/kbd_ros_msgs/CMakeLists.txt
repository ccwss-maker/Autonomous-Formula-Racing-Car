cmake_minimum_required(VERSION 2.8.12)

project(kbd_ros_msgs)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
)

add_message_files(
  FILES
    kbd.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    roscpp
    message_runtime
    std_msgs
)
