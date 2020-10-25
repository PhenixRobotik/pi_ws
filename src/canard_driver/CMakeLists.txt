cmake_minimum_required(VERSION 3.0.2)
project(canard_driver)

add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services
#add_message_files(DIRECTORY msg FILES Num.msg)
#add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

include_directories(include libcanard/libcanard ${catkin_INCLUDE_DIRS})

add_executable(driver src/driver.cpp src/can.cpp src/can2ros.cpp src/can2ros_z.cpp src/can2ros_tsmr.cpp libcanard/libcanard/canard.c)
target_link_libraries(driver ${catkin_LIBRARIES} socketcan)
add_dependencies(driver canard_driver_generate_messages_cpp)

add_executable(odom_converter src/odom_converter.cpp)
target_link_libraries(odom_converter ${catkin_LIBRARIES})
