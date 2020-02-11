cmake_minimum_required(VERSION 2.8.3)
project(temizlik_robotu_engelden_sakinma)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

generate_messages(DEPENDENCIES std_msgs nav_msgs sensor_msgs)

catkin_package(
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
