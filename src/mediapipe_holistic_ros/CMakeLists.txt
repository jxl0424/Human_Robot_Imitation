cmake_minimum_required(VERSION 2.8.3)
project(mediapipe_holistic_ros)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

add_message_files(
  FILES
    MediaPipePose.msg
    MediaPipeHolistic.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


#add_executable(map_to_odom src/tf/map_to_odom.cpp)
#target_link_libraries(map_to_odom ${catkin_LIBRARIES})
