cmake_minimum_required(VERSION 2.8.3)
project(luggage)

set(CMAKE_CXX_STANDARD 17)

# set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  behaviortree_cpp_v3
  roslint
  sensor_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
  darknet_ros_msgs
  dialogflow_ros_msgs
  std_srvs
  sound_play
  actionlib
  actionlib_msgs
  gb_dialog
  backward_ros
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs behaviortree_cpp_v3
  sensor_msgs
  tf
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)

find_package(OpenCV)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZMQ_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_library(luggage src/PIDController.cpp)
add_library(asr_start_bt_node SHARED src/luggage/Start.cpp)
add_library(asr_follow_person1_bt_node SHARED src/luggage/FollowPerson.cpp)
add_library(asr_lost_bt_node SHARED src/luggage/Lost.cpp)
add_library(asr_percieve_person1_bt_node SHARED src/luggage/PercievePerson.cpp)
add_library(asr_go_to_bag_bt_node SHARED src/luggage/GoToBag.cpp)
add_library(asr_detect_luggage_bt_node SHARED src/luggage/DetectLuggage.cpp)
add_library(asr_go_to_ref_bt_node SHARED src/luggage/GoToRef.cpp)
add_library(asr_go_to_origin_bt_node SHARED src/luggage/GoToOrigin.cpp)
list(APPEND plugin_libs
  asr_start_bt_node
  asr_follow_person1_bt_node
  asr_lost_bt_node
  asr_percieve_person1_bt_node
  asr_go_to_bag_bt_node 
  asr_detect_luggage_bt_node 
  asr_go_to_ref_bt_node
  asr_go_to_origin_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} luggage)
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(luggage_node src/luggage_node.cpp src/luggage/DialogInterface.cpp )
target_link_libraries(luggage_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

roslint_cpp(
  src/luggage_node.cpp
  include/luggage/nav_node.h src/luggage/nav_node.cpp
  include/luggage/DialogInterface.h src/luggage/DialogInterface.cpp
  include/luggage/Dialog.h
  include/luggage/GoToOrigin.h src/luggage/GoToOrigin.cpp
  include/luggage/GoToRef.h src/luggage/GoToRef.cpp
  include/luggage/GoToBag.h src/luggage/GoToBag.cpp
  include/luggage/DetectLuggage.h src/luggage/DetectLuggage.cpp
  include/luggage/Lost.h src/luggage/Lost.cpp
  include/luggage/PIDController.h src/luggage/PIDController.cpp
  include/luggage/Start.h src/luggage/Start.cpp
)