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

add_library(asr_detect_luggage_bt_node SHARED src/luggage/DetectLuggage.cpp)
add_library(asr_goto_bag_bt_node SHARED src/luggage/GoToBag.cpp)
add_library(asr_dialog_interface_node SHARED src/luggage/DialogInterface.cpp)
list(APPEND plugin_libs
  asr_detect_luggage_bt_node
  asr_dialog_interface_node
  asr_goto_bag_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(luggage_node src/luggage_node.cpp src/luggage/DialogInterface.cpp )
target_link_libraries(luggage_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

roslint_cpp(
  src/luggage_node.cpp
  include/luggage/DetectLuggage.h src/luggage/DetectLuggage.cpp
  include/luggage/GoToBag.h src/luggage/GoToBag.cpp
  include/luggage/DialogInterface.h src/luggage/DialogInterface.cpp
  include/luggage/example.cpp
  include/luggage/Dialog.h
)