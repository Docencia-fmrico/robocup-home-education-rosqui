cmake_minimum_required(VERSION 2.8.3)
project(find_my_mates)

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

add_library(asr_analyze_person_bt_node SHARED src/find_my_mates/Analyze_person.cpp)
add_library(asr_go_to_arena_bt_node SHARED src/find_my_mates/Go_to_arena.cpp)
add_library(asr_go_to_person_bt_node SHARED src/find_my_mates/Go_to_person.cpp)
add_library(asr_go_to_referee_bt_node SHARED src/find_my_mates/Go_to_referee.cpp)
add_library(asr_go_say_the_description_bt_node SHARED src/find_my_mates/Say_the_description.cpp)
list(APPEND plugin_libs
  asr_analyze_person_bt_node
  asr_go_to_arena_bt_node
  asr_go_to_person_bt_node
  asr_go_to_referee_bt_node
  asr_go_say_the_description_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(find_my_mates_node src/find_my_mates_node.cpp)
target_link_libraries(find_my_mates_node ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${OpenCV_LIBRARIES})

set(cv_bridge_DIR /usr/local/share/cv_bridge/cmake)

roslint_cpp(
  src/find_my_mates_node.cpp
  include/find_my_mates/Analyze_person.h src/find_my_mates/Analyze_person.cpp
  include/find_my_mates/Go_to_arena.h src/find_my_mates/Go_to_arena.cpp
  include/find_my_mates/Go_to_person.h src/find_my_mates/Go_to_person.cpp
  include/find_my_mates/Go_to_referee.h src/find_my_mates/Go_to_referee.cpp
  include/find_my_mates/Say_the_description.h src/find_my_mates/Say_the_description.cpp
  include/find_my_mates/BTNavAction.h
  include/find_my_mates/DialogInterface.h
  include/find_my_mates/Dialog.h
)