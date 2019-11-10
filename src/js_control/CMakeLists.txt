cmake_minimum_required(VERSION 2.8.3)
project(js_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(little_ant_msgs)

IF(little_ant_msgs_FOUND)

	catkin_package(
	  INCLUDE_DIRS include
	  LIBRARIES js_control
	  CATKIN_DEPENDS little_ant_msgs roscpp rospy std_msgs
	#  DEPENDS system_lib
	)



	include_directories(
		include
		${catkin_INCLUDE_DIRS}
	)

	add_executable(js_control src/js_control.cpp)
	add_dependencies(js_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
	target_link_libraries(js_control ${catkin_LIBRARIES})

ELSE()
	message(WARNING "little_ant_msgs not founded, package js_control not compile!!!")
ENDIF()
