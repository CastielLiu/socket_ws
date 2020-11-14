cmake_minimum_required(VERSION 2.8.3)
project(remote_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             roscpp
             sensor_msgs
             cv_bridge
             )

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
             sensor_msgs
             cv_bridge
)
include_directories(
             include
             ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")


add_executable(fixed_client_node src/fixed_client_node.cpp)
target_include_directories(fixed_client_node PRIVATE ${OpenCV_INCLUDE_DIR})
target_link_libraries(fixed_client_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

#add_executable(move_client_node src/move_client_node.cpp)
#target_include_directories(move_client_node PRIVATE ${OpenCV_INCLUDE_DIR})
#target_link_libraries(move_client_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
