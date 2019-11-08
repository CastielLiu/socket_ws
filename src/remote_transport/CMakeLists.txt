cmake_minimum_required(VERSION 2.8.3)
project(remote_transport)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED
             roscpp
             sensor_msgs
             cv_bridge)

find_package(OpenCV REQUIRED)

catkin_package(CATKIN_DEPENDS
             sensor_msgs
             cv_bridge)

include_directories(
             include
             ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")


add_executable(msg_sender_node src/msg_sender.cpp)
target_include_directories(msg_sender_node PRIVATE ${OpenCV_INCLUDE_DIR})
target_link_libraries(msg_sender_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(msg_receiver_node src/msg_receiver.cpp)
target_include_directories(msg_receiver_node PRIVATE ${OpenCV_INCLUDE_DIR})
target_link_libraries(msg_receiver_node ${catkin_LIBRARIES} ${OpenCV_LIBS})
