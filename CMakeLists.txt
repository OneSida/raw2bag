cmake_minimum_required(VERSION 3.8.2)
project(raw2bag)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(Protobuf REQUIRED)

set(ROS_DEP roscpp sensor_msgs rosbag)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEP})
catkin_package(CATKIN_DEPENDS ${ROS_DEP})

aux_source_directory(src ALL_SRC)
add_executable(${PROJECT_NAME} main.cc ${ALL_SRC})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${PROTOBUF_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${PROTOBUF_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./)

