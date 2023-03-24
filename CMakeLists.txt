cmake_minimum_required(VERSION 3.5)
project(raw2bag)

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(OpenCV REQUIRED)

find_package(Protobuf REQUIRED)
set(PROTO_DIR proto)
set(PROTO_FILE ${PROTO_DIR}/error_code.proto ${PROTO_DIR}/header.proto
    ${PROTO_DIR}/sensor_image.proto ${PROTO_DIR}/pointcloud.proto
    ${PROTO_DIR}/message.proto)
protobuf_generate_cpp(PROTO_C PROTO_H ${PROTO_FILE})
include_directories(${CMAKE_CURRENT_BINARY_DIR})

find_package(PCL 1.7 REQUIRED) 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DEFINITIONS}) 

set(ROS_DEP roscpp sensor_msgs rosbag cv_bridge pcl_ros)
find_package(catkin REQUIRED COMPONENTS ${ROS_DEP})
catkin_package(CATKIN_DEPENDS ${ROS_DEP})

aux_source_directory(src ALL_SRC)
add_executable(${PROJECT_NAME} main.cc ${ALL_SRC} ${PROTO_C})
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${catkin_LIBRARIES} ${Protobuf_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC include
    ${catkin_INCLUDE_DIRS} ${Protobuf_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./)

