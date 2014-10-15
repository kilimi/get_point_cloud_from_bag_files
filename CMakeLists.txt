cmake_minimum_required(VERSION 2.8.3)
project(get_point_cloud_from_bag_files)

find_package(catkin REQUIRED COMPONENTS rosbag roscpp rospy std_msgs sensor_msgs cv_bridge image_transport)
find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )

catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS rosbag roscpp topic_tools)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(CoViS REQUIRED HINTS $ENV{HOME}/CoViS $ENV{HOME}/Code/CoViS $ENV{HOME}/workspace/CoViS ../../../CoViS)
include_directories( ${COVIS_INCLUDE_DIR} )
link_directories( ${COVIS_LIBRARY_DIRS} )

add_executable(start src/start.cpp)
target_link_libraries(start ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${COVIS_LIBRARIES} ) 
add_dependencies(start ${catkin_EXPORTED_TARGETS})