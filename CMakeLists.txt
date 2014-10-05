cmake_minimum_required(VERSION 2.8.3)
project(get_point_cloud_from_bag_files)

find_package(catkin REQUIRED COMPONENTS rosbag roscpp rospy std_msgs sensor_msgs cv_bridge image_transport)
find_package(Boost REQUIRED COMPONENTS system)
find_package( OpenCV REQUIRED )

#add_service_files(FILES test.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)


catkin_package(INCLUDE_DIRS include CATKIN_DEPENDS rosbag roscpp topic_tools)

include_directories(include ${catkin_INCLUDE_DIRS})

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#SET(RW_ROOT "$ENV{RW_ROOT}")
#SET(RWS_ROOT "$ENV{RWS_ROOT}")
#SET(RWSIM_ROOT "$ENV{RWSIM_ROOT}")

#SET(RobWork_DIR ${RW_ROOT}/cmake)
#SET(RobWorkStudio_DIR ${RWS_ROOT}cmake)
#SET(RobWorkSim_DIR ${RWSIM_ROOT}cmake)

#Include default settings for constructing a robwork dependent project

#FIND_PACKAGE(RobWork REQUIRED)
#FIND_PACKAGE(RobWorkStudio REQUIRED)
#FIND_PACKAGE(RobWorkSim REQUIRED)
#INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} ${ROBWORKSIM_INCLUDE_DIRS})
#LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} ${ROBWORKSIM_LIBRARY_DIRS})

find_package(CoViS REQUIRED HINTS $ENV{HOME}/CoViS $ENV{HOME}/Code/CoViS $ENV{HOME}/workspace/CoViS ../../../CoViS)
include_directories( ${COVIS_INCLUDE_DIR} )
link_directories( ${COVIS_LIBRARY_DIRS} )

add_executable(start src/start.cpp)
target_link_libraries(start ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${COVIS_LIBRARIES} ) 
add_dependencies(start ${catkin_EXPORTED_TARGETS})