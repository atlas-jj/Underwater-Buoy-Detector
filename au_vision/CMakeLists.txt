cmake_minimum_required(VERSION 2.8.3)
project(au_vision)

## Add '-g' for segmentation fault debugging in gdb
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  roscpp
  rospy
  nodelet
  cv_bridge
  image_transport
  sensor_msgs
  message_generation
  au_core
  actionlib
)

find_package(OpenCV REQUIRED)
# find_package(Boost REQUIRED COMPONENTS system)



generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  sensor_msgs
  au_core
  actionlib
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES au_vision
   CATKIN_DEPENDS actionlib_msgs roscpp rospy std_msgs actionlib au_core
   DEPENDS message_runtime sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

######################
## Camshift Tracker ##
######################

# create test tracker library
add_library( test_tracker src/tracker.cpp src/test_tracker.cpp  )
add_dependencies(
  test_tracker ${PROJECT_NAME}_gencfg
  ${au_core_EXPORTED_TARGETS}
)
target_link_libraries( test_tracker
  ${catkin_LIBRARIES}
)

# create camshift tracker library
add_library( camshift_tracker src/tracker.cpp src/camshift_tracker.cpp  )
add_dependencies(
  camshift_tracker ${PROJECT_NAME}_gencfg
  ${au_core_EXPORTED_TARGETS}
)
target_link_libraries( camshift_tracker
  ${catkin_LIBRARIES}
)

# create bin detector library
add_library( bin_detector src/detector.cpp src/test_detector.cpp src/bin_detector.cpp )
add_dependencies(
  bin_detector ${PROJECT_NAME}_gencfg
  ${au_core_EXPORTED_TARGETS}
)
target_link_libraries( bin_detector
  ${catkin_LIBRARIES}
)

# create naive_buoy detector library
add_library( naive_buoy_detector src/detector.cpp src/test_detector.cpp src/naive_buoy_detector.cpp )
add_dependencies(
  naive_buoy_detector ${PROJECT_NAME}_gencfg
  ${au_core_EXPORTED_TARGETS}
)
target_link_libraries( bin_detector
  ${catkin_LIBRARIES}
)

# create arc_detector library
add_library( arc_detector
  src/arc_detector.cpp
  src/detector.cpp
  src/arc_detector/image.c
  src/arc_detector/pgm.c
  src/arc_detector/misc.c
  src/arc_detector/svg.c
  src/arc_detector/polygon.c
  src/arc_detector/ring.c
  src/arc_detector/elsdc.c
  src/arc_detector/iterator.c
  src/arc_detector/curve_grow.c
  src/arc_detector/ellipse_fit.c
  src/arc_detector/rectangle.c
  src/arc_detector/gauss.c
  src/arc_detector/lapack_wrapper.c
  src/arc_detector/pgm.c
  src/arc_detector/polygon.c
)
target_link_libraries(arc_detector
  vision_bridge
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  lapack
  blas
)

# create test detector library
add_library( test_detector src/detector.cpp src/test_detector.cpp )
add_dependencies(
  test_detector ${PROJECT_NAME}_gencfg
  ${au_core_EXPORTED_TARGETS}
  actionlib_msgs_generate_messages_cpp
)
target_link_libraries( test_detector
  ${catkin_LIBRARIES}
)

# Create tracker node
add_executable( tracker_node src/tracker_node.cpp )
target_link_libraries( tracker_node
  test_tracker
  camshift_tracker
  ${catkin_LIBRARIES}
)

# Create tracker nodelet
add_library( tracker_nodelet src/tracker_nodelet.cpp )
target_link_libraries( tracker_nodelet
  test_tracker
  camshift_tracker
  ${catkin_LIBRARIES}
)

###################
## SURF Detector ##
###################

## Create vision_bridge library
add_library(vision_bridge src/vision_bridge.cpp)
target_link_libraries(vision_bridge
  ${catkin_LIBRARIES}
)

## Create surf_detector library
#add_library(surf_detector
#  src/detector.cpp src/surf_detector/surf_detector.cpp)
#target_link_libraries(surf_detector
#  vision_bridge
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)

## Create surf_trainer executable
#add_executable(surf_trainer
#  src/surf_detector/surf_trainer.cpp)
#target_link_libraries(surf_trainer
#  surf_detector
#  ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
#)

####################
## Detector Nodes ##
####################

# Create Detector node
add_executable( detector_node src/detector_node.cpp)
target_link_libraries( detector_node
  test_detector
  #surf_detector
  arc_detector
  ${catkin_LIBRARIES}
)

# Create detector nodelet
add_library( detector_nodelet src/detector_nodelet.cpp )
target_link_libraries( detector_nodelet
  test_detector
  ${catkin_LIBRARIES}
)

# Create Detector Server
add_executable(detector_server src/detector_server.cpp)
add_dependencies( detector_server std_msgs_generate_messages_cpp )
target_link_libraries(detector_server
  test_detector
  bin_detector
  naive_buoy_detector
  #surf_detector
  arc_detector
  ${catkin_LIBRARIES}
)

# Action server
add_executable(detector_action_server src/detector_action_server.cpp src/detector_action.cpp src/detector.cpp src/test_detector.cpp)
add_dependencies(
  detector_action_server ${PROJECT_NAME}_gencfg
  ${au_core_EXPORTED_TARGETS}
  actionlib_msgs_generate_messages_cpp
)
target_link_libraries(
  detector_action_server
#  surf_detector
  arc_detector
  ${catkin_LIBRARIES}
)

# Test action client
add_executable(test_action_client src/test_action_client.cpp)
add_dependencies(
  test_action_client ${PROJECT_NAME}_gencfg
  ${au_core_EXPORTED_TARGETS}
  actionlib_msgs_generate_messages_cpp
)
target_link_libraries(
  test_action_client
  ${catkin_LIBRARIES}
)
