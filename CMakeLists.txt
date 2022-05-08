cmake_minimum_required(VERSION 2.8.3)
project(rgbd_risk)

## Compile as C++11, supported in ROS Kinetic and newer
 #add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  pcl_conversions
  pcl_ros roscpp
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)
find_package(ZED 3 REQUIRED)
find_package(OpenCV REQUIRED)
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES rgbd_risk
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if (LINK_SHARED_ZED)
  SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
  SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  # ${ZED_LIBS} # add zed libs
  # ${OpenCV_LIBRARIES} 
  ${ZED_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  pcl_visualization
)

add_executable(rgbd_node src/RGBD_node.cpp src/RGBD.cpp)
target_link_libraries(rgbd_node 
                  ${catkin_LIBRARIES} ${PCL_LIBRARIES}
                  ${ZED_LIBS}
                  ${OpenCV_LIBRARIES}  )
add_dependencies(rgbd_node ${catkin_EXPORTED_TARGETS})

