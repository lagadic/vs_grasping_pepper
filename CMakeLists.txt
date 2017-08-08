cmake_minimum_required(VERSION 2.8.3)
project(vs_grasping_pepper)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  naoqi_libqi
  naoqi_libqicore
  visp_bridge
  image_transport
  cv_bridge
  whycon
)

find_package(VISP REQUIRED)

add_definitions(-DLIBQI_VERSION=${naoqi_libqi_VERSION_MAJOR}${naoqi_libqi_VERSION_MINOR})

find_package(VISP_NAOQI REQUIRED )

catkin_package(
  
 
  LIBRARIES
  vs_grasping_pepper
  
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf
  image_transport
  cv_bridge

  DEPENDS 
  VISP
  VISP_NAOQI
  visp_bridge
)

include_directories(include)


include_directories(SYSTEM
  ${VISP_INCLUDE_DIRS}
  ${visp_naoqi_INCLUDE_DIRS}
  ${NAOqi_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}

)

## Make sure Boost.Filesystem v3 is used.
add_definitions(-DBOOST_FILESYSTEM_VERSION=3)

# Add package definitions
add_definitions(${VISP_DEFINITIONS})

# Declare a cpp library
add_library(vs_grasping_pepper
  include/vs_grasping_pepper.h
  include/recorded_motion.h
  src/vs_grasping_pepper.cpp
)


target_link_libraries(vs_grasping_pepper
  ${catkin_LIBRARIES}
  ${VISP_LIBRARIES}
  ${naoqi_libqi_LIBRARIES}
  ${visp_naoqi_LIBRARIES}
)

## Declare a cpp executable
#add_executable(vs_grasping_pepper_node src/vs_grasping_pepper.cpp)
add_executable(vs_grasping_pepper_node src/main.cpp)


  target_link_libraries(vs_grasping_pepper_node
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    vs_grasping_pepper
    )


