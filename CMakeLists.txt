cmake_minimum_required(VERSION 2.8.3)
project(kf_tracker)

set(CMAKE_CXX_FLAGS "-std=c++11 -fpermissive ${CMAKE_CXX_FLAGS} -Wfatal-errors\ ")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  custom_msgs
  std_srvs
  geometry_msgs
)

find_package(Armadillo REQUIRED)

# Initialise the export variables
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES obs_tracker obs_tracker_node
  CATKIN_DEPENDS roscpp std_msgs custom_msgs std_srvs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
  ${ARMADILLO_INCLUDE_DIRS}
  include
)

# Library
add_library(obs_tracker src/obs_tracker.cpp)
target_link_libraries(obs_tracker ${catkin_LIBRARIES}  ${ARMADILLO_LIBRARIES})
add_dependencies(obs_tracker ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})

# Executable
add_executable(obs_tracker_node src/obs_tracker_node.cpp)
target_link_libraries(obs_tracker_node obs_tracker)



install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
# For include dir
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#install(DIRECTORY param
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY maps
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#install(DIRECTORY rviz
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)

#
# Install nodes
#
install(TARGETS obs_tracker_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
