cmake_minimum_required(VERSION 2.8.3)
project(or_velma)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  openrave_catkin
  sensor_msgs
)
find_package(OpenRAVE REQUIRED)

catkin_package()
include_directories(
  include
  ${OpenRAVE_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
    ${OpenRAVE_LIBRARY_DIRS}
)

set(COLLISION_SOURCES
 	src/CollisionCheckerPlugin.cpp
 	src/VelmaQ5Q6CollisionChecker.cpp)

#openrave_plugin(${PROJECT_NAME}
#    ${SOURCES}
#)

openrave_plugin("${PROJECT_NAME}_checker"
    ${COLLISION_SOURCES}
)

#target_link_libraries(${PROJECT_NAME}
#    ${catkin_LIBRARIES}
#    ${OPENRAVE_LIBRARIES}
#)

target_link_libraries("${PROJECT_NAME}_checker"
    ${catkin_LIBRARIES}
    ${OPENRAVE_LIBRARIES}
)
