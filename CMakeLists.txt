cmake_minimum_required(VERSION 3.0.2)
project(turtlebot_ekf_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  cmake_modules
)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES TurtlebotDataHandler
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(TurtlebotDataHandler
  src/TurtlebotDataHandler.cpp
)


target_link_libraries(TurtlebotDataHandler
  ${catkin_LIBRARIES}
)

add_executable(turtlebot_ekf_localization src/turtlebot_ekf_localization.cpp)
add_dependencies(turtlebot_ekf_localization ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(turtlebot_ekf_localization ${catkin_LIBRARIES} TurtlebotDataHandler)

