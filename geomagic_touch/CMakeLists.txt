cmake_minimum_required(VERSION 3.1)
project(geomagic_touch)

# Use C++14
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_message_files(
  DIRECTORY msg
  FILES TouchState.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(omni_node src/omni.cpp)
add_dependencies(omni_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_node ${catkin_LIBRARIES} HD HDU)
