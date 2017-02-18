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
find_package(Boost REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_message_files(
  DIRECTORY msg
  FILES ButtonEvent.msg
)

add_service_files(
  DIRECTORY srv
  FILES SetForceOutput.srv
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

if(CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif()

add_executable(geotouch_node src/geotouch_node.cpp)
add_dependencies(geotouch_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(geotouch_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} HD)
