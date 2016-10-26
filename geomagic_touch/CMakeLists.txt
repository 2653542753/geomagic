cmake_minimum_required(VERSION 2.8.3)
project(geomagic_touch)

# Use C++11
if(${CMAKE_VERSION} VERSION_LESS "3.1.0")
  if(CMAKE_COMPILER_IS_GNUCXX)
    include(CheckCXXCompilerFlag)
    CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
    if(COMPILER_SUPPORTS_CXX11)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    else()
      message(WARNING "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.")
    endif()
  endif()
else()
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  sensor_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_message_files(
  DIRECTORY msg
  FILES PhantomButtonEvent.msg OmniFeedback.msg
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(omni_node src/omni.cpp)
add_dependencies(omni_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(omni_node ${catkin_LIBRARIES} HD HDU)
