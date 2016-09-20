cmake_minimum_required(VERSION 2.8.3)
project(semrec_plugins)

find_package(catkin REQUIRED COMPONENTS roscpp roslib rospack designator_integration_cpp designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs semrec)

set(COMPILE_DEBUG 0)

if(CMAKE_COMPILER_IS_GNUCXX)
  if(COMPILE_DEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pg")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2")
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp roslib designator_integration_cpp designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs semrec)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)

set(PLUGINS_SOURCE_DIRECTORY "${PROJECT_SOURCE_DIR}/src/")

add_semrec_plugin(gazebo)
add_semrec_plugin(console)
add_semrec_plugin(experiment_context)
add_semrec_plugin(interactive)
add_semrec_plugin(supervisor)
add_semrec_plugin(imagecapturer)


install(TARGETS sr_plugin_gazebo sr_plugin_console sr_plugin_experiment_context sr_plugin_interactive sr_plugin_imagecapturer sr_plugin_supervisor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
