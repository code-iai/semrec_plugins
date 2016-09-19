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
  LIBRARIES sr_plugin_gazebo sr_plugin_console sr_plugin_experiment_context sr_plugin_interactive sr_plugin_supervisor sr_plugin_dotexporter
  CATKIN_DEPENDS roscpp roslib designator_integration_cpp designator_integration_msgs sensor_msgs cv_bridge interactive_markers visualization_msgs semrec
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include)


add_library(sr_plugin_gazebo SHARED
  src/gazebo/PluginGazebo.cpp)

add_library(sr_plugin_console SHARED
  src/console/PluginConsole.cpp)

add_library(sr_plugin_experiment_context SHARED
  src/experiment-context/PluginExperimentContext.cpp)

add_library(sr_plugin_interactive SHARED
  src/interactive/PluginInteractive.cpp
  src/interactive/InteractiveObject.cpp)

add_library(sr_plugin_supervisor SHARED
  src/supervisor/PluginSupervisor.cpp)

add_library(sr_plugin_imagecapturer SHARED
  src/imagecapturer/CImageCapturer.cpp
  src/imagecapturer/PluginImageCapturer.cpp)

target_link_libraries(sr_plugin_gazebo
  sr_base_plugin)

target_link_libraries(sr_plugin_console
  sr_base_plugin
  ncurses)

target_link_libraries(sr_plugin_experiment_context
  sr_exporter_plugin)

target_link_libraries(sr_plugin_interactive
  sr_base_plugin)

target_link_libraries(sr_plugin_imagecapturer
  sr_base_plugin)

target_link_libraries(sr_plugin_supervisor
  sr_base_plugin)

install(TARGETS sr_plugin_gazebo sr_plugin_console sr_plugin_experiment_context sr_plugin_interactive sr_plugin_imagecapturer sr_plugin_supervisor
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
