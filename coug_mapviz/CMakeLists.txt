cmake_minimum_required(VERSION 3.22)
project(coug_mapviz)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies ---
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Qt5 REQUIRED COMPONENTS Core Gui OpenGL Widgets)
find_package(OpenGL REQUIRED)

# --- Qt Setup ---
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui")

# --- Plugin Library ---
ament_auto_add_library(${PROJECT_NAME} SHARED NO_TARGET_LINK_LIBRARIES
  include/${PROJECT_NAME}/coug_waypoints_plugin.hpp
  src/coug_waypoints_plugin.cpp
  src/utils/agent_interface.cpp
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  "PLUGINLIB__DISABLE_BOOST_FUNCTIONS"
  "WFlags=WindowFlags"
)

target_link_libraries(${PROJECT_NAME}
  Qt5::Core
  Qt5::Gui
  Qt5::OpenGL
  Qt5::Widgets
  OpenGL::GL
)

# --- Export ---
pluginlib_export_plugin_description_file(mapviz mapviz_plugins.xml)

ament_export_dependencies(Qt5)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
