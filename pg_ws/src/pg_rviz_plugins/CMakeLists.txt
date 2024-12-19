cmake_minimum_required(VERSION 3.8)
project(pg_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rviz2 REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pg_msgs REQUIRED)
find_package(pg_pump_crane_msgs REQUIRED)
find_package(pg_moisture_sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bagtube_rviz_plugins REQUIRED)

set(dependencies
  rviz_common
  rviz_rendering
  rclcpp
  pg_msgs
  pg_pump_crane_msgs
  pg_moisture_sensor_msgs
  std_srvs
  bagtube_rviz_plugins
)
find_package(Qt5 REQUIRED COMPONENTS Widgets Test)

set(pg_rviz_plugins_headers_to_moc
  include/pg_rviz_plugins/moisture_sensor_panel.hpp
  include/pg_rviz_plugins/pump_crane_panel.hpp
  include/pg_rviz_plugins/pump_crane_joystick.hpp
)

foreach(header "${pg_rviz_plugins_headers_to_moc}")
  qt5_wrap_cpp(pg_rviz_plugins_moc_files "${header}")
endforeach()

set(pg_rviz_plugins_source_files
  src/common.cpp
  src/moisture_sensor_panel.cpp
  src/pump_crane_panel.cpp
  src/pump_crane_joystick.cpp
)

add_library(pg_rviz_plugins SHARED
  ${pg_rviz_plugins_moc_files}
  ${pg_rviz_plugins_source_files}
)
target_compile_features(pg_rviz_plugins PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(pg_rviz_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  ${Qt5Widgets_INCLUDE_DIRS}
)
target_link_libraries(pg_rviz_plugins PUBLIC
  rviz_ogre_vendor::OgreMain
  rviz_ogre_vendor::OgreOverlay
  ignition-math6
)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
ament_target_dependencies(pg_rviz_plugins
  PUBLIC
  ${dependencies}
)

# Export old-style CMake variables
ament_export_include_directories(include)
# Export modern CMake targets
ament_export_targets(pg_rviz_plugins HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${dependencies}
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pg_rviz_plugins PRIVATE "pg_rviz_plugins_BUILDING_LIBRARY")
# prevent pluginlib from using boost
target_compile_definitions(pg_rviz_plugins PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS pg_rviz_plugins
  EXPORT pg_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package(
  CONFIG_EXTRAS "pg_rviz_plugins-extras.cmake"
)
