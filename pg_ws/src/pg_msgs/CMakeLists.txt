cmake_minimum_required(VERSION 3.8)
project(pg_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_link_options(-z defs)  # linker check for shared libraries
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

set(dependencies
  builtin_interfaces
  std_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoolStamped.msg"
  "msg/Float64ArrayStamped.msg"
  "msg/Float64Stamped.msg"
  "msg/MeasurementHeader.msg"
  "msg/Device.msg"
  "msg/Job.msg"
  "srv/EditDevice.srv"
  "srv/EditJob.srv"
  "srv/GetDeviceList.srv"
  "srv/GetJobList.srv"
  "srv/GetState.srv"
  DEPENDENCIES ${dependencies}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies} rosidl_default_generators rosidl_default_runtime)
ament_package()
