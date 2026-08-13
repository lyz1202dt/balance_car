cmake_minimum_required(VERSION 3.8)
project(controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)
find_package(autodiff REQUIRED)

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

pluginlib_export_plugin_description_file(controller_interface plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

add_library(lqr_controller SHARED
  src/controller/controller.cpp
  src/controller/controller_params.cpp
  src/controller/leg_calc.cpp
  src/controller/lqr_gain_scheduler.cpp
  src/controller/mujoco_sim.cpp
  src/controller/real_hardware.cpp
  src/tools/leg_calc_base.cpp
  src/tools/pid.cpp
)

target_include_directories(lqr_controller PUBLIC
${EIGEN3_INCLUDE_DIRS}
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>
)

target_link_libraries(lqr_controller Eigen3::Eigen autodiff::autodiff)

ament_target_dependencies(
  lqr_controller
  rclcpp
  controller_interface
  pluginlib
  hardware_interface
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  robot_interfaces
  urdf
)

install(TARGETS lqr_controller
LIBRARY DESTINATION lib)

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
