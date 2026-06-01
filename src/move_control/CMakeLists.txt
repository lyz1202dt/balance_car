cmake_minimum_required(VERSION 3.8)
project(move_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

find_package(Eigen3 REQUIRED PATHS /usr/include/eigen3)

# Use pkg-config to find pinocchio
find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

# Try to find casadi
find_package(casadi QUIET)
if(NOT casadi_FOUND)
  pkg_check_modules(casadi REQUIRED casadi)
endif()

set(ACADOS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../acados)
set(NMPC_CODEGEN_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/model/car_nmpc_codegen.py)
set(NMPC_MODEL_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/model/car_nmpc_model.py)
set(NMPC_URDF ${CMAKE_CURRENT_SOURCE_DIR}/../car/model/car.urdf)
set(NMPC_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/car_nmpc)
set(NMPC_GENERATION_STAMP ${NMPC_GENERATED_DIR}/generation.stamp)
set(NMPC_GENERATED_MAIN_HEADER ${NMPC_GENERATED_DIR}/acados_solver_balance_car_full_state.h)
set(NMPC_GENERATED_MAIN_SOURCE ${NMPC_GENERATED_DIR}/acados_solver_balance_car_full_state.c)
set(NMPC_GENERATED_MODEL_HEADER ${NMPC_GENERATED_DIR}/balance_car_full_state_model/balance_car_full_state_model.h)
set(NMPC_GENERATED_MODEL_ODE ${NMPC_GENERATED_DIR}/balance_car_full_state_model/balance_car_full_state_expl_ode_fun.c)
set(NMPC_GENERATED_MODEL_VDE_FORW ${NMPC_GENERATED_DIR}/balance_car_full_state_model/balance_car_full_state_expl_vde_forw.c)
set(NMPC_GENERATED_MODEL_VDE_ADJ ${NMPC_GENERATED_DIR}/balance_car_full_state_model/balance_car_full_state_expl_vde_adj.c)
set(NMPC_GENERATED_COST_DIR ${NMPC_GENERATED_DIR}/balance_car_full_state_cost)
set(NMPC_GENERATED_CONSTRAINT_DIR ${NMPC_GENERATED_DIR}/balance_car_full_state_constraints)
set(NMPC_GENERATED_COST_HEADER ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost.h)
set(NMPC_GENERATED_CONSTRAINT_HEADER ${NMPC_GENERATED_CONSTRAINT_DIR}/balance_car_full_state_constraints.h)
set(NMPC_GENERATED_COST_SOURCES
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_0_fun.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_0_fun_jac_ut_xt.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_0_hess.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_fun.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_fun_jac_ut_xt.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_hess.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_e_fun.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_e_fun_jac_ut_xt.c
  ${NMPC_GENERATED_COST_DIR}/balance_car_full_state_cost_y_e_hess.c
)
set(NMPC_GENERATED_CONSTRAINT_SOURCES
  ${NMPC_GENERATED_CONSTRAINT_DIR}/balance_car_full_state_constr_h_fun.c
  ${NMPC_GENERATED_CONSTRAINT_DIR}/balance_car_full_state_constr_h_fun_jac_uxt_zt.c
  ${NMPC_GENERATED_CONSTRAINT_DIR}/balance_car_full_state_constr_h_e_fun.c
  ${NMPC_GENERATED_CONSTRAINT_DIR}/balance_car_full_state_constr_h_e_fun_jac_uxt_zt.c
)

add_custom_command(
  OUTPUT
    ${NMPC_GENERATION_STAMP}
    ${NMPC_GENERATED_MAIN_HEADER}
    ${NMPC_GENERATED_MAIN_SOURCE}
    ${NMPC_GENERATED_MODEL_HEADER}
    ${NMPC_GENERATED_MODEL_ODE}
    ${NMPC_GENERATED_MODEL_VDE_FORW}
    ${NMPC_GENERATED_MODEL_VDE_ADJ}
    ${NMPC_GENERATED_COST_HEADER}
    ${NMPC_GENERATED_CONSTRAINT_HEADER}
    ${NMPC_GENERATED_COST_SOURCES}
    ${NMPC_GENERATED_CONSTRAINT_SOURCES}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${NMPC_GENERATED_DIR}
  COMMAND ${CMAKE_COMMAND} -E env
    ACADOS_SOURCE_DIR=${ACADOS_ROOT}
    LD_LIBRARY_PATH=${ACADOS_ROOT}/lib:$ENV{LD_LIBRARY_PATH}
    ${Python3_EXECUTABLE} ${NMPC_CODEGEN_SCRIPT}
    --urdf ${NMPC_URDF}
    --export-dir ${NMPC_GENERATED_DIR}
    --json ${NMPC_GENERATED_DIR}/acados_ocp.json
  COMMAND ${CMAKE_COMMAND} -E touch ${NMPC_GENERATION_STAMP}
  DEPENDS
    ${NMPC_CODEGEN_SCRIPT}
    ${NMPC_MODEL_SCRIPT}
    ${NMPC_URDF}
  COMMENT "Generating balance car NMPC C code"
  VERBATIM
)

add_custom_target(generate_balance_car_nmpc DEPENDS ${NMPC_GENERATION_STAMP})

add_executable(move_control
src/main.cpp
src/controller.cpp
src/nmpc_solver.cpp
${NMPC_GENERATED_MAIN_SOURCE}
${NMPC_GENERATED_MODEL_ODE}
${NMPC_GENERATED_MODEL_VDE_FORW}
${NMPC_GENERATED_MODEL_VDE_ADJ}
${NMPC_GENERATED_COST_SOURCES}
${NMPC_GENERATED_CONSTRAINT_SOURCES}
)

add_dependencies(move_control generate_balance_car_nmpc)

target_include_directories(move_control
  PUBLIC
    ${EIGEN3_INCLUDE_DIRS}
    ${pinocchio_INCLUDE_DIRS}
    ${casadi_INCLUDE_DIRS}
    ${ACADOS_ROOT}/include
    ${ACADOS_ROOT}/include/blasfeo/include
    ${ACADOS_ROOT}/include/hpipm/include
    ${NMPC_GENERATED_DIR}
    ${NMPC_GENERATED_DIR}/balance_car_full_state_model
    ${NMPC_GENERATED_COST_DIR}
    ${NMPC_GENERATED_CONSTRAINT_DIR}
    /opt/openrobots/include
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(move_control 
  Eigen3::Eigen 
  ${pinocchio_LDFLAGS}
  ${casadi_LDFLAGS}
  ${ACADOS_ROOT}/lib/libacados.so
  ${ACADOS_ROOT}/lib/libhpipm.so
  ${ACADOS_ROOT}/lib/libblasfeo.so
  ${ACADOS_ROOT}/lib/libqpOASES_e.so
)

set_target_properties(move_control PROPERTIES
  BUILD_RPATH "${ACADOS_ROOT}/lib"
  INSTALL_RPATH "${ACADOS_ROOT}/lib"
)

# Add flags from pkg-config
add_compile_options(${pinocchio_CFLAGS})
target_compile_options(move_control PUBLIC ${casadi_CFLAGS})

ament_target_dependencies(
  move_control
  rclcpp
  geometry_msgs
  sensor_msgs
  visualization_msgs
  tf2
  tf2_ros
  robot_interfaces
  pinocchio
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

install(TARGETS
move_control
DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config rviz 
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
