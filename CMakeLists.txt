cmake_minimum_required(VERSION 3.0.2)
project(qr_mpc)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

find_package(
  Eigen3 REQUIRED
)

set(acados_include "/home/rocky/acados/include")
set(acados_lib "/home/rocky/acados/lib")
set(quadrotor_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

add_message_files(
  FILES
  TakeoffLandTrigger.msg
  AiroPx4.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qr_mpc
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${quadrotor_model}
  ${acados_include}
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/acados/
  ${acados_include}/qpOASES_e/
)

link_directories(${quadrotor_model})

add_executable(qr_mpc_node src/qr_mpc_node.cpp)
target_link_libraries(qr_mpc_node
  ${quadrotor_model}/libacados_ocp_solver_quadrotor.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)

add_executable(airo_px4_node
  src/airo_px4_node.cpp
  src/airo_px4_fsm.cpp
  src/quadrotor_mpc.cpp
  src/rc_input.cpp
)

target_link_libraries(airo_px4_node
  ${quadrotor_model}/libacados_ocp_solver_quadrotor.so
  ${acados_lib}/libacados.so
  ${catkin_LIBRARIES}
)

add_executable(system_identification src/system_identification.cpp)
target_link_libraries(system_identification
  ${catkin_LIBRARIES}
)

add_executable(lala src/lala.cpp)
target_link_libraries(lala
  ${catkin_LIBRARIES}
)