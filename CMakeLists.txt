cmake_minimum_required(VERSION 3.0.2)
project(qr_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(
  Eigen3 REQUIRED
)

set(acados_include "/home/yang/acados/include")
set(acados_lib "/home/yang/acados/lib")
set(quadrotor_model ${PROJECT_SOURCE_DIR}/scripts/c_generated_code)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qr_mpc
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
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
