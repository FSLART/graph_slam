cmake_minimum_required(VERSION 3.8)
project(graph_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lart_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(g2o REQUIRED)
find_package(fmt REQUIRED)
find_package(PCL REQUIRED COMPONENTS registration common)

set(G2O_INCREMENTAL_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../interactive_slam/g2o_incremental)
set(G2O_INTERACTIVE_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../interactive_slam/g2o_interactive)
set(INTERACTIVE_SLAM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../interactive_slam)

add_executable(graph_slam_node
  src/graph_slam.cpp
  src/associationSolver.cpp
  src/types_graph_slam.cpp
  src/slamInterface.cpp

  # incremental optimizer implementation (keep)
  ${G2O_INCREMENTAL_DIR}/graph_optimizer_sparse_incremental.cpp

  # NOTE: do NOT compile ${G2O_INCREMENTAL_DIR}/g2o_incremental.cpp (it is an example main)
)

target_include_directories(graph_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>

  # local copied example headers
  ${G2O_INCREMENTAL_DIR}
  ${G2O_INTERACTIVE_DIR}

  # so includes like "slam_parser/interface/abstract_slam_interface.h" resolve
  ${INTERACTIVE_SLAM_DIR}
)

# If your local g2o_interactive folder has these .cpp files, compile them too
foreach(_src
  graph_optimizer_sparse_online.cpp
  types_slam2d_online.cpp
)
  if(EXISTS "${G2O_INTERACTIVE_DIR}/${_src}")
    target_sources(graph_slam_node PRIVATE "${G2O_INTERACTIVE_DIR}/${_src}")
  endif()
endforeach()

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../lart_common
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(graph_slam_node
  rclcpp
  lart_msgs
  geometry_msgs
)

target_link_libraries(graph_slam_node
  fmt::fmt
  g2o_core
  g2o_types_slam2d
  g2o_solver_eigen
  g2o_stuff
  ${PCL_LIBRARIES}
)

# --- SuiteSparse / CHOLMOD (required by g2o_incremental) ---
find_path(SUITESPARSE_INCLUDE_DIR suitesparse/camd.h)
if(NOT SUITESPARSE_INCLUDE_DIR)
  message(FATAL_ERROR "SuiteSparse headers not found (expected suitesparse/camd.h). Install libsuitesparse-dev.")
endif()

find_library(CHOLMOD_LIBRARY cholmod)
find_library(AMD_LIBRARY amd)
find_library(CAMD_LIBRARY camd)
find_library(COLAMD_LIBRARY colamd)
find_library(CCOLAMD_LIBRARY ccolamd)
find_library(SUITESPARSECONFIG_LIBRARY suitesparseconfig)

target_include_directories(graph_slam_node PUBLIC
  ${SUITESPARSE_INCLUDE_DIR}
)

target_link_libraries(graph_slam_node
  ${CHOLMOD_LIBRARY}
  ${AMD_LIBRARY}
  ${CAMD_LIBRARY}
  ${COLAMD_LIBRARY}
  ${CCOLAMD_LIBRARY}
  ${SUITESPARSECONFIG_LIBRARY}
)

install(TARGETS
  graph_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY ../launch/
  DESTINATION share/${PROJECT_NAME}/launch
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

ament_package()
