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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(graph_slam_node
  src/graph_slam.cpp
  src/associationSolver.cpp
  src/types_graph_slam.cpp
  src/map_manager.cpp
)

target_include_directories(graph_slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${YAML_CPP_INCLUDE_DIRS}
)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../lart_common
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(graph_slam_node
  rclcpp
  lart_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  yaml-cpp
)

target_link_libraries(graph_slam_node
  fmt::fmt
  g2o_core
  g2o_types_slam2d
  g2o_solver_eigen
  g2o_stuff
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ament_index_cpp::ament_index_cpp
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

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/../maps/
  DESTINATION share/${PROJECT_NAME}/maps
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
