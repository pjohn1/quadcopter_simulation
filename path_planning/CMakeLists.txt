cmake_minimum_required(VERSION 3.8)
project(path_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(initialize_occupancy_grid src/initialize_occupancy_grid.cpp)
ament_target_dependencies(initialize_occupancy_grid rclcpp sensor_msgs Eigen3)

add_executable(remap_pcd src/remap_pcd.cpp)
ament_target_dependencies(remap_pcd rclcpp sensor_msgs Eigen3)

add_executable(visualize_free_space src/visualize_free_space.cpp)
ament_target_dependencies(visualize_free_space rclcpp visualization_msgs geometry_msgs Eigen3)

add_executable(path_plan_main src/path_plan_main.cpp)
ament_target_dependencies(path_plan_main rclcpp visualization_msgs geometry_msgs Eigen3)


install(TARGETS initialize_occupancy_grid DESTINATION lib/${PROJECT_NAME})
install(TARGETS remap_pcd DESTINATION lib/${PROJECT_NAME})
install(TARGETS visualize_free_space DESTINATION lib/${PROJECT_NAME})
install(TARGETS path_plan_main DESTINATION lib/${PROJECT_NAME})

ament_package()
