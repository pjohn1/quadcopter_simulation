cmake_minimum_required(VERSION 3.8)
project(quad_model)

set(CMAKE_C_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_COMPILER_LAUNCHER ccache)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
	  DIRECTORY launch/
	    DESTINATION share/quad_model/launch)

install(DIRECTORY meshes/
      DESTINATION share/${PROJECT_NAME}/meshes
    )

install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
  
# find dependencies
# find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(physics_sim sim/physics_sim.cpp)
target_include_directories(physics_sim PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(physics_sim ${GAZEBO_LIBRARIES})
ament_target_dependencies(physics_sim rclcpp geometry_msgs tf2_ros)

add_executable(physics_node sim/physics_node.cpp)
ament_target_dependencies(physics_node rclcpp std_msgs geometry_msgs Eigen3)

add_executable(velocity_controller controllers/velocity_controller.cpp)
ament_target_dependencies(velocity_controller rclcpp std_msgs geometry_msgs Eigen3)

add_executable(controller_node controllers/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp std_msgs geometry_msgs Eigen3)

install(TARGETS physics_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS physics_sim DESTINATION lib/${PROJECT_NAME})
install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})
install(TARGETS velocity_controller DESTINATION lib/${PROJECT_NAME})

ament_package()