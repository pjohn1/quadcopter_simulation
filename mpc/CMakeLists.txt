cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ACADOS_PATH "/mnt/c/Desktop/quadcopter_simulation/mpc/acados")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo REQUIRED)

add_executable(mpc src/mpc.cpp)
ament_target_dependencies(mpc rclcpp Eigen3)

target_link_libraries(mpc 
    ${ACADOS_PATH}/lib/libacados.so
)

install(TARGETS mpc DESTINATION lib/${PROJECT_NAME})

ament_package()
