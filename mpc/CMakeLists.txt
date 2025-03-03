cmake_minimum_required(VERSION 3.8)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

install(
	  DIRECTORY launch/
	    DESTINATION share/mpc/launch)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(mpc src/mpc.cpp)
ament_target_dependencies(mpc rclcpp Eigen3 std_msgs Python3 geometry_msgs)

target_include_directories(mpc PRIVATE ${Python3_INCLUDE_DIRS})

install(TARGETS mpc DESTINATION lib/${PROJECT_NAME})

ament_package()
