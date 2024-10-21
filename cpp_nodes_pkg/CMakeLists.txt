cmake_minimum_required(VERSION 3.8)
project(cpp_final_pro_udemy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(my_robot_interfaces REQUIRED)

add_executable(controller src/controller.cpp)
ament_target_dependencies(controller rclcpp geometry_msgs turtlesim my_robot_interfaces)

add_executable(spawner src/spawner.cpp)
ament_target_dependencies(spawner rclcpp geometry_msgs turtlesim my_robot_interfaces)

install(TARGETS
        controller
        spawner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
