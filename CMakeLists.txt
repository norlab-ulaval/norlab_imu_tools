cmake_minimum_required(VERSION 3.8)
project(norlab_imu_tools)

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
        ${norlab_imu_tools_INCLUDE_DIRS}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

add_executable(imu_bias_observer src/imu_bias_observer.cpp)
ament_target_dependencies(imu_bias_observer
        rclcpp
        rclpy
        std_msgs
        nav_msgs
        sensor_msgs
        tf2_ros
        tf2
        tf2_geometry_msgs
        )
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imu_bias_observer "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)
install(TARGETS imu_bias_observer
        DESTINATION lib/${PROJECT_NAME}
        )

add_executable(imu_and_wheel_odom_node src/imu_and_wheel_odom_node.cpp)
ament_target_dependencies(imu_and_wheel_odom_node
        rclcpp
        rclpy
        std_msgs
        sensor_msgs
        nav_msgs
        tf2_ros
        tf2
        tf2_geometry_msgs
        )
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imu_and_wheel_odom_node "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)
install(TARGETS imu_and_wheel_odom_node
        DESTINATION lib/${PROJECT_NAME}
        )
add_executable(imu_bias_compensator_node src/imu_bias_compensator_node.cpp)
ament_target_dependencies(imu_bias_compensator_node
        rclcpp
        rclpy
        std_msgs
        sensor_msgs
        nav_msgs
        tf2_ros
        tf2
        tf2_geometry_msgs
        )
#rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(imu_bias_compensator_node "${cpp_typesupport_target}")
ament_export_dependencies(rosidl_default_runtime)
install(TARGETS imu_bias_compensator_node
        DESTINATION lib/${PROJECT_NAME}
        )

ament_package()
