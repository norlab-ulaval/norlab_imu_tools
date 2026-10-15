cmake_minimum_required(VERSION 3.8)
project(norlab_imu_tools)

set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

foreach(node imu_bias_observer imu_bias_compensator_node)
    add_executable(${node} src/${node}.cpp)
    ament_target_dependencies(${node}
            rclcpp
            std_msgs
            geometry_msgs
            sensor_msgs
            nav_msgs
            tf2_ros
            tf2
            tf2_geometry_msgs
            )
    install(TARGETS ${node}
            DESTINATION lib/${PROJECT_NAME}
            )
endforeach()

ament_package()
