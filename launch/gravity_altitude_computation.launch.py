from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    formula_arg = DeclareLaunchArgument(
        "formula",
        default_value="hypsometric",
        description="Altitude computation formula to use"
    )

    set_sim_time = SetLaunchConfiguration("use_sim_time", "true")

    nodes = []

    for i in range(1, 5):
        node = Node(
            package="norlab_imu_tools",
            executable="altitude_computation_node",
            name="altitude_computation",
            output="screen",
            remappings=[
                ("ref_pressure_in_setra", "/setra/pressure"),
                ("ref_pressure_in_dps", "/dps310_alti/filtered_pressure_temp_compensated"),
                ("ref_temp_in", "/mcp9808/temperature"),
                ("sensor_pressure_in", f"/dps310_{i}/filtered_pressure_temp_compensated"),
                ("altitude_out", f"/altitude_{i}")
            ],
            parameters=[
                {"formula": LaunchConfiguration("formula")},
                {"use_setra": False},
            ]
        )
        nodes.append(node)

    return LaunchDescription([formula_arg, set_sim_time] + nodes)
