from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("radarays_gazebo_plugins")
    worlds_dir = PathJoinSubstitution([package_share, "worlds"])
    default_world = PathJoinSubstitution([worlds_dir, "gz_dynamic_radar_cpu.sdf"])

    resource_path = [
        worlds_dir,
        ":",
        EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("verbosity", default_value="4"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            ExecuteProcess(
                cmd=[
                    "gz",
                    "sim",
                    "-v",
                    LaunchConfiguration("verbosity"),
                    LaunchConfiguration("world"),
                ],
                output="screen",
            ),
        ]
    )
