from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable
from launch_ros.actions import Node
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

    # rmagine's auto-downloaded Embree binary needs libiomp5.so at dlopen
    # time inside gz-sim; this machine only has a libgomp.so.1-based
    # symlink shim at /usr/local/lib/libiomp5.so (see MIGRATION_HANDOFF.md,
    # "Known Environment Issues"). ldconfig's cache can't index it under
    # the name "libiomp5.so" (it indexes by the target's own embedded
    # SONAME, libgomp.so.1) -- confirmed empirically that ldconfig alone
    # is not enough, LD_LIBRARY_PATH is what actually makes gz-sim's
    # dlopen find it. Without this, rmagine_embree_map_system fails to
    # load silently (readable in the console output, easy to miss) and
    # every sensor topic gets advertised but never publishes real data.
    ld_library_path = [
        "/usr/local/lib",
        ":",
        EnvironmentVariable("LD_LIBRARY_PATH", default_value=""),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("verbosity", default_value="4"),
            SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", resource_path),
            SetEnvironmentVariable("LD_LIBRARY_PATH", ld_library_path),
            ExecuteProcess(
                cmd=[
                    "gz",
                    "sim",
                    "-v",
                    LaunchConfiguration("verbosity"),
                    # -r: start running immediately. Every sensor System here
                    # (radarays_embree_sensor_system, rmagine_embree_sensor_system,
                    # etc) checks _info.paused and skips publishing while paused --
                    # gz sim loads paused by default without this flag, which looks
                    # exactly like a broken pipeline (no /radar/image, no error) if
                    # you don't notice the GUI's pause button.
                    "-r",
                    LaunchConfiguration("world"),
                ],
                output="screen",
            ),
            # rmagine_gazebo_plugins's rmagine_embree_sensor_system (the
            # generic /radar/scan+/radar/points path -- NOT
            # radarays_embree_sensor_system, which still publishes
            # /radar/image directly via an embedded rclcpp node) publishes
            # over native gz-transport only since amock's rewrite, not
            # rclcpp directly -- this bridge is what actually gets those
            # two topics into ROS 2.
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[
                    "radar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
                    "radar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
                ],
                output="screen",
            ),
        ]
    )
