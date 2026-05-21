from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("gici_ros")

    config_file = LaunchConfiguration("config_file")
    rviz_config = LaunchConfiguration("rviz_config")

    default_config_file = PathJoinSubstitution(
        [package_share, "option", "ros_real_time_estimation_RTK_RRR.yaml"]
    )
    default_rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "gici_gic.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="GICI YAML config file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 config file.",
            ),
            Node(
                package="gici_ros",
                executable="gici_ros_main",
                name="gici",
                output="screen",
                arguments=[config_file],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
