from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_params = PathJoinSubstitution([
        FindPackageShare("my_epuck_project"),
        "resource",
        "ekf_odom_imu_tf.yaml",
    ])

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
    )

    return LaunchDescription([
        ekf_node,
    ])
