from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os


def generate_launch_description():
    pkg_share = get_package_share_path("project_2")
    default_model_path = pkg_share / "urdf/the_bot.urdf"

    sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Flag to enable use simulation time",
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(default_model_path),
        description="Absolute path to robot urdf file",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            str(pkg_share / "configs/ekf.yaml"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return launch.LaunchDescription(
        [
            sim_time_arg,
            model_arg,
            joint_state_publisher_node,
            robot_state_publisher_node,
            robot_localization_node,
        ]
    )

