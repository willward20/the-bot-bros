from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_share = get_package_share_path("project_2")
    default_model_path = pkg_share / "urdf/the_bot.urdf"
    rplidar_pkg = get_package_share_path("rplidar_ros2")

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

    odom_pub_node = Node(
        package="project_2",
        executable="odom_publisher",
        name="odom_publisher",
    )

    imu_pub_node = Node(
        package="project_2",
        executable="imu_publisher",
        name="imu_pub",
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

    launch_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(rplidar_pkg / "launch/rplidar_launch.py")
        ),
        launch_arguments={
            "frame_id": "lidar_link"
        }.items()
    )

    return launch.LaunchDescription(
        [
            sim_time_arg,
            model_arg,
            odom_pub_node,
            joint_state_publisher_node,
            robot_state_publisher_node,
            robot_localization_node,
            launch_rplidar,
        ]
    )

