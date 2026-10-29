from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("armando_description")
    xacro_path = PathJoinSubstitution(
        [description_share, "urdf", "arm.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_path,
                " package_path:=",
                description_share,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_id", default_value="0"),
            DeclareLaunchArgument(
                "marker_topic",
                default_value="/aruco_marker_publisher/markers",
            ),
            DeclareLaunchArgument("target_pose_topic", default_value="/target_pose"),
            DeclareLaunchArgument(
                "target_normal_offset",
                default_value="0.1",
                description="Offset in meters along the marker's positive local Z axis.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="armando_controller",
                executable="armando_ik_kdl_node",
                name="armando_ik_kdl_node",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
            Node(
                package="armando_controller",
                executable="armando_aruco_target_node",
                name="armando_aruco_target_node",
                output="screen",
                parameters=[
                    {
                        "target_id": LaunchConfiguration("target_id"),
                        "marker_topic": LaunchConfiguration("marker_topic"),
                        "target_pose_topic": LaunchConfiguration("target_pose_topic"),
                        "target_normal_offset": LaunchConfiguration("target_normal_offset"),
                        "use_sim_time": LaunchConfiguration("use_sim_time"),
                    }
                ],
            ),
        ]
    )
