from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("armando_controller"),
                            "launch",
                            "armando_ik_kdl.launch.py",
                        ]
                    )
                )
            ),
            # TODO(classwork): launch the ArUco detector with the camera topics,
            # selected official dictionary, marker size, and frame parameters.
            # TODO(classwork): launch armando_aruco_target_node and configure
            # its marker input and /target_pose output.
        ]
    )
