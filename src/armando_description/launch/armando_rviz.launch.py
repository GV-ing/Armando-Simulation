from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Ottieni il percorso del pacchetto
    pkg_description_path = get_package_share_directory('armando_description')
    use_sim_time = True
    # Percorsi dei file
    urdf_path = os.path.join(pkg_description_path, "urdf", "arm.urdf.xacro")
    rviz_config_path = os.path.join(pkg_description_path, "config", "rviz", "armando_display.rviz")

    # Genera la descrizione del robot processando xacro
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_path,
            ' package_path:=', pkg_description_path
        ]),
        value_type=str
    )

    robot_description_param = {'robot_description': robot_description_content}

    # Nodo: robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}],
    )

    # Nodo: joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Nodo: rviz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
    )

    return LaunchDescription([
        #robot_state_publisher_node,
        #joint_state_publisher_node,
        rviz_node
    ])

