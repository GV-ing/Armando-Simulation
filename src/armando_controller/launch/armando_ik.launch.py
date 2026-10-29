import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

     # Get the package share directory
    pkg_share = get_package_share_directory('armando_description')
    xacro_file_name = 'arm.urdf.xacro' 
    xacro_path = os.path.join(pkg_share, 'urdf', xacro_file_name)

    # Wrap the command to force it as a string
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_path]),
        value_type=str
    )

    # Informs ROS 2 that the base_link is located at Z=0.44 relative to the world,
    # exactly where it was spawned in Gazebo
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.44', '0.0', '0.0', '0.0', 'world', 'base_link'],
        output='screen'
    )

    armando_ik_node = Node(
        package='armando_controller',
        executable='armando_ik_node',
        name='armando_ik_node',
        output='screen',
        parameters=[{
            'target_id': -1,
            'robot_description': robot_description_content,
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        static_tf_node,  
        armando_ik_node
    ])