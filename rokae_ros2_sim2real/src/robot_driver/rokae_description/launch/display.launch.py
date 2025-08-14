import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression,PathJoinSubstitution,Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = Path(FindPackageShare('rokae_description').find('rokae_description')) / 'urdf' / 'xMatePro7.urdf'
    robot_description_content = urdf_path.read_text()
    rviz_config_file = LaunchConfiguration('rviz_config', default=PathJoinSubstitution([
        FindPackageShare('rokae_description'), 'launch', 'xMatePro7.rviz'
    ]))

    #declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description = 'Use simulation/Gazebo clock if true'
    )
    #state publisher
    robot_state_publisher_node = Node(
        package= 'robot_state_publisher',
        executable= 'robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        output='screen',
        remappings = [('/joint_states','/joint_state')]

    )

    joint_state_publisher_gui_node = Node(
        package='rokae_driver', 
        executable='My_joint_publisher_gui', 
        output='screen',
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', rviz_config_file]
    )

    robot_controller_node = Node(
        package='rokae_driver',  
        executable='rokae_controller',  
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )


    return LaunchDescription([
        declare_use_sim_time_argument,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        robot_controller_node
    ])