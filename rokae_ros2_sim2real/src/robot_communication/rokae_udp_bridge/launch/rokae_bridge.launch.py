from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # 配置文件路径
    config_dir = os.path.join(get_package_share_directory('rokae_udp_bridge'), 'launch')
    params_file = os.path.join(config_dir, 'config.yaml') #不确定有没有设置上
    
    # 启动参数
    isaac_ip_arg = DeclareLaunchArgument(
        'isaac_ip',
        default_value='192.168.1.29',
        description='Isaac Gym IP address'
    )
    
    isaac_port_arg = DeclareLaunchArgument(
        'isaac_port', 
        default_value='5666',
        description='Isaac Gym port'
    )
    
    local_port_arg = DeclareLaunchArgument(
        'local_port',
        default_value='5006', 
        description='Local UDP port'
    )
    
    joint_count_arg = DeclareLaunchArgument(
        'joint_count',
        default_value='23',
        description='Number of rokae joints'
    )
    
    # UDP Bridge节点
    udp_bridge_node = Node(
        package='rokae_udp_bridge',
        executable='rokae_udp_bridge',
        name='rokae_udp_bridge',
        parameters=[
            params_file,
            {
                'isaac_ip': LaunchConfiguration('isaac_ip'),
                'isaac_port': LaunchConfiguration('isaac_port'),
                'local_port': LaunchConfiguration('local_port'),
                'joint_count': LaunchConfiguration('joint_count'),
            }
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        isaac_ip_arg,
        isaac_port_arg, 
        local_port_arg,
        joint_count_arg,
        udp_bridge_node
    ])