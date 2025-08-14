from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration, PythonExpression,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        #realsense 相机
        IncludeLaunchDescription (
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'enable_color': 'true',
                # 'enable_depth': 'false',
                # 'color_width': '1280',
                # 'color_height': '720',
                # 'color_fps': '30'
            }.items()
        ),
        #ArUco 标记
        Node(
            package='aruco_ros',
            executable= 'single',
            name= 'aruco_single',
            parameters= [{
                'camera_frame': 'camera_color_frame',
                'marker_frame': 'aruco_marker_frame',
                'marker_id': 26,
                'marker_size': 0.05,#5cm
                'reference_frame': 'camera_color_frame',
                
            }],
            remappings=[
                ('image','/camera/camera/color/image_raw'),
                ('camera_info','/camera/camera/color/camera_info')
            ]
        ),
        #Rokae display
        IncludeLaunchDescription (
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('rokae_description'),
                        'launch',
                        'display.launch.py'
                    ])
                ]),
        ),
        #手眼标定
        IncludeLaunchDescription (
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('easy_handeye2'),
                    'launch',
                    'calibrate.launch.py'
                ])
            ]),
            launch_arguments={
                'calibration_type': 'eye_on_base',
                'name': 'my_eob_calib',
                'robot_base_frame': 'xMatePro7_base',
                'robot_effector_frame': 'block_link',
                'tracking_base_frame': 'camera_color_frame',
                'tracking_marker_frame': 'aruco_marker_frame',
            }.items()
        ),


        
    ])
        