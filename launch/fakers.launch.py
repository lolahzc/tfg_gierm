from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'id',
            default_value='0',
            description='Agent ID'
        ),
        
        DeclareLaunchArgument(
            'ns_prefix',
            default_value='uav',
            description='Namespace prefix'
        ),
        
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mission_planner'),
                'config',
                'conf.yaml'
            ]),
            description='Path to config file'
        ),
        
        DeclareLaunchArgument(
            'battery_faker',
            default_value='false',
            description='Use battery faker'
        ),
        
        DeclareLaunchArgument(
            'battery_mode',
            default_value='recharge_in_base',
            description='Battery mode: static, only_discharge, recharge_in_base, recharge_anywhere'
        ),
        
        DeclareLaunchArgument(
            'ist_ugv_faker',
            default_value='false',
            description='Use IST UGV faker'
        ),
        
        # Battery faker
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mission_planner'),
                    'launch',
                    'battery_faker.launch.py'
                ])
            ]),
            launch_arguments={
                'id': LaunchConfiguration('id'),
                'ns_prefix': LaunchConfiguration('ns_prefix'),
                'config_file': LaunchConfiguration('config_file'),
                'battery_mode': LaunchConfiguration('battery_mode'),
                'debugger': 'none'
            }.items(),
            condition=IfCondition(LaunchConfiguration('battery_faker'))
        ),
        
        # IST UGV faker
        Node(
            package='mission_planner',
            executable='ist_ugv_faker',
            name='ist_ugv_faker',
            output='screen',
            condition=IfCondition(LaunchConfiguration('ist_ugv_faker'))
        )
    ])