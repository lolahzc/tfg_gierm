from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

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
            'debugger',
            default_value='none',
            description='Debuggers: gdb, none'
        ),
        
        DeclareLaunchArgument(
            'battery_mode',
            default_value='recharge_in_base',
            description='Battery mode: static, only_discharge, recharge_in_base, recharge_anywhere'
        ),
        
        # Nodo battery_faker sin debugger
        Node(
            package='mission_planner',
            executable='battery_faker',
            name='battery_faker',
            namespace=[LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
            output='screen',
            parameters=[{
                'id': [LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
                'config_file': LaunchConfiguration('config_file'),
                'battery_mode': LaunchConfiguration('battery_mode'),
                'pose_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/pose'],
                'state_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/state']
            }],
            condition=IfCondition(LaunchConfiguration('debugger') == 'none')
        ),
        
        # Nodo battery_faker con GDB
        Node(
            package='mission_planner',
            executable='battery_faker',
            name='battery_faker',
            namespace=[LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
            output='screen',
            parameters=[{
                'id': [LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
                'config_file': LaunchConfiguration('config_file'),
                'battery_mode': LaunchConfiguration('battery_mode'),
                'pose_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/pose'],
                'state_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/state']
            }],
            prefix='xterm -e gdb -ex "set print thread-events off" -ex run --args',
            condition=IfCondition(LaunchConfiguration('debugger') == 'gdb')
        )
    ])