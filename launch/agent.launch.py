from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    
    return LaunchDescription([
        # Argumentos
        DeclareLaunchArgument(
            'agent_debugger',
            default_value='none',
            description='Debuggers: gdb, valgrind, none'
        ),
        
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
            'type',
            default_value='ACW',
            description='Agent type'
        ),
        
        DeclareLaunchArgument(
            'pose_frame_id',
            default_value='map',
            description='Pose frame ID'
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
            default_value='true',
            description='Use battery faker'
        ),
        
        DeclareLaunchArgument(
            'take_off_height',
            default_value='1',
            description='Take off height'
        ),
        
        DeclareLaunchArgument(
            'distance_error',
            default_value='2',
            description='Distance error'
        ),
        
        DeclareLaunchArgument(
            'goto_error',
            default_value='1',
            description='Go to error'
        ),
        
        DeclareLaunchArgument(
            'world',
            default_value='power_tower',
            description='World name'
        ),
        
        # Nodo agent_behaviour_manager con GDB
        Node(
            package='mission_planner',
            executable='agent_behaviour_manager',
            name='agent_behaviour_manager',
            namespace=[LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
            output='screen',
            parameters=[{
                'id': LaunchConfiguration('id'),
                'ns_prefix': LaunchConfiguration('ns_prefix'),
                'type': LaunchConfiguration('type'),
                'pose_frame_id': LaunchConfiguration('pose_frame_id'),
                'config_file': LaunchConfiguration('config_file'),
                'battery_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/battery_fake'],
                'pose_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/pose'],
                'state_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/state'],
                'take_off_height': LaunchConfiguration('take_off_height'),
                'distance_error': LaunchConfiguration('distance_error'),
                'goto_error': LaunchConfiguration('goto_error')
            }],
            prefix='xterm -e gdb -ex "set print thread-events off" -ex run --args',
            condition=IfCondition(LaunchConfiguration('agent_debugger') == 'gdb')
        ),
        
        # Nodo agent_behaviour_manager con Valgrind
        Node(
            package='mission_planner',
            executable='agent_behaviour_manager',
            name='agent_behaviour_manager',
            namespace=[LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
            output='screen',
            parameters=[{
                'id': LaunchConfiguration('id'),
                'ns_prefix': LaunchConfiguration('ns_prefix'),
                'type': LaunchConfiguration('type'),
                'pose_frame_id': LaunchConfiguration('pose_frame_id'),
                'config_file': LaunchConfiguration('config_file'),
                'battery_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/battery_fake'],
                'pose_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/pose'],
                'state_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/state'],
                'take_off_height': LaunchConfiguration('take_off_height'),
                'distance_error': LaunchConfiguration('distance_error'),
                'goto_error': LaunchConfiguration('goto_error')
            }],
            prefix='valgrind -v --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=/home/baldman/valgrind_agent_',
            condition=IfCondition(LaunchConfiguration('agent_debugger') == 'valgrind')
        ),
        
        # Nodo agent_behaviour_manager sin debugger
        Node(
            package='mission_planner',
            executable='agent_behaviour_manager',
            name='agent_behaviour_manager',
            namespace=[LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
            output='screen',
            parameters=[{
                'id': LaunchConfiguration('id'),
                'ns_prefix': LaunchConfiguration('ns_prefix'),
                'type': LaunchConfiguration('type'),
                'pose_frame_id': LaunchConfiguration('pose_frame_id'),
                'config_file': LaunchConfiguration('config_file'),
                'battery_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/battery_fake'],
                'pose_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/pose'],
                'state_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/state'],
                'take_off_height': LaunchConfiguration('take_off_height'),
                'distance_error': LaunchConfiguration('distance_error'),
                'goto_error': LaunchConfiguration('goto_error')
            }],
            condition=IfCondition(LaunchConfiguration('agent_debugger') == 'none')
        )
    ])