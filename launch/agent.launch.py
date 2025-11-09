from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    return LaunchDescription([
        # Argumentos simples
        DeclareLaunchArgument(
            'id',
            default_value=TextSubstitution(text='0'),
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
            'take_off_height',
            default_value=TextSubstitution(text='10.0'),
            description='Take off height'
        ),
        
        DeclareLaunchArgument(
            'distance_error',
            default_value=TextSubstitution(text='2.0'),
            description='Distance error'
        ),
        
        DeclareLaunchArgument(
            'goto_error',
            default_value=TextSubstitution(text='1.0'),
            description='Go to error'
        ),
        
        # Solo un nodo simple para testing
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
                'take_off_height': LaunchConfiguration('take_off_height'),
                'distance_error': LaunchConfiguration('distance_error'),
                'goto_error': LaunchConfiguration('goto_error'),
                'pose_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/pose'],
                'state_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/ual/state'],
                'battery_topic': ['/', LaunchConfiguration('ns_prefix'), LaunchConfiguration('id'), '/battery_fake']
            }]
        )
    ])