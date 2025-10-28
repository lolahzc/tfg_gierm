from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Obtener el ID como string
    uav_id = LaunchConfiguration('id').perform(context)
    
    nodes = []
    nodes.append(
        Node(
            package='mission_planner',
            executable='agent_behaviour_manager',
            name='agent_behaviour_manager',
            namespace=f'uav{uav_id}',
            output='screen',
            parameters=[{
                'id': uav_id,  # string
                'config_file': LaunchConfiguration('config_file').perform(context),
                'battery_topic': f'/uav{uav_id}/battery_fake',
                'pose_topic': f'/uav{uav_id}/ual/pose', 
                'state_topic': f'/uav{uav_id}/ual/state',
                'take_off_height': 1.0,  # float
                'distance_error': 2.0,   # float
                'goto_error': 1.0        # float
            }]
        )
    )
    return nodes

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'id',
            default_value='1',
            description='UAV ID'
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
        
        OpaqueFunction(function=launch_setup)
    ])