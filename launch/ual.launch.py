from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'ns_prefix',
            default_value='uav',
            description='Namespace prefix'
        ),
        
        DeclareLaunchArgument(
            'id',
            default_value='0',
            description='UAV ID'
        ),
        
        DeclareLaunchArgument(
            'robot_model',
            default_value='mbzirc',
            description='Robot model'
        ),
        
        DeclareLaunchArgument(
            'x',
            default_value='0',
            description='Initial X position'
        ),
        
        DeclareLaunchArgument(
            'y', 
            default_value='0',
            description='Initial Y position'
        ),
        
        DeclareLaunchArgument(
            'z',
            default_value='0',
            description='Initial Z position'
        ),
        
        DeclareLaunchArgument(
            'yaw',
            default_value='0',
            description='Initial yaw'
        ),
        
        # Spawn simple usando ros2 run
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'as2_gazebo_assets', 'set_entity_pose_bridge',
                '--robot_namespace', [LaunchConfiguration('ns_prefix'), LaunchConfiguration('id')],
                '--x', LaunchConfiguration('x'),
                '--y', LaunchConfiguration('y'),
                '--z', LaunchConfiguration('z'),
                '--yaw', LaunchConfiguration('yaw')
            ],
            output='screen'
        )
    ])