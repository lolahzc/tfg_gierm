from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Función para determinar el prefix basado en el debugger
    def get_prefix(context):
        debugger = LaunchConfiguration('planner_debugger').perform(context)
        if debugger == 'gdb':
            return 'xterm -e gdb -ex "set print thread-events off" -ex run --args'
        elif debugger == 'valgrind':
            return 'valgrind -v --leak-check=full --show-leak-kinds=all --track-origins=yes --log-file=/home/baldman/valgrind_planner.log'
        else:
            return ''  # Sin prefix para 'none'
    
    return LaunchDescription([
        # Argumentos
        DeclareLaunchArgument(
            'planner_debugger',
            default_value='none',
            description='Debuggers: gdb, valgrind, none'
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
        
        # Un solo nodo con prefix condicional
        Node(
            package='mission_planner',
            executable='high_level_planner',
            name='high_level_planner',
            output='screen',
            parameters=[{
                'config_file': LaunchConfiguration('config_file'),
                'pose_topic': '/ual/pose',
                'battery_topic': '/battery_fake'
            }]
        )
    ])