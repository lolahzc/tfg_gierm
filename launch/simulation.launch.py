from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_uavs(context, *args, **kwargs):
    number_uav = int(LaunchConfiguration('number_UAV').perform(context))
    nodes = []
    
    # Configuraciones para cada UAV
    uav_configs = [
        {'id': '1', 'type': 'PhysicalACW', 'material': 'Red', 'x': '0', 'y': '0', 'z': '0'},
        {'id': '2', 'type': 'InspectionACW', 'material': 'Green', 'x': '0', 'y': '5', 'z': '0'},
        {'id': '3', 'type': 'SafetyACW', 'material': 'Blue', 'x': '0', 'y': '-5', 'z': '0'},
        {'id': '4', 'type': 'SafetyACW', 'material': 'Blue', 'x': '5', 'y': '5', 'z': '0'},
        {'id': '5', 'type': 'SafetyACW', 'material': 'Blue', 'x': '-5', 'y': '-5', 'z': '0'}
    ]
    
    for i in range(min(number_uav, 5)):
        config = uav_configs[i]
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('mission_planner'),
                        'launch',
                        'uav.launch.py'
                    ])
                ]),
                launch_arguments={
                    'id': config['id'],
                    'config_file': LaunchConfiguration('config_file').perform(context)
                }.items()
            )
        )
    
    return nodes

def generate_launch_description():
    
    return LaunchDescription([
        # Argumentos principales
        DeclareLaunchArgument(
            'number_UAV',
            default_value='1',
            description='Number of UAVs'
        ),
        
        DeclareLaunchArgument(
            'world',
            default_value='pv_farm',
            description='World name'
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
        
        # Launch Gazebo world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mission_planner'),
                    'launch',
                    'world.launch.py'
                ])
            ]),
            launch_arguments={
                'world': LaunchConfiguration('world')
            }.items()
        ),
        
        # Launch Planner
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('mission_planner'),
                    'launch',
                    'planner.launch.py'
                ])
            ]),
            launch_arguments={
                'config_file': LaunchConfiguration('config_file')
            }.items()
        ),
        
        # Launch UAVs dinámicamente
        OpaqueFunction(function=launch_uavs)
    ])