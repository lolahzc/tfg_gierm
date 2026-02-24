import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mission_planner')
    config_file = LaunchConfiguration('config_file').perform(context)
    
    # 1. Obtener número de drones del argumento (por defecto 3)
    try:
        n_drones = int(LaunchConfiguration('n_drones').perform(context))
    except Exception:
        n_drones = 3
        
    nodes_to_launch = []

    # 2. NODOS GLOBALES
    nodes_to_launch.append(Node(
        package='mission_planner', executable='high_level_planner',
        name='planner_node', output='screen',
        parameters=[{
            'config_file': config_file
        }]
    ))

    nodes_to_launch.append(Node(
        package='mission_planner', executable='heuristic_planner_simulator',
        name='heuristic_planner_simulator', output='screen'
    ))

    # 3. NODOS POR DRON (Generados en bucle de forma segura)
    for i in range(n_drones):
        uav_id = f"drone{i}"
        
        nodes_to_launch.append(Node(
            package='mission_planner', executable='agent_behaviour_manager',
            name=f'agent_behaviour_manager_{uav_id}', output='screen',
            parameters=[{'id': uav_id, 'ns_prefix': '', 'pose_frame_id': 'earth', 'config_file': config_file}]
        ))
        
        nodes_to_launch.append(Node(
            package='mission_planner', executable='battery_faker',
            name=f'battery_faker_{uav_id}', output='screen',
            parameters=[{'id': uav_id, 'battery_mode': 'recharge_in_base', 'config_file': config_file}]
        ))

    # nodes_to_launch.append(Node(
    #     package='mission_planner', executable='swarm_initializer.py',
    #     name='swarm_initializer', output='screen',
    #     parameters=[{'n_drones': n_drones}]
    # ))

    nodes_to_launch.append(Node(
        package='mission_planner', 
        executable='mission_sequencer.py',
        name='mission_sequencer', 
        output='screen',
        parameters=[{'n_drones': n_drones}]
    ))

    return nodes_to_launch

def generate_launch_description():
    pkg_share = get_package_share_directory('mission_planner')
    return LaunchDescription([
        # Argumento principal que puedes cambiar desde la terminal
        DeclareLaunchArgument('n_drones', default_value='3', description='Número de drones a lanzar'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(pkg_share, 'config', 'conf.yaml')),
        OpaqueFunction(function=launch_setup)
    ])