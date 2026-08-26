import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mission_planner')
    config_file = LaunchConfiguration('config_file').perform(context)
    mission_file = LaunchConfiguration('mission_file').perform(context)

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
        parameters=[{'config_file': config_file}]
    ))

    # high_level_planner blocks task allocation forever waiting on the
    # /heuristic_planning action server (real Matlab planner, or this faker).
    # Without it no NewTaskList is ever sent to any agent, so drones stay
    # armed on the ground even though offboard/arming succeeded.
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

    # NOTA: Si el mission_sequencer ya arma los drones, podemos comentar el swarm_initializer 
    # para que no haya conflictos. Si lo necesitas, simplemente descoméntalo.
    # nodes_to_launch.append(Node(
    #     package='mission_planner', executable='swarm_initializer.py',
    #     name='swarm_initializer', output='screen',
    #     parameters=[{'n_drones': n_drones}]
    # ))

    # 4. NODOS DE SECUENCIA Y MISION (¡Descomentado!)
    nodes_to_launch.append(Node(
        package='mission_planner',
        executable='mission_sequencer.py',
        name='mission_sequencer',
        output='screen',
        parameters=[{'n_drones': n_drones, 'mission_file': mission_file}]
    ))

    # 5. HUD VISUAL EN GAZEBO (opcional, viz:=false para desactivarlo)
    # Dibuja sobre cada dron una bola de su color (identidad) y otra con el
    # nivel de bateria, porque los tres modelos son identicos en Gazebo.
    nodes_to_launch.append(Node(
        package='mission_planner',
        executable='mission_viz.py',
        name='mission_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('viz')),
        parameters=[{'drones': [f'drone{i}' for i in range(n_drones)]}]
    ))

    return nodes_to_launch

def generate_launch_description():
    pkg_share = get_package_share_directory('mission_planner')
    return LaunchDescription([
        # Must match mission_planner_gazebo/launch/simulation_launch.py's RMW choice,
        # or this process can't discover the AS2 platform's topics/services at all.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        DeclareLaunchArgument('n_drones', default_value='3', description='Número de drones a lanzar'),
        DeclareLaunchArgument('config_file', default_value=os.path.join(pkg_share, 'config', 'conf.yaml')),
        DeclareLaunchArgument('mission_file', default_value=os.path.join(pkg_share, 'config', 'mission.yaml'),
                              description='YAML mission plan for mission_sequencer.py (see config/mission.yaml)'),
        DeclareLaunchArgument('viz', default_value='true',
                              description='Dibuja el HUD de identificacion/bateria en Gazebo'),
        OpaqueFunction(function=launch_setup)
    ])