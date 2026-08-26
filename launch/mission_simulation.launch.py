import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- ARGUMENTOS ---
    # Drone id and config file can be overridden from the command line
    uav_id_arg = DeclareLaunchArgument(
        'uav_id', default_value='uav0',
        description='UAV identifier (namespace)'
    )
    
    # Default path to conf.yaml
    # Installed under share/mission_planner/config after building
    # Point it at src/ instead when developing
    config_pkg_dir = get_package_share_directory('mission_planner')
    default_config_path = os.path.join(config_pkg_dir, 'config', 'conf.yaml')
    
    config_file_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config_path,
        description='Ruta al archivo conf.yaml'
    )

    uav_id = LaunchConfiguration('uav_id')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = {'use_sim_time': True}

    # --- NODOS DE TU TFG ---

    # 1. PLANIFICADOR GLOBAL (High Level Planner)
    # Decides which tasks get done. No per-drone namespace, since it can
    # manage several of them; it just takes the config.
    planner_node = Node(
        package='mission_planner',
        executable='high_level_planner',
        name='high_level_planner',
        output='screen',
        parameters=[
            {'config_file': config_file},
            use_sim_time
        ],
        emulate_tty=True
    )

    # 2. GESTOR DEL AGENTE (Agent Behaviour Manager)
    # This node runs the behavior tree.
    # Must live in the drone namespace so its logs and topics do not
    # clash when more drones are added.
    agent_node = Node(
        package='mission_planner',
        executable='agent_behaviour_manager',
        namespace=uav_id,
        name='agent_behaviour_manager',
        output='screen',
        parameters=[
            # The C++ side concatenates ns_prefix + id.
            # Si pones ns_prefix='uav' e id='0', sale 'uav0'.
            {'id': '0'},          
            {'ns_prefix': 'uav'},
            {'config_file': config_file},
            use_sim_time
        ],
        emulate_tty=True
    )

    # 3. SIMULADOR DE BATERÍA (Battery Faker)
    # Publishes the simulated battery state.
    battery_faker = Node(
        package='mission_planner',
        executable='battery_faker',
        name='battery_faker',
        # namespace=uav_id,  # optional: the code already uses the 'id' parameter
        output='screen',
        parameters=[
            {'id': uav_id}, # Se le pasa 'uav0'
            {'config_file': config_file},
            use_sim_time
        ]
    )

    # 4. SIMULADOR HEURÍSTICO (Matlab Faker)
    # Stands in for the external planner.
    heuristic_sim = Node(
        package='mission_planner',
        executable='heuristic_planner_simulator',
        name='heuristic_planner_simulator',
        output='screen',
        parameters=[use_sim_time]
    )

    # 5. SIMULADOR UGV (IST UGV Faker)
    # Drives the simulated ground robots.
    ugv_faker = Node(
        package='mission_planner',
        executable='ist_ugv_faker',
        name='ist_ugv_faker',
        output='screen',
        parameters=[use_sim_time]
    )

    return LaunchDescription([
        # Must match mission_planner_gazebo/launch/simulation_launch.py's RMW choice,
        # or this process can't discover the AS2 platform's topics/services at all.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        uav_id_arg,
        config_file_arg,
        LogInfo(msg="--- LANZANDO SISTEMA DE MISIÓN (TFG) ---"),
        planner_node,
        agent_node,
        battery_faker,
        heuristic_sim,
        ugv_faker
    ])