import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # --- ARGUMENTOS ---
    # Permitimos cambiar el ID y el archivo de configuración desde la terminal si hace falta
    uav_id_arg = DeclareLaunchArgument(
        'uav_id', default_value='uav0',
        description='Identificador del UAV (namespace)'
    )
    
    # Ruta por defecto a tu conf.yaml (Ajusta si tu carpeta se llama distinto)
    # Asumo que está en install/share/mission_planner/config tras compilar
    # O puedes apuntar directamente al src si prefieres para desarrollo
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
    # Este nodo decide qué tareas se hacen. No suele llevar namespace específico del dron
    # porque podría gestionar varios, pero se le pasa el config.
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
    # Este es el nodo que tiene el Behavior Tree.
    # Es CRÍTICO que esté en el namespace del dron (uav0) para que sus logs
    # y tópicos no choquen si pones más drones.
    agent_node = Node(
        package='mission_planner',
        executable='agent_behaviour_manager',
        namespace=uav_id,
        name='agent_behaviour_manager',
        output='screen',
        parameters=[
            # En tu código cpp, concatenas ns_prefix + id. 
            # Si pones ns_prefix='uav' e id='0', sale 'uav0'.
            {'id': '0'},          
            {'ns_prefix': 'uav'},
            {'config_file': config_file},
            use_sim_time
        ],
        emulate_tty=True
    )

    # 3. SIMULADOR DE BATERÍA (Battery Faker)
    # Publica el estado de la batería simulado.
    battery_faker = Node(
        package='mission_planner',
        executable='battery_faker',
        name='battery_faker',
        # namespace=uav_id, # Opcional, tu código ya usa el parámetro 'id' para los tópicos
        output='screen',
        parameters=[
            {'id': uav_id}, # Se le pasa 'uav0'
            {'config_file': config_file},
            use_sim_time
        ]
    )

    # 4. SIMULADOR HEURÍSTICO (Matlab Faker)
    # Simula la respuesta del planificador complejo.
    heuristic_sim = Node(
        package='mission_planner',
        executable='heuristic_planner_simulator',
        name='heuristic_planner_simulator',
        output='screen',
        parameters=[use_sim_time]
    )

    # 5. SIMULADOR UGV (IST UGV Faker)
    # Mueve los robots de tierra simulados.
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