import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # --- CONFIGURACIÓN ---
    uav_id = 'uav0'
    use_sim_time = 'true'
    
    # Rutas
    json_config = '/home/lolussk/lola_tfg/src/tfg_gierm/config/config_drone.json'
    behaviors_config = '/home/lolussk/lola_tfg/src/tfg_gierm/config/behaviors.yaml'

    # --- 1. PLATAFORMA ---
    platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([get_package_share_directory('as2_platform_gazebo'), 'launch', 'platform_gazebo_launch.py'])
        ]),
        launch_arguments={
            'namespace': uav_id,
            'use_sim_time': use_sim_time,
            'simulation_config_file': json_config,
            'platform_control_mode': 'speed'
        }.items()
    )

    # --- 2. PUENTE MANUAL (LA SOLUCIÓN AL ERROR DE TERMINAL 1) ---
    # Gazebo no estaba enviando la posición. Esto lo fuerza.
    # Puenteamos: Gazebo /model/uav0/pose  --> ROS /uav0/ground_truth/pose
    manual_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=uav_id,
        name='manual_pose_bridge',
        arguments=[
            f'/model/{uav_id}/pose@geometry_msgs/msg/PoseStamped[ignition.msgs.Pose'
        ],
        remappings=[
            (f'/model/{uav_id}/pose', 'ground_truth/pose')
        ],
        output='screen'
    )

    # --- 3. ESTIMADOR DE ESTADO ---
    estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([get_package_share_directory('as2_state_estimator'), 'launch', 'state_estimator_launch.py'])
        ]),
        launch_arguments={
            'namespace': uav_id,
            'use_sim_time': use_sim_time,
            'plugin_name': 'ground_truth',
        }.items()
    )

    # --- 4. CONTROLADOR ---
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([get_package_share_directory('as2_motion_controller'), 'launch', 'controller_launch.py'])
        ]),
        launch_arguments={
            'namespace': uav_id,
            'use_sim_time': use_sim_time,
            'plugin_name': 'pid_speed_controller',
            'motion_controller_config_file': behaviors_config
        }.items()
    )

    # --- 5. BEHAVIORS ---
    behaviors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([get_package_share_directory('as2_behaviors_motion'), 'launch', 'motion_behaviors_launch.py'])
        ]),
        launch_arguments={
            'namespace': uav_id,
            'use_sim_time': use_sim_time,
            'config_file': behaviors_config,
            'takeoff_plugin_name': 'takeoff_plugin_platform',
            'land_plugin_name': 'land_plugin_platform',
            'go_to_plugin_name': 'go_to_plugin_position',
            'follow_path_plugin_name': 'follow_path_plugin_trajectory'
        }.items()
    )

    # --- 6. RED DE SEGURIDAD TF ---
    # Une la Tierra con el Mapa
    tf_earth_to_map = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_earth_map',
        arguments=['0', '0', '0', '0', '0', '0', 'earth', f'{uav_id}/map']
    )

    # Une el Mapa con la Odometría (Cierre del bucle)
    tf_map_to_odom = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', f'{uav_id}/map', f'{uav_id}/odom']
    )

    return LaunchDescription([
        LogInfo(msg="--- LANZANDO PILA AS2 CON PUENTE MANUAL ---"),
        platform,
        manual_bridge,  # <--- Nuevo nodo crítico
        estimator,
        controller,
        behaviors,
        tf_earth_to_map,
        tf_map_to_odom
    ])