import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mission_planner')
    default_config_file = os.path.join(pkg_share, 'config', 'conf.yaml')

    uav_id_arg = DeclareLaunchArgument('uav_id', default_value='0')
    config_file_arg = DeclareLaunchArgument('config_file', default_value=default_config_file)

    # --- SOLUCIÓN AL ERROR DE TIPO ---
    # Esta expresión envuelve el valor en comillas extra para que ROS lo vea como "0" (string) y no 0 (int)
    uav_id_str = PythonExpression(["'\"' + '", LaunchConfiguration('uav_id'), "' + '\"'"])

    planner_node = Node(
        package='mission_planner',
        executable='high_level_planner',
        name='planner_node',
        output='screen',
        parameters=[{'config_file': LaunchConfiguration('config_file')}]
    )

    heuristic_sim_node = Node(
        package='mission_planner', executable='heuristic_planner_simulator',
        name='heuristic_planner_simulator', output='screen'
    )

    battery_faker_node = Node(
        package='mission_planner', executable='battery_faker', name='battery_faker',
        output='screen',
        parameters=[{
            'id': uav_id_str, # <--- Usamos la cadena forzada
            'battery_mode': 'recharge_in_base',
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    agent_bt_node = Node(
        package='mission_planner', executable='agent_behaviour_manager',
        name=['agent_behaviour_manager_', LaunchConfiguration('uav_id')],
        output='screen',
        parameters=[{
            'id': uav_id_str, # <--- Usamos la cadena forzada
            'ns_prefix': '',
            'pose_frame_id': 'earth', 
            'take_off_height': 5.0,
            'config_file': LaunchConfiguration('config_file')
        }]
    )

    mission_sequencer_node = Node(
        package='mission_planner', executable='mission_sequencer.py',
        name='mission_sequencer', output='screen'
    )

    return LaunchDescription([
        uav_id_arg, config_file_arg, planner_node,
        heuristic_sim_node, battery_faker_node, agent_bt_node, mission_sequencer_node
    ])