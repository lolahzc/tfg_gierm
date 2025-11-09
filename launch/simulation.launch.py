#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        # High Level Planner
        Node(
            package='mission_planner',
            executable='high_level_planner',
            name='high_level_planner',
            output='screen'
        ),
        
        # Heuristic Planner Simulator
        Node(
            package='mission_planner',
            executable='heuristic_planner_simulator',
            name='heuristic_planner_simulator',
            output='screen'
        ),
        
        # Agent Behaviour Manager
        Node(
            package='mission_planner',
            executable='agent_behaviour_manager',
            name='agent_behaviour_manager_uav0',
            output='screen',
            parameters=[{
                'id': '0',
                'ns_prefix': 'uav',
                'pose_frame_id': 'map',
                'pose_topic': 'self_localization/pose',
                'state_topic': 'platform/info',
                'battery_topic': 'sensor_measurements/battery',
                'take_off_height': 5.0,
                'distance_error': 2.0,
                'goto_error': 1.0,
                'config_file': 'config/conf.yaml'  # El nodo lo carga internamente
            }]
        ),
        
        # Battery Faker
        Node(
            package='mission_planner',
            executable='battery_faker',
            name='battery_faker',
            output='screen',
            parameters=[{
                'id': 'uav0',
                'battery_mode': 'static'
            }]
        ),
        
        # IST UGV Faker
        Node(
            package='mission_planner',
            executable='ist_ugv_faker', 
            name='ist_ugv_faker',
            output='screen'
        )
    ])
