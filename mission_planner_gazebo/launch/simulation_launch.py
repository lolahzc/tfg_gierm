"""Bring up the full AEROSTACK2 + Gazebo simulation: spawns Gazebo with the
world/drones described in `simulation_config_file`, then brings up the AS2
platform/state_estimator/controller/behaviors stack for every drone listed
in that same file.

This only brings up the flight stack. Run mission_planner's own
mission_launch.py / mission_simulation.launch.py afterwards to bring up the
mission planner nodes on top of it.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('mission_planner_gazebo')
    simulation_config_file = LaunchConfiguration('simulation_config_file').perform(context)

    with open(simulation_config_file) as f:
        world_config = yaml.safe_load(f)
    drone_namespaces = [d['model_name'] for d in world_config.get('drones', [])]
    if not drone_namespaces:
        raise RuntimeError(f'No drones found in {simulation_config_file}')

    actions = []

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_gazebo_assets'),
            'launch', 'launch_simulation.py')),
        launch_arguments={
            'simulation_config_file': simulation_config_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    ))

    for namespace in drone_namespaces:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_share, 'launch', 'platform_launch.py')),
            launch_arguments={
                'namespace': namespace,
                'simulation_config_file': simulation_config_file,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items()
        ))

    return actions


def _gz_resource_path():
    """mission_planner's worlds/ and models/, for GZ_SIM_RESOURCE_PATH.

    as2_gazebo_assets resolves world_name by looking for
    "<world_name>.sdf.jinja" along GZ_SIM_RESOURCE_PATH, and Gazebo resolves
    "model://<name>/..." the same way. Adding both directories makes the evora
    world and its models available without touching the aerostack2 install.
    """
    mp_share = get_package_share_directory('mission_planner')
    entries = [os.path.join(mp_share, 'worlds'), os.path.join(mp_share, 'models')]
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if existing:
        entries.append(existing)
    return ':'.join(entries)


def generate_launch_description():
    pkg_share = get_package_share_directory('mission_planner_gazebo')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', _gz_resource_path()),
        # Fast DDS: CycloneDDS runs out of participant indices with this many
        # nodes per drone. mission_planner's launch files must match, or the
        # two processes cannot discover each other at all.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        DeclareLaunchArgument(
            'simulation_config_file',
            default_value=os.path.join(pkg_share, 'config', 'world_swarm.yaml'),
            description='as2_gazebo_assets world/drones config file '
                        '(world.yaml for a single drone, world_swarm.yaml for 3)'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
