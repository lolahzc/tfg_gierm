"""Bring up the AEROSTACK2 stack (platform, state estimator, controller,
motion behaviors) for a single simulated drone in Gazebo.

Meant to be included once per drone by simulation_launch.py, not run
standalone (it assumes as2_gazebo_assets/launch_simulation.py has already
spawned the drone named `namespace` in Gazebo).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('mission_planner_gazebo')

    # See simulation_launch.py: pinned so this also works if launched standalone.
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp')

    namespace_arg = DeclareLaunchArgument(
        'namespace', description='Drone namespace (must match a drone in simulation_config_file)')
    simulation_config_file_arg = DeclareLaunchArgument(
        'simulation_config_file', description='as2_gazebo_assets world/drones config file')
    platform_config_file_arg = DeclareLaunchArgument(
        'platform_config_file',
        default_value=os.path.join(pkg_share, 'config', 'config.yaml'),
        description='AS2 platform/state_estimator/controller/behaviors config file')
    plugin_config_file_arg = DeclareLaunchArgument(
        'plugin_config_file',
        default_value=os.path.join(pkg_share, 'config', 'pid_speed_controller.yaml'),
        description='pid_speed_controller gains file')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    namespace = LaunchConfiguration('namespace')
    simulation_config_file = LaunchConfiguration('simulation_config_file')
    platform_config_file = LaunchConfiguration('platform_config_file')
    plugin_config_file = LaunchConfiguration('plugin_config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_platform_gazebo'),
            'launch', 'platform_gazebo_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'platform_config_file': platform_config_file,
            'simulation_config_file': simulation_config_file,
            'use_sim_time': use_sim_time,
            # platform_gazebo_launch.py otherwise derives these itself from
            # LaunchConfiguration('namespace') inside its own OpaqueFunction,
            # which is evaluated lazily and picks up whatever drone's
            # namespace happens to be current in the shared launch context
            # at that point - not necessarily this drone's. Confirmed: with
            # 3 drones included in a loop, drone1's platform node ended up
            # publishing to /gz/drone0/cmd_vel, /gz/drone0/arm, /gz/drone0/acro
            # (ros2 node info /drone1/platform). Passing these explicitly,
            # built from this include's own resolved namespace string, avoids
            # relying on that lazy re-evaluation.
            'cmd_vel_topic': ['/gz/', namespace, '/cmd_vel'],
            'arm_topic': ['/gz/', namespace, '/arm'],
            'acro_topic': ['/gz/', namespace, '/acro'],
        }.items()
    )

    state_estimator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_state_estimator'),
            'launch', 'state_estimator_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'config_file': platform_config_file,
            # Pinned explicitly: as2_motion_controller's own 'plugin_name' argument
            # (set below, e.g. "pid_speed_controller") shares this same launch
            # configuration name and otherwise leaks into this include on the next
            # drone's platform_launch.py, since IncludeLaunchDescription doesn't
            # sandbox launch configurations between includes.
            'plugin_name': 'ground_truth',
            'use_sim_time': use_sim_time,
        }.items()
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_motion_controller'),
            'launch', 'controller_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'config_file': platform_config_file,
            'plugin_name': 'pid_speed_controller',
            'plugin_config_file': plugin_config_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    behaviors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('as2_behaviors_motion'),
            'launch', 'motion_behaviors_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'config_file': platform_config_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([
        rmw_env,
        namespace_arg,
        simulation_config_file_arg,
        platform_config_file_arg,
        plugin_config_file_arg,
        use_sim_time_arg,
        platform,
        state_estimator,
        controller,
        behaviors,
    ])
