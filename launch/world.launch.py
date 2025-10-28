from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
import os

def generate_launch_description():
    
    # Buscar en toda la workspace de ROS2
    workspace_path = os.path.join(os.path.expanduser('~'), 'lola_tfg')
    mission_planner_path = os.path.join(workspace_path, 'src', 'mission_planner')
    
    world_path = os.path.join(workspace_path, 'install', 'mission_planner', 'share', 'mission_planner', 'worlds', 'evora_solar_panel_farm.world')
    
    # Buscar todas las posibles rutas de modelos
    possible_model_paths = [
        os.path.join(mission_planner_path, 'models'),
        os.path.join(mission_planner_path, 'gazebo_worlds', 'models'),
        os.path.join(mission_planner_path, 'worlds', 'models'),
        os.path.join(workspace_path, 'install', 'mission_planner', 'share', 'mission_planner', 'models'),
    ]
    
    # Agregar rutas estándar de Gazebo
    gazebo_default_paths = [
        '/usr/share/gazebo-11/models',
        os.path.expanduser('~/.gazebo/models')
    ]
    
    all_model_paths = possible_model_paths + gazebo_default_paths
    existing_model_paths = [path for path in all_model_paths if os.path.exists(path)]
    
    # Mantener el path actual
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if current_model_path:
        existing_model_paths.insert(0, current_model_path)
    
    gazebo_model_path = ':'.join(existing_model_paths)
    
    print(f"Using GAZEBO_MODEL_PATH: {gazebo_model_path}")
    
    return LaunchDescription([
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path
        ),
        
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path],
            output='screen',
            shell=True
        )
    ])