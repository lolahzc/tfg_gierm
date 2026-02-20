#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mission_planner.action import NewTask

class MissionSequencer(Node):
    def __init__(self):
        super().__init__('mission_sequencer')
        self._action_client = ActionClient(self, NewTask, 'incoming_task_action')

    def send_mission(self):
        self.get_logger().info('Esperando 5 segundos para la estabilidad del sistema...')
        import time
        time.sleep(5)
        
        # Batería de pruebas sin UGV: 4 misiones diferentes
        mission_tasks = [
            {
                'id': 'tarea_monitor_1',
                'type': ord('M'), # Monitor
                'params': {'human_target_id': 'human_target_1', 'distance': 2.5, 'number': 1}
            },
            {
                'id': 'tarea_inspeccion_1',
                'type': ord('I'), # Inspect normal
                'params': {'waypoints': [
                    {'x': -15.0, 'y': 0.0, 'z': 10.0}
                ]}
            },
            {
                'id': 'tarea_inspeccion_pv_1',
                'type': ord('A'), # Inspect PV Array (Requiere exactamente 2 waypoints)
                'params': {'waypoints': [
                    {'x': 5.0, 'y': 10.0, 'z': 15.0},
                    {'x': 10.0, 'y': 10.0, 'z': 15.0}
                ]}
            },
            {
                'id': 'tarea_entrega_1',
                'type': ord('D'), # Deliver Tool
                'params': {'tool_id': 'tool_1', 'human_target_id': 'human_target_1'}
            }
        ]

        # Enviamos todas las misiones al planificador con un pequeño retraso 
        # para que las vaya encolando ordenadamente
        for task_data in mission_tasks:
            self.get_logger().info(f'Enviando tarea al planificador: {task_data["id"]}')
            self.send_goal(task_data)
            time.sleep(2) 

        self.get_logger().info('¡Todas las misiones enviadas con éxito! Observa el comportamiento del dron.')

    def send_goal(self, task_data):
        goal_msg = NewTask.Goal()
        goal_msg.task.id = task_data['id']
        goal_msg.task.type = task_data['type']

        # Rellenar los parámetros dinámicamente según el tipo de tarea
        if task_data['type'] == ord('M'):
            goal_msg.task.monitor.human_target_id = task_data['params']['human_target_id']
            goal_msg.task.monitor.distance = float(task_data['params']['distance'])
            goal_msg.task.monitor.number = int(task_data['params']['number'])
            
        elif task_data['type'] in [ord('I'), ord('A')]:
            from mission_planner.msg import Waypoint
            for wp in task_data['params']['waypoints']:
                msg_wp = Waypoint()
                msg_wp.x, msg_wp.y, msg_wp.z = float(wp['x']), float(wp['y']), float(wp['z'])
                goal_msg.task.inspect.waypoints.append(msg_wp)
                
        elif task_data['type'] == ord('D'):
            goal_msg.task.deliver.tool_id = task_data['params']['tool_id']
            goal_msg.task.deliver.human_target_id = task_data['params']['human_target_id']

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    sequencer = MissionSequencer()
    sequencer.send_mission()
    
    # Mantener vivo para procesar el envío asíncrono
    rclpy.spin_once(sequencer, timeout_sec=2.0)
    
    sequencer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()