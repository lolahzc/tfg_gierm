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
        self.get_logger().info('Esperando estabilidad del sistema...')
        import time
        time.sleep(5)
        mission_tasks = [
            {
                'id': 'tarea_monitor_1',
                'type': ord('M'),
                'params': {'human_target_id': 'human_target_1', 'distance': 2.5, 'number': 1}
            },
            {
                'id': 'tarea_inspeccion_1',
                'type': ord('I'),
                'params': {'waypoints': [
                    {'x': -15.0, 'y': 0.0, 'z': 10.0},
                    {'x': 5.0, 'y': 10.0, 'z': 15.0}
                ]}
            }
        ]

        for task_data in mission_tasks:
            self.get_logger().info(f'Enviando tarea: {task_data["id"]}')
            self.send_goal(task_data)
            import time
            time.sleep(10)  # Esperamos un poco entre tareas para simular una secuencia

    def send_goal(self, task_data):
        goal_msg = NewTask.Goal()
        goal_msg.task.id = task_data['id']
        goal_msg.task.type = task_data['type']

        if task_data['type'] == ord('M'):
            goal_msg.task.monitor.human_target_id = task_data['params']['human_target_id']
            goal_msg.task.monitor.distance = task_data['params']['distance']
            goal_msg.task.monitor.number = task_data['params']['number']
        elif task_data['type'] == ord('I'):
            from mission_planner.msg import Waypoint
            for wp in task_data['params']['waypoints']:
                msg_wp = Waypoint()
                msg_wp.x, msg_wp.y, msg_wp.z = wp['x'], wp['y'], wp['z']
                goal_msg.task.inspect.waypoints.append(msg_wp)

        self._action_client.wait_for_server()
        self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    sequencer = MissionSequencer()
    sequencer.send_mission()
    
    # CORRECCIÓN: Usar spin_once con un pequeño timeout
    rclpy.spin_once(sequencer, timeout_sec=1.0)
    
    sequencer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()