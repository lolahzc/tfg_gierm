#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import SetBool  
from mission_planner.action import NewTask
from mission_planner.msg import Waypoint
import time

class MissionSequencer(Node):
    def __init__(self):
        super().__init__('mission_sequencer')
        
        # 1. Leer el número de drones del Launch File
        self.declare_parameter('n_drones', 3)
        self.n_drones = self.get_parameter('n_drones').value
        self.drone_ids = [f'drone{i}' for i in range(self.n_drones)]
        
        # Cliente para enviar las misiones al planificador
        self._action_client = ActionClient(self, NewTask, 'incoming_task_action')
        
        # 2. Crear diccionarios de clientes para TODOS los drones, no solo drone0
        self.arming_clients = {}
        self.offboard_clients = {}
        
        for d_id in self.drone_ids:
            self.offboard_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_offboard_mode')
            self.arming_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_arming_state')

    def arm_and_offboard_all(self):
        for d_id in self.drone_ids:
            self.get_logger().info(f'🚀 Preparando {d_id}...')
            self.arming_clients[d_id].wait_for_service()
            self.offboard_clients[d_id].wait_for_service()

            req = SetBool.Request(data=True)
            
            future_off = self.offboard_clients[d_id].call_async(req)
            rclpy.spin_until_future_complete(self, future_off)
            
            future_arm = self.arming_clients[d_id].call_async(req)
            rclpy.spin_until_future_complete(self, future_arm)
            
            self.get_logger().info(f'✅ {d_id} armado y en Offboard.')
            
            # Darle a Gazebo 3 segundos de respiro entre despegues
            time.sleep(3)

    def send_mission(self):
        # 3. Llamada corregida a la función
        self.arm_and_offboard_all()

        self.get_logger().info('Esperando 5 segundos de estabilización antes de iniciar las misiones...')
        time.sleep(5)
        
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
                    {'x': 15.0, 'y': 10.0, 'z': 10.0}
                ]}
            },
            {
                'id': 'tarea_inspeccion_pv_1',
                'type': ord('A'), # Inspect PV Array
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

        for task_data in mission_tasks:
            self.get_logger().info(f'Enviando tarea al planificador: {task_data["id"]}')
            self.send_goal(task_data)
            time.sleep(2) 

        self.get_logger().info('¡Todas las misiones enviadas con éxito! El enjambre ahora es completamente autónomo.')

    def send_goal(self, task_data):
        goal_msg = NewTask.Goal()
        goal_msg.task.id = task_data['id']
        goal_msg.task.type = task_data['type']

        if task_data['type'] == ord('M'):
            goal_msg.task.monitor.human_target_id = task_data['params']['human_target_id']
            goal_msg.task.monitor.distance = float(task_data['params']['distance'])
            goal_msg.task.monitor.number = int(task_data['params']['number'])
            
        elif task_data['type'] in [ord('I'), ord('A')]:
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
    
    # Inicia la secuencia completa
    sequencer.send_mission()
    
    rclpy.spin_once(sequencer, timeout_sec=2.0)
    
    sequencer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()