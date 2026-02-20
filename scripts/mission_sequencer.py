#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_srvs.srv import SetBool  # <-- IMPORTANTE: Para los servicios de armar y offboard
from mission_planner.action import NewTask
import time

class MissionSequencer(Node):
    def __init__(self):
        super().__init__('mission_sequencer')
        # Cliente para enviar las misiones al planificador
        self._action_client = ActionClient(self, NewTask, 'incoming_task_action')
        
        # Clientes para automatizar el encendido de Aerostack2
        self._offboard_client = self.create_client(SetBool, '/drone0/set_offboard_mode')
        self._arming_client = self.create_client(SetBool, '/drone0/set_arming_state')

    def arm_and_offboard(self):
        """Activa el modo Offboard y arma el dron automáticamente."""
        self.get_logger().info('Esperando a que los servicios de vuelo de Aerostack2 estén disponibles...')
        self._offboard_client.wait_for_service()
        self._arming_client.wait_for_service()

        req = SetBool.Request()
        req.data = True

        # 1. Activar Offboard
        self.get_logger().info('Activando modo Offboard...')
        future_offboard = self._offboard_client.call_async(req)
        rclpy.spin_until_future_complete(self, future_offboard)
        if future_offboard.result() is not None and future_offboard.result().success:
            self.get_logger().info('✅ Offboard activado con éxito.')
        else:
            self.get_logger().warning('⚠️ Problema al activar Offboard.')

        time.sleep(1) # Pequeña pausa de seguridad

        # 2. Armar el dron
        self.get_logger().info('Armando los motores...')
        future_arming = self._arming_client.call_async(req)
        rclpy.spin_until_future_complete(self, future_arming)
        if future_arming.result() is not None and future_arming.result().success:
            self.get_logger().info('✅ Motores armados con éxito.')
        else:
            self.get_logger().warning('⚠️ Problema al armar los motores.')

    def send_mission(self):
        # 1. PREPARAR EL DRON AUTOMÁTICAMENTE
        self.arm_and_offboard()

        self.get_logger().info('Esperando 5 segundos de estabilización antes de iniciar las misiones...')
        time.sleep(5)
        
        # 2. LISTA DE MISIONES
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

        # 3. ENVIAR MISIONES SECUENCIALMENTE
        for task_data in mission_tasks:
            self.get_logger().info(f'Enviando tarea al planificador: {task_data["id"]}')
            self.send_goal(task_data)
            time.sleep(2) 

        self.get_logger().info('¡Todas las misiones enviadas con éxito! El dron ahora es completamente autónomo.')

    def send_goal(self, task_data):
        goal_msg = NewTask.Goal()
        goal_msg.task.id = task_data['id']
        goal_msg.task.type = task_data['type']

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
    
    # Inicia la secuencia completa (Armar -> Offboard -> Misiones)
    sequencer.send_mission()
    
    # Mantener vivo un instante para procesar el envío de las misiones
    rclpy.spin_once(sequencer, timeout_sec=2.0)
    
    sequencer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()