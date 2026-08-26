#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  
import time

class SwarmInitializer(Node):
    def __init__(self):
        super().__init__('swarm_initializer')
        
        # 1. Leer el número de drones del Launch File
        self.declare_parameter('n_drones', 3)
        self.n_drones = self.get_parameter('n_drones').value
        self.drone_ids = [f'drone{i}' for i in range(self.n_drones)]
        
        # 2. Crear diccionarios de clientes para TODOS los drones
        self.arming_clients = {}
        self.offboard_clients = {}
        
        for d_id in self.drone_ids:
            self.offboard_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_offboard_mode')
            self.arming_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_arming_state')

    def arm_and_offboard_all(self):
        for d_id in self.drone_ids:
            self.get_logger().info(f'🚀 Preparando {d_id}...')
            
            # Esperar a que los servicios de Aerostack2/Gazebo estén levantados
            self.offboard_clients[d_id].wait_for_service()
            self.arming_clients[d_id].wait_for_service()

            req = SetBool.Request(data=True)
            
            # 1. Activar Offboard
            self.get_logger().info(f'  -> {d_id}: Activando Offboard...')
            future_off = self.offboard_clients[d_id].call_async(req)
            rclpy.spin_until_future_complete(self, future_off)
            
            # 2. Armar motores
            self.get_logger().info(f'  -> {d_id}: Armando motores...')
            future_arm = self.arming_clients[d_id].call_async(req)
            rclpy.spin_until_future_complete(self, future_arm)
            
            self.get_logger().info(f'✅ {d_id} armado y en Offboard.')
            
            # Darle a Gazebo 3 segundos de respiro entre despegues
            time.sleep(3.0)

        self.get_logger().info('¡Todo el enjambre está armado y listo para recibir misiones de MATLAB!')

def main(args=None):
    rclpy.init(args=args)
    initializer = SwarmInitializer()
    
    # Inicia la secuencia de armado
    initializer.arm_and_offboard_all()
    
    # Destruir nodo y salir limpiamente cuando termine
    initializer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()