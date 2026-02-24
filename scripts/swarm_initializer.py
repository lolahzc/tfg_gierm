#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class SwarmInitializer(Node):
    def __init__(self):
        super().__init__('swarm_initializer')
        self.declare_parameter('n_drones', 3)
        n_drones = self.get_parameter('n_drones').get_parameter_value().integer_value
        self.drone_ids = [f'drone{i}' for i in range(n_drones)]
        
    def setup_swarm(self):
        for d_id in self.drone_ids:
            self.get_logger().info(f'🔧 Activando servicios para {d_id}...')
            
            # Clientes de servicio para Aerostack2
            offboard_cli = self.create_client(SetBool, f'/{d_id}/set_offboard_mode')
            arming_cli = self.create_client(SetBool, f'/{d_id}/set_arming_state')

            # Esperar a que los servicios estén listos
            if not offboard_cli.wait_for_service(timeout_sec=5.0) or \
               not arming_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().error(f'❌ Servicios de {d_id} no disponibles. Saltando...')
                continue

            req = SetBool.Request(data=True)
            
            # 1. Modo Offboard
            self.get_logger().info(f'  -> {d_id}: Activando Offboard...')
            offboard_cli.call_async(req)
            time.sleep(1.0)
            
            # 2. Armado (Mueve al dron al Estado 2: LANDED_ARMED)
            self.get_logger().info(f'  -> {d_id}: Armando motores...')
            arming_cli.call_async(req)
            
            self.get_logger().info(f'✅ {d_id} listo para recibir misiones de MATLAB.')
            time.sleep(1.0)

def main():
    rclpy.init()
    initializer = SwarmInitializer()
    initializer.setup_swarm()
    initializer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()