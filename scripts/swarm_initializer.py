#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool  
import time

class SwarmInitializer(Node):
    def __init__(self):
        super().__init__('swarm_initializer')
        
        # Read the number of drones from the launch file
        self.declare_parameter('n_drones', 3)
        self.n_drones = self.get_parameter('n_drones').value
        self.drone_ids = [f'drone{i}' for i in range(self.n_drones)]
        
        # Build the client dictionaries for every drone
        self.arming_clients = {}
        self.offboard_clients = {}
        
        for d_id in self.drone_ids:
            self.offboard_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_offboard_mode')
            self.arming_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_arming_state')

    def arm_and_offboard_all(self):
        for d_id in self.drone_ids:
            self.get_logger().info(f' Preparando {d_id}...')
            
            # Wait for the AEROSTACK2/Gazebo services to come up
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
            
            self.get_logger().info(f' {d_id} armado y en Offboard.')
            
            # Give Gazebo a few seconds between takeoffs
            time.sleep(3.0)

        self.get_logger().info('Whole swarm armed and ready to receive missions')

def main(args=None):
    rclpy.init(args=args)
    initializer = SwarmInitializer()
    
    # Start the arming sequence
    initializer.arm_and_offboard_all()
    
    # Destroy the node and exit cleanly
    initializer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()