#!/usr/bin/env python3
"""HUD en Gazebo: identifica cada dron y ensena su bateria en vivo.

Sobre cada dron dibuja dos marcadores:
  * una bola grande del color de su base (drone0 rojo, drone1 verde,
    drone2 azul), para saber cual es cual en vuelo - los tres modelos son
    identicos y sin esto no hay forma de distinguirlos;
  * una bola pequena encima cuyo color indica la bateria
    (verde >60%, amarillo 30-60%, rojo <30% = va a irse a recargar).

Se dibuja con el servicio /marker de Gazebo. Se usan SPHERE/CYLINDER a
proposito: el marcador TEXT (tipo 7) NO esta implementado en el renderer
Ogre2 de Gazebo Sim 8 y falla con "Invalid Marker type [7]", asi que el
texto del mundo va horneado en texturas (ver gen_mission_labels.py) y aqui
solo se usan formas.

Todos los marcadores de un ciclo van en UNA sola llamada a /marker_array,
porque cada llamada lanza un proceso `gz service`.

Lanzar con:
    ros2 run mission_planner mission_viz.py
"""
import subprocess

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String

DRONE_COLORS = {
    'drone0': (0.86, 0.16, 0.16),
    'drone1': (0.16, 0.67, 0.24),
    'drone2': (0.20, 0.35, 0.86),
}

# Mismos colores que usan los postes de waypoint en el suelo
# (gen_mission_labels.TASK_COLORS): si el dron lleva el disco naranja, va a por
# los postes naranjas. Sin tarea = gris apagado.
TASK_COLORS = {
    'I': (0.00, 0.63, 0.78),   # Inspect
    'A': (0.94, 0.55, 0.08),   # InspectPVArray
    'M': (0.78, 0.24, 0.63),   # Monitor
    'D': (0.59, 0.24, 0.78),   # DeliverTool
    'R': (0.95, 0.85, 0.10),   # Recharge
    'F': (0.20, 0.70, 0.70),   # MonitorUGV
}
NO_TASK_COLOR = (0.35, 0.35, 0.35)


def battery_color(pct):
    """Verde / amarillo / rojo. El rojo coincide con el umbral (30%) en el
    que isBatteryEnough() aborta la tarea y manda el dron a recargar."""
    if pct is None:
        return (0.5, 0.5, 0.5)
    if pct >= 0.60:
        return (0.10, 0.80, 0.10)
    if pct >= 0.30:
        return (0.95, 0.80, 0.10)
    return (0.90, 0.10, 0.10)


class MissionViz(Node):

    def __init__(self):
        super().__init__('mission_viz')
        self.declare_parameter('drones', list(DRONE_COLORS.keys()))
        self.declare_parameter('rate', 2.0)
        self.drones = self.get_parameter('drones').value
        rate = float(self.get_parameter('rate').value)

        self.pose = {}
        self.batt = {}
        self.task = {}
        for d in self.drones:
            self.create_subscription(
                PoseStamped, f'/{d}/self_localization/pose',
                self._mk_pose_cb(d), qos_profile_sensor_data)
            self.create_subscription(
                BatteryState, f'/{d}/sensor_measurements/battery',
                self._mk_batt_cb(d), 10)
            self.create_subscription(
                String, f'/{d}/current_task', self._mk_task_cb(d), 10)

        self.timer = self.create_timer(1.0 / rate, self.publish_markers)
        self.get_logger().info(
            f'HUD activo para {", ".join(self.drones)} a {rate:.1f} Hz. '
            'Bola grande = identidad del dron, bola pequena = bateria, '
            'disco = tipo de tarea (mismo color que sus postes en el suelo).')

    def _mk_pose_cb(self, drone):
        def cb(msg):
            self.pose[drone] = msg.pose.position
        return cb

    def _mk_batt_cb(self, drone):
        def cb(msg):
            self.batt[drone] = msg.percentage
        return cb

    def _mk_task_cb(self, drone):
        def cb(msg):
            # Formato "t_1|I"; vacio si el dron no tiene tarea.
            self.task[drone] = msg.data.split('|')[-1] if msg.data else ''
        return cb

    @staticmethod
    def _marker(ns, mid, mtype, x, y, z, rgb, scale, zscale=None):
        r, g, b = rgb
        if zscale is None:
            # Los cilindros se usan como disco plano bajo el dron.
            zscale = 0.12 if mtype == 'CYLINDER' else scale
        return (f'marker {{ action: ADD_MODIFY ns: "{ns}" id: {mid} '
                f'type: {mtype} visibility: GUI '
                f'pose {{ position {{ x: {x:.3f} y: {y:.3f} z: {z:.3f} }} '
                f'orientation {{ w: 1 }} }} '
                f'scale {{ x: {scale:.3f} y: {scale:.3f} z: {zscale:.3f} }} '
                f'material {{ ambient {{ r: {r:.3f} g: {g:.3f} b: {b:.3f} a: 1 }} '
                f'diffuse {{ r: {r:.3f} g: {g:.3f} b: {b:.3f} a: 1 }} '
                f'emissive {{ r: {r*0.5:.3f} g: {g*0.5:.3f} b: {b*0.5:.3f} a: 1 }} }} }}')

    def publish_markers(self):
        parts = []
        for i, d in enumerate(self.drones):
            p = self.pose.get(d)
            if p is None:
                continue
            # OJO: los ids empiezan en 1, nunca en 0. Gazebo trata el id 0 como
            # "sin asignar" y crea un marcador NUEVO en cada actualizacion en
            # vez de mover el existente, asi que el primer dron iba dejando un
            # rastro de bolas por todo el mapa mientras los demas se movian
            # bien.
            base_id = (i + 1) * 10

            color = DRONE_COLORS.get(d, (0.7, 0.7, 0.7))
            # Identidad del dron.
            parts.append(self._marker('viz_id', base_id, 'SPHERE',
                                      p.x, p.y, p.z + 1.1, color, 0.85))
            # Nivel de bateria.
            parts.append(self._marker('viz_bat', base_id + 1, 'SPHERE',
                                      p.x, p.y, p.z + 2.0,
                                      battery_color(self.batt.get(d)), 0.5))
            # Tipo de tarea en curso, del mismo color que sus postes en el suelo.
            tcolor = TASK_COLORS.get(self.task.get(d, ''), NO_TASK_COLOR)
            parts.append(self._marker('viz_task', base_id + 2, 'CYLINDER',
                                      p.x, p.y, p.z + 0.45, tcolor, 1.5))
        if not parts:
            return

        req = ' '.join(parts)
        try:
            subprocess.run(
                ['gz', 'service', '-s', '/marker_array',
                 '--reqtype', 'gz.msgs.Marker_V',
                 '--reptype', 'gz.msgs.Boolean',
                 '--timeout', '800', '--req', req],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                timeout=3.0, check=False)
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            # Gazebo puede no estar levantado todavia; no es motivo para morir.
            self.get_logger().warn(f'No se pudieron dibujar los marcadores: {e}',
                                   throttle_duration_sec=10.0)


def main():
    rclpy.init()
    node = MissionViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
