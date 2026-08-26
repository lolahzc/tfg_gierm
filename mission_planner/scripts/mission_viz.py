#!/usr/bin/env python3
"""Gazebo HUD: identifies each drone and shows its battery live.

Draws three markers above every drone:
  * a large sphere in the colour of its charging pad, to tell the otherwise
    identical drones apart in flight;
  * a small sphere whose colour encodes the battery level;
  * a flat disc in the colour of the task type currently being executed.

Only shapes are used: the TEXT marker is not implemented in the Ogre2
renderer, so world text is baked into textures (see gen_mission_labels.py).
All markers go out in a single /marker_array call per cycle.

Run with:
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

# Same colours as the waypoint posts on the ground
# (gen_mission_labels.TASK_COLORS), so a drone matches the posts it flies to.
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
    """Green / amber / red, with red at the low-battery threshold."""
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
            f'HUD active for {", ".join(self.drones)} at {rate:.1f} Hz. '
            'Large sphere = drone identity, small sphere = battery, '
            'disc = task type (same colour as its waypoint posts).')

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
            # Format "t_1|I"; empty when the drone has no task.
            self.task[drone] = msg.data.split('|')[-1] if msg.data else ''
        return cb

    @staticmethod
    def _marker(ns, mid, mtype, x, y, z, rgb, scale, zscale=None):
        r, g, b = rgb
        if zscale is None:
            # Cylinders are drawn as a flat disc under the drone.
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
            # Ids never start at 0: Gazebo treats 0 as unset and adds a new
            # marker on every update instead of moving the existing one.
            base_id = (i + 1) * 10

            color = DRONE_COLORS.get(d, (0.7, 0.7, 0.7))
            # Drone identity.
            parts.append(self._marker('viz_id', base_id, 'SPHERE',
                                      p.x, p.y, p.z + 1.1, color, 0.85))
            # Battery level.
            parts.append(self._marker('viz_bat', base_id + 1, 'SPHERE',
                                      p.x, p.y, p.z + 2.0,
                                      battery_color(self.batt.get(d)), 0.5))
            # Task type, matching the colour of its waypoint posts.
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
            # Gazebo may not be up yet; not a reason to die.
            self.get_logger().warn(f'Could not draw the markers: {e}',
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
