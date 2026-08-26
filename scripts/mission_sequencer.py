#!/usr/bin/env python3
"""Arms/offboards the swarm and injects a configurable mission plan.

The mission plan (which tasks, to which waypoints, and when) comes from a
YAML file instead of being hardcoded, and is organized into "waves" so new
tasks keep getting injected over time instead of just once at startup. See
config/mission.yaml for the schema and an example.
"""
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from mission_planner.action import NewTask
from mission_planner.msg import Waypoint
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import SetBool

TYPE_CHAR_TO_ORD = {
    'M': ord('M'), 'I': ord('I'), 'A': ord('A'), 'D': ord('D'),
}


class MissionSequencer(Node):
    def __init__(self):
        super().__init__('mission_sequencer')

        self.declare_parameter('n_drones', 3)
        self.n_drones = self.get_parameter('n_drones').value
        self.drone_ids = [f'drone{i}' for i in range(self.n_drones)]

        default_mission_file = os.path.join(
            get_package_share_directory('mission_planner'), 'config', 'mission.yaml')
        self.declare_parameter('mission_file', default_mission_file)
        self.mission_file = self.get_parameter('mission_file').value

        self.declare_parameter('arm_offboard_timeout_sec', 10.0)
        self.declare_parameter('arm_offboard_max_attempts', 3)
        self.arm_offboard_timeout_sec = self.get_parameter('arm_offboard_timeout_sec').value
        self.arm_offboard_max_attempts = self.get_parameter('arm_offboard_max_attempts').value

        self._action_client = ActionClient(self, NewTask, 'incoming_task_action')

        self.arming_clients = {}
        self.offboard_clients = {}
        for d_id in self.drone_ids:
            self.offboard_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_offboard_mode')
            self.arming_clients[d_id] = self.create_client(SetBool, f'/{d_id}/set_arming_state')

    # ------------------------------------------------------------------
    # Arm / offboard
    # ------------------------------------------------------------------
    def _call_bool_service(self, client, drone_id, service_name):
        """Call a SetBool service with a timeout and a few retries.

        Returns True only if the service both responded and reported
        success, so a drone whose platform never comes up (or that
        rejects the request) can't hang the whole sequencer or be
        silently treated as ready.
        """
        for attempt in range(1, self.arm_offboard_max_attempts + 1):
            if not client.wait_for_service(timeout_sec=self.arm_offboard_timeout_sec):
                self.get_logger().warn(
                    f'  -> {drone_id}: {service_name} not available after '
                    f'{self.arm_offboard_timeout_sec:.0f}s (attempt {attempt}/'
                    f'{self.arm_offboard_max_attempts})')
                continue

            future = client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(self, future, timeout_sec=self.arm_offboard_timeout_sec)

            if not future.done():
                self.get_logger().warn(
                    f'  -> {drone_id}: {service_name} call timed out '
                    f'(attempt {attempt}/{self.arm_offboard_max_attempts})')
                continue
            if future.exception() is not None:
                self.get_logger().warn(
                    f'  -> {drone_id}: {service_name} call raised {future.exception()} '
                    f'(attempt {attempt}/{self.arm_offboard_max_attempts})')
                continue

            response = future.result()
            if response.success:
                return True
            self.get_logger().warn(
                f'  -> {drone_id}: {service_name} rejected: {response.message} '
                f'(attempt {attempt}/{self.arm_offboard_max_attempts})')

        return False

    def arm_and_offboard_all(self):
        """Arms/offboards every drone. Returns the set of drones that made it.

        A drone that fails is logged and skipped rather than blocking the
        rest of the swarm - the mission still runs for whichever drones
        are actually ready.
        """
        ready_drones = set()
        for d_id in self.drone_ids:
            self.get_logger().info(f'Preparing {d_id}')

            self.get_logger().info(f'  -> {d_id}: Activando Offboard...')
            offboard_ok = self._call_bool_service(
                self.offboard_clients[d_id], d_id, 'set_offboard_mode')

            armed_ok = False
            if offboard_ok:
                self.get_logger().info(f'  -> {d_id}: Armando motores...')
                armed_ok = self._call_bool_service(
                    self.arming_clients[d_id], d_id, 'set_arming_state')

            if offboard_ok and armed_ok:
                self.get_logger().info(f' {d_id} armado y en Offboard.')
                ready_drones.add(d_id)
            else:
                self.get_logger().error(
                    f'{d_id} could not be prepared (offboard={offboard_ok}, armed={armed_ok}); '
                    'skipping it for this mission.')

            # Give Gazebo a moment of breathing room between drones.
            time.sleep(3.0)

        if ready_drones:
            self.get_logger().info(
                f' {len(ready_drones)}/{len(self.drone_ids)} agentes listos: '
                f'{sorted(ready_drones)}')
        else:
            self.get_logger().error('No agent could be prepared; no mission will be injected.')
        return ready_drones

    # ------------------------------------------------------------------
    # Mission plan (from YAML) -> NewTask goals
    # ------------------------------------------------------------------
    def load_mission_plan(self):
        """Loads waves of tasks from self.mission_file. See config/mission.yaml."""
        try:
            with open(self.mission_file) as f:
                plan = yaml.safe_load(f) or {}
        except OSError as e:
            self.get_logger().error(f'Could not read mission_file {self.mission_file}: {e}')
            return []

        waves = plan.get('waves', [])
        if not waves:
            self.get_logger().warn(f'{self.mission_file} defines no task waves.')
        return waves

    def build_goal(self, task_cfg):
        task_type = task_cfg['type'].upper()
        if task_type not in TYPE_CHAR_TO_ORD:
            raise ValueError(f"Unknown task type '{task_type}' for task '{task_cfg['id']}'")

        goal_msg = NewTask.Goal()
        goal_msg.task.id = task_cfg['id']
        goal_msg.task.type = TYPE_CHAR_TO_ORD[task_type]

        if task_type == 'M':
            goal_msg.task.monitor.human_target_id = task_cfg['human_target_id']
            goal_msg.task.monitor.distance = float(task_cfg['distance'])
            goal_msg.task.monitor.number = int(task_cfg['number'])
        elif task_type in ('I', 'A'):
            for wp in task_cfg['waypoints']:
                msg_wp = Waypoint()
                msg_wp.x, msg_wp.y, msg_wp.z = float(wp['x']), float(wp['y']), float(wp['z'])
                goal_msg.task.inspect.waypoints.append(msg_wp)
        elif task_type == 'D':
            goal_msg.task.deliver.tool_id = task_cfg['tool_id']
            goal_msg.task.deliver.human_target_id = task_cfg['human_target_id']

        return goal_msg

    def send_goal(self, task_cfg):
        try:
            goal_msg = self.build_goal(task_cfg)
        except (KeyError, ValueError) as e:
            self.get_logger().error(f"Task '{task_cfg.get('id', '?')}' is malformed: {e}")
            return

        if not self._action_client.wait_for_server(timeout_sec=self.arm_offboard_timeout_sec):
            self.get_logger().error(
                f"incoming_task_action unavailable, could not send task '{task_cfg['id']}'")
            return

        self.get_logger().info(f"Sending task {task_cfg['id']} to the planner")
        self._action_client.send_goal_async(goal_msg)

    def inject_wave(self, wave, wave_index):
        tasks = wave.get('tasks', [])
        self.get_logger().info(f'Injecting wave {wave_index} ({len(tasks)} tasks)')
        for task_cfg in tasks:
            self.send_goal(task_cfg)
            time.sleep(0.5)  # small gap so the planner processes each goal cleanly

    def run_mission(self, waves):
        """Schedules each wave at its configured delay (seconds since mission
        start), so tasks keep arriving over time instead of all at once."""
        if not waves:
            return
        mission_start = time.monotonic()
        for i, wave in enumerate(waves):
            target_time = mission_start + float(wave.get('delay', 0.0))
            remaining = target_time - time.monotonic()
            if remaining > 0:
                self.get_logger().info(f'Waiting {remaining:.1f}s for wave {i}')
                time.sleep(remaining)
            self.inject_wave(wave, i)

        self.get_logger().info('All mission waves have been injected.')


def main(args=None):
    rclpy.init(args=args)
    sequencer = MissionSequencer()

    sequencer.arm_and_offboard_all()

    waves = sequencer.load_mission_plan()
    sequencer.run_mission(waves)

    try:
        rclpy.spin(sequencer)
    except KeyboardInterrupt:
        pass

    sequencer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
