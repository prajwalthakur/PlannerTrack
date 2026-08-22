#!/usr/bin/env python3
# Author: Prajwal Thakur <prajwalthakur98@gmail.com>
"""
Watchdog that respawns an agent at its configured init pose/velocity
(agents.yaml's init* fields, applied via AgentModel::resetToInitialState())
whenever it drives off the scenario.

"Off the scenario" == outside the bounding box formed by the extremums of
every lane centerline in params.yaml's "Lanes" section (e.g. the four arm
tips of the intersection crossroad) -- not a hardcoded map size, so this
keeps working if the corridor geometry changes.

For each agent 1..num_agents:
  subscribes  /agent_<i>/odom   (nav_msgs/Odometry)
  publishes   /agent_<i>/reset  (std_msgs/Empty)   -- agent_sim's
                                                       AgentInterface::addAgents()
                                                       wires this straight to
                                                       AgentModel::resetToInitialState()

A per-agent latch avoids re-publishing reset on every odom tick while an
agent is still outside the bounds waiting for the reset to take effect --
it only re-arms once that agent is observed back inside them.
"""
import yaml

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


def load_ros_params(params_file):
    with open(params_file) as f:
        raw = yaml.safe_load(f)
    # params.yaml wraps everything under "/**" -> "ros__parameters"
    return raw.get('/**', {}).get('ros__parameters', raw)


def lane_extent_bound(ros_params):
    """Max |x| or |y| over every lane centerline endpoint -- the frame."""
    lanes = ros_params['Lanes']
    num_lanes = int(lanes['num_lanes'])
    bound = 0.0
    for i in range(1, num_lanes + 1):
        lane = lanes[f'lane_{i}']
        for point in (lane['start'], lane['end']):
            bound = max(bound, abs(float(point[0])), abs(float(point[1])))
    return bound


class ResetStateWatchdog(Node):
    def __init__(self):
        super().__init__('reset_state_watchdog')

        self.declare_parameter('params_file', '')
        params_file = self.get_parameter('params_file').get_parameter_value().string_value
        if not params_file:
            from ament_index_python.packages import get_package_share_directory
            params_file = get_package_share_directory('scenarios') + \
                '/intersection/params/params.yaml'

        ros_params = load_ros_params(params_file)
        self.bound = lane_extent_bound(ros_params)
        num_agents = int(ros_params['agent_interface']['num_agents'])

        self.get_logger().info(
            f'reset_state watchdog: bound=+/-{self.bound:.2f} m, '
            f'watching {num_agents} agent(s)')

        self._pending = {}
        self._reset_pubs = {}
        for i in range(1, num_agents + 1):
            self._pending[i] = False
            self._reset_pubs[i] = self.create_publisher(Empty, f'/agent_{i}/reset', 1)
            self.create_subscription(
                Odometry, f'/agent_{i}/odom', self._make_odom_cb(i), 10)

    def _make_odom_cb(self, agent_idx):
        def _cb(msg: Odometry):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            out_of_frame = abs(x) > self.bound or abs(y) > self.bound

            if out_of_frame and not self._pending[agent_idx]:
                self._pending[agent_idx] = True
                self.get_logger().warn(
                    f'agent_{agent_idx} left the frame at ({x:.2f}, {y:.2f}), resetting')
                self._reset_pubs[agent_idx].publish(Empty())
            elif not out_of_frame and self._pending[agent_idx]:
                self._pending[agent_idx] = False

        return _cb


def main():
    rclpy.init()
    node = ResetStateWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
