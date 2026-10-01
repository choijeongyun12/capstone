from __future__ import annotations

import struct
import time

import carla
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


class AutopilotNode(Node):
    """Radar-based safety layer on top of Carla's built-in autopilot.

    Normal driving is handled by the Traffic Manager (autopilot=True).
    This node monitors the radar and takes over control when an approaching
    obstacle is detected within safe_distance. Autopilot is re-enabled once
    the path is clear.
    """

    def __init__(self) -> None:
        super().__init__('autopilot_node')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('role_name', 'ego_vehicle')
        self.declare_parameter('safe_distance',  15.0)   # m — start slowing
        self.declare_parameter('brake_distance',  8.0)   # m — full emergency brake
        self.declare_parameter('vehicle_search_timeout', 30.0)

        self.safe_dist  = self.get_parameter('safe_distance').value
        self.brake_dist = self.get_parameter('brake_distance').value

        self.vehicle: carla.Vehicle | None = None
        self._autopilot_on = True   # tracks what we last set

        self._connect_to_carla()

        self.radar_sub   = self.create_subscription(
            PointCloud2, '/carla/ego/radar', self._on_radar, qos_profile_sensor_data
        )
        self.min_dist_pub = self.create_publisher(Float32, '/carla/ego/min_radar_dist', 10)

        # Retry finding the vehicle every second until timeout
        self._search_start = self.get_clock().now()
        if self.vehicle is None:
            self._search_timer = self.create_timer(1.0, self._find_vehicle)

    # ------------------------------------------------------------------
    def _connect_to_carla(self) -> None:
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        client = carla.Client(host, port)
        client.set_timeout(10.0)
        self._world = client.get_world()
        self._find_vehicle()

    def _find_vehicle(self) -> None:
        role = self.get_parameter('role_name').value
        for actor in self._world.get_actors():
            if actor.attributes.get('role_name') == role:
                self.vehicle = actor
                self.get_logger().info(f'Found vehicle: {actor.type_id}  (id={actor.id})')
                if hasattr(self, '_search_timer'):
                    self._search_timer.cancel()
                return

        elapsed = (self.get_clock().now() - self._search_start).nanoseconds / 1e9
        timeout = self.get_parameter('vehicle_search_timeout').value
        if elapsed > timeout:
            self.get_logger().error(
                f'Vehicle with role_name="{role}" not found after {timeout}s. '
                'Is vehicle_spawner running?'
            )
            if hasattr(self, '_search_timer'):
                self._search_timer.cancel()

    # ------------------------------------------------------------------
    def _on_radar(self, msg: PointCloud2) -> None:
        if self.vehicle is None or not self.vehicle.is_alive:
            return

        min_depth = self._parse_min_approaching_depth(msg)

        dist_msg = Float32()
        dist_msg.data = min_depth if min_depth < float('inf') else -1.0
        self.min_dist_pub.publish(dist_msg)

        if min_depth < self.brake_dist:
            self._apply_emergency_brake()
        elif min_depth < self.safe_dist:
            ratio = (min_depth - self.brake_dist) / (self.safe_dist - self.brake_dist)
            self._apply_gradual_brake(ratio)
        else:
            self._restore_autopilot()

    def _parse_min_approaching_depth(self, msg: PointCloud2) -> float:
        """Return the closest depth among detections with negative (approaching) velocity."""
        # Fields: x(0) y(4) z(8) velocity(12) depth(16) — each float32
        point_step = msg.point_step
        data       = msg.data
        min_depth  = float('inf')

        for i in range(msg.width):
            off      = i * point_step
            velocity = struct.unpack_from('f', data, off + 12)[0]
            depth    = struct.unpack_from('f', data, off + 16)[0]
            if velocity < 0 and depth < min_depth:
                min_depth = depth

        return min_depth

    def _apply_emergency_brake(self) -> None:
        if self._autopilot_on:
            self.vehicle.set_autopilot(False)
            self._autopilot_on = False

        ctrl = carla.VehicleControl()
        ctrl.brake    = 1.0
        ctrl.throttle = 0.0
        self.vehicle.apply_control(ctrl)
        self.get_logger().warn(
            f'Emergency brake applied — obstacle within {self.brake_dist}m'
        )

    def _apply_gradual_brake(self, clear_ratio: float) -> None:
        """clear_ratio: 0.0 = near brake_dist, 1.0 = near safe_dist."""
        if self._autopilot_on:
            self.vehicle.set_autopilot(False)
            self._autopilot_on = False

        ctrl = self.vehicle.get_control()
        ctrl.brake    = (1.0 - clear_ratio) * 0.6
        ctrl.throttle = ctrl.throttle * clear_ratio
        self.vehicle.apply_control(ctrl)

    def _restore_autopilot(self) -> None:
        if not self._autopilot_on:
            self.vehicle.set_autopilot(True)
            self._autopilot_on = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutopilotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
