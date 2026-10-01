from __future__ import annotations
import math
import carla
import rclpy
from rclpy.node import Node

class SpectatorFollower(Node):
    def __init__(self) -> None:
        super().__init__('spectator_follower')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('role_name', 'ego_vehicle')
        self.declare_parameter('offset_x', -8.0)
        self.declare_parameter('offset_z', 5.0)
        self.declare_parameter('pitch', -15.0)
        self.declare_parameter('alpha_pos', 0.15)
        self.declare_parameter('alpha_rot', 0.10)
        
        client = carla.Client(self.get_parameter('host').value, self.get_parameter('port').value)
        client.set_timeout(10.0)
        self._world = client.get_world()
        self._vehicle = None
        self._prev_loc = None
        self._prev_yaw = None
        self._callback_id = None
        
        if not self._find_vehicle():
            self._retry = self.create_timer(1.0, self._retry_find)
        else:
            self._start_following()

    def _find_vehicle(self) -> bool:
        role = self.get_parameter('role_name').value
        for actor in self._world.get_actors():
            if actor.attributes.get('role_name') == role:
                self._vehicle = actor
                self.get_logger().info(f'Target found: {actor.id}')
                return True
        return False

    def _retry_find(self) -> None:
        if self._find_vehicle():
            self._retry.cancel()
            self._start_following()

    def _start_following(self) -> None:
        self._callback_id = self._world.on_tick(self._on_world_tick)

    def _on_world_tick(self, snapshot) -> None:
        if self._vehicle is None or not self._vehicle.is_alive:
            if self._callback_id:
                self._world.remove_on_tick(self._callback_id)
                self._callback_id = None
            self._vehicle = None
            self._retry = self.create_timer(1.0, self._retry_find)
            return
        
        vt = self._vehicle.get_transform()
        ap = self.get_parameter('alpha_pos').value
        ar = self.get_parameter('alpha_rot').value
        ox = self.get_parameter('offset_x').value
        oz = self.get_parameter('offset_z').value
        
        target_loc = vt.transform(carla.Location(x=ox, z=oz))
        target_yaw = vt.rotation.yaw
        
        if self._prev_loc is None:
            sl, sy = target_loc, target_yaw
        else:
            sl = carla.Location(x=self._lerp(self._prev_loc.x, target_loc.x, ap),
                                y=self._lerp(self._prev_loc.y, target_loc.y, ap),
                                z=self._lerp(self._prev_loc.z, target_loc.z, ap))
            sy = self._lerp_angle(self._prev_yaw, target_yaw, ar)
        
        self._prev_loc, self._prev_yaw = sl, sy
        self._world.get_spectator().set_transform(carla.Transform(sl, carla.Rotation(pitch=self.get_parameter('pitch').value, yaw=sy)))

    @staticmethod
    def _lerp(a, b, t): return a + (b - a) * t
    
    @staticmethod
    def _lerp_angle(a, b, t):
        d = ((b - a + 180) % 360) - 180
        return (a + d * t) % 360

def main(args=None):
    rclpy.init(args=args)
    node = SpectatorFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._callback_id:
            node._world.remove_on_tick(node._callback_id)
        node.destroy_node()
        rclpy.shutdown()
