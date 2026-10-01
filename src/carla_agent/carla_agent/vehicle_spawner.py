from __future__ import annotations

import math
import struct
import threading
import time

import carla
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header


def _make_radar_cloud(header: Header, detections) -> PointCloud2:
    """Convert Carla radar detections to PointCloud2 (x,y,z,velocity,depth)."""
    fields = [
        PointField(name='x',        offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',        offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',        offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='velocity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='depth',    offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    point_step = 20  # 5 × float32
    buf = bytearray()
    for d in detections:
        x = d.depth * math.cos(d.altitude) * math.cos(d.azimuth)
        y = d.depth * math.cos(d.altitude) * math.sin(d.azimuth)
        z = d.depth * math.sin(d.altitude)
        buf += struct.pack('fffff', x, y, z, d.velocity, d.depth)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(detections)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * len(detections)
    msg.data = bytes(buf)
    msg.is_dense = True
    return msg


class VehicleSpawner(Node):
    def __init__(self) -> None:
        super().__init__('vehicle_spawner')

        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 2000)
        self.declare_parameter('timeout', 10.0)
        self.declare_parameter('map', 'Town04_Opt')
        self.declare_parameter('vehicle_filter', 'vehicle.tesla.model3')
        self.declare_parameter('role_name', 'ego_vehicle')
        # (x, y, z, pitch, yaw, roll) — 기존 프로젝트 기준 검증된 좌표
        self.declare_parameter('spawn_x',   -270.990)
        self.declare_parameter('spawn_y',     30.0)
        self.declare_parameter('spawn_z',      2.0)
        self.declare_parameter('spawn_pitch',  0.0)
        self.declare_parameter('spawn_yaw',    0.22)
        self.declare_parameter('spawn_roll',   0.0)
        self.declare_parameter('autopilot', False)
        self.declare_parameter('cam_width', 640)
        self.declare_parameter('cam_height', 480)
        self.declare_parameter('cam_fov', 90.0)
        self.declare_parameter('radar_range', 50.0)
        self.declare_parameter('radar_hfov', 30.0)
        self.declare_parameter('radar_vfov', 10.0)

        self.front_cam_pub = self.create_publisher(Image, '/carla/ego/front_camera',    qos_profile_sensor_data)
        self.rear_cam_pub  = self.create_publisher(Image, '/carla/ego/rear_camera',     qos_profile_sensor_data)
        self.ss_cam_pub    = self.create_publisher(Image, '/carla/ego/front_camera_ss', qos_profile_sensor_data)
        self.radar_pub      = self.create_publisher(PointCloud2, '/carla/ego/radar',      qos_profile_sensor_data)
        self.radar_rear_pub = self.create_publisher(PointCloud2, '/carla/ego/radar_rear', qos_profile_sensor_data)

        self._bridge  = CvBridge()
        self._actors: list = []
        self._lock    = threading.Lock()

        self._connect_and_spawn()

    # ------------------------------------------------------------------
    def _connect_and_spawn(self) -> None:
        host    = self.get_parameter('host').value
        port    = self.get_parameter('port').value
        timeout = self.get_parameter('timeout').value

        self.get_logger().info(f'Connecting to Carla at {host}:{port} …')
        client = carla.Client(host, port)
        client.set_timeout(timeout)

        world    = client.get_world()
        map_name = self.get_parameter('map').value
        if world.get_map().name.split('/')[-1] != map_name:
            self.get_logger().info(f'Loading map {map_name} …')
            world = client.load_world(map_name)
            time.sleep(3.0)
        self.world = world

        self._spawn_vehicle()
        self._attach_cameras()
        self._attach_radar()

        if self.get_parameter('autopilot').value:
            self.vehicle.set_autopilot(True)
            self.get_logger().info('Autopilot enabled')

    # 맵별 검증된 스폰 좌표 (x, y, z, pitch, yaw, roll)
    SPAWN_BY_MAP = {
        'Town04_Opt': (-270.990, 27.0,   2.0, 0.0, 0.22, 0.0),
        'Town06_Opt': (-115.990, 243.5,  0.0, 0.0, 0.0,  0.0),
        'IHP':        (-1139.0,    4.5,  1.0, 0.0, 0.0,  0.0),
        'k-track':    (  -75.6,  50.75, 2.0, 0.0, 0.0,  0.0),
    }

    def _spawn_vehicle(self) -> None:
        bp_lib     = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.find(self.get_parameter('vehicle_filter').value)
        vehicle_bp.set_attribute('role_name', self.get_parameter('role_name').value)

        map_name = self.get_parameter('map').value
        if map_name in self.SPAWN_BY_MAP:
            x, y, z, pitch, yaw, roll = self.SPAWN_BY_MAP[map_name]
            self.get_logger().info(f'Using preset spawn for {map_name}')
        else:
            x     = self.get_parameter('spawn_x').value
            y     = self.get_parameter('spawn_y').value
            z     = self.get_parameter('spawn_z').value
            pitch = self.get_parameter('spawn_pitch').value
            yaw   = self.get_parameter('spawn_yaw').value
            roll  = self.get_parameter('spawn_roll').value

        spawn_tf = carla.Transform(
            carla.Location(x=x, y=y, z=z),
            carla.Rotation(pitch=pitch, yaw=yaw, roll=roll),
        )

        self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_tf)
        self._actors.append(self.vehicle)
        self.get_logger().info(
            f'Spawned {self.vehicle.type_id} at ({x:.1f}, {y:.1f}, {z:.1f})'
        )

    def _attach_cameras(self) -> None:
        bp_lib = self.world.get_blueprint_library()
        w   = str(self.get_parameter('cam_width').value)
        h   = str(self.get_parameter('cam_height').value)
        fov = str(self.get_parameter('cam_fov').value)

        def _make_rgb(bp_lib):
            bp = bp_lib.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', w)
            bp.set_attribute('image_size_y', h)
            bp.set_attribute('fov', fov)
            return bp

        # 전방 RGB
        front_tf = carla.Transform(carla.Location(x=2.0, z=1.4))
        front_cam = self.world.spawn_actor(_make_rgb(bp_lib), front_tf, attach_to=self.vehicle)
        front_cam.listen(self._on_front_image)
        self._actors.append(front_cam)

        # 후방 RGB
        rear_tf = carla.Transform(carla.Location(x=-2.0, z=1.4), carla.Rotation(yaw=180.0))
        rear_cam = self.world.spawn_actor(_make_rgb(bp_lib), rear_tf, attach_to=self.vehicle)
        rear_cam.listen(self._on_rear_image)
        self._actors.append(rear_cam)

        # 전방 Semantic Segmentation (차선 검출용)
        ss_bp = bp_lib.find('sensor.camera.semantic_segmentation')
        ss_bp.set_attribute('image_size_x', w)
        ss_bp.set_attribute('image_size_y', h)
        ss_bp.set_attribute('fov', fov)
        ss_cam = self.world.spawn_actor(ss_bp, front_tf, attach_to=self.vehicle)
        ss_cam.listen(self._on_ss_image)
        self._actors.append(ss_cam)

        self.get_logger().info('Cameras attached — RGB front/rear + SS front (640×480, FOV 90°)')

    def _attach_radar(self) -> None:
        bp_lib   = self.world.get_blueprint_library()
        radar_bp = bp_lib.find('sensor.other.radar')
        radar_bp.set_attribute('range',            str(self.get_parameter('radar_range').value))
        radar_bp.set_attribute('horizontal_fov',   str(self.get_parameter('radar_hfov').value))
        radar_bp.set_attribute('vertical_fov',     str(self.get_parameter('radar_vfov').value))
        radar_bp.set_attribute('points_per_second', '1500')

        # 전방 레이더
        radar_tf = carla.Transform(carla.Location(x=2.5, z=1.0))
        radar    = self.world.spawn_actor(radar_bp, radar_tf, attach_to=self.vehicle)
        radar.listen(self._on_radar)
        self._actors.append(radar)

        # 후방 레이더 (yaw=180 — 뒤를 향함)
        radar_rear_tf = carla.Transform(carla.Location(x=-2.5, z=1.0),
                                        carla.Rotation(yaw=180.0))
        radar_rear    = self.world.spawn_actor(radar_bp, radar_rear_tf, attach_to=self.vehicle)
        radar_rear.listen(self._on_radar_rear)
        self._actors.append(radar_rear)

        self.get_logger().info(
            f'Radar attached (front+rear) — range {self.get_parameter("radar_range").value}m, '
            f'hFOV {self.get_parameter("radar_hfov").value}°'
        )

    # ------------------------------------------------------------------
    def _on_front_image(self, image) -> None:
        msg = self._carla_image_to_msg(image, 'front_camera')
        self.front_cam_pub.publish(msg)

    def _on_rear_image(self, image) -> None:
        msg = self._carla_image_to_msg(image, 'rear_camera')
        self.rear_cam_pub.publish(msg)

    def _on_ss_image(self, image) -> None:
        # SS 이미지는 R 채널에 클래스 ID가 담겨 있음 (mono8로 퍼블리시)
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        class_ids = arr[:, :, 2]   # Carla SS: BGRA 순서에서 R = class ID
        msg = self._bridge.cv2_to_imgmsg(class_ids, encoding='mono8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'front_camera_ss'
        self.ss_cam_pub.publish(msg)

    def _carla_image_to_msg(self, image, frame_id: str) -> Image:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(image.height, image.width, 4)
        bgr = arr[:, :, :3]
        msg = self._bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        return msg

    def _on_radar(self, radar_data) -> None:
        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'radar'
        self.radar_pub.publish(_make_radar_cloud(header, radar_data))

    def _on_radar_rear(self, radar_data) -> None:
        header = Header()
        header.stamp    = self.get_clock().now().to_msg()
        header.frame_id = 'radar_rear'
        self.radar_rear_pub.publish(_make_radar_cloud(header, radar_data))

    # ------------------------------------------------------------------
    def destroy_node(self) -> None:
        self.get_logger().info('Cleaning up Carla actors …')
        for actor in reversed(self._actors):
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception:
                pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleSpawner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
