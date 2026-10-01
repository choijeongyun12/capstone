from __future__ import annotations

import struct
import threading
from typing import Optional

import cv2
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2

WINDOW = 'Carla Agent Monitor'


class Visualizer(Node):
    """2패널 OpenCV 창: 전방(차선+YOLO+전방거리) | 후방(YOLO+후방거리)"""

    DISPLAY_HZ = 30

    def __init__(self) -> None:
        super().__init__('visualizer')

        self.declare_parameter('panel_width',  640)
        self.declare_parameter('panel_height', 480)

        self._pw = self.get_parameter('panel_width').value
        self._ph = self.get_parameter('panel_height').value

        self._bridge = CvBridge()
        self._lock   = threading.Lock()
        self._front: np.ndarray | None = None
        self._rear:  np.ndarray | None = None
        self._front_dist: Optional[float] = None   # 전방 레이더 최근접 거리
        self._rear_dist:  Optional[float] = None   # 후방 레이더 최근접 거리

        self.create_subscription(Image,       '/carla/ego/front_det',   self._on_front,       qos_profile_sensor_data)
        self.create_subscription(Image,       '/carla/ego/rear_det',    self._on_rear,         qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/carla/ego/radar',       self._on_radar_front,  qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/carla/ego/radar_rear',  self._on_radar_rear,   qos_profile_sensor_data)

        total_w = self._pw * 2 + 2
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, total_w, self._ph)

        self.create_timer(1.0 / self.DISPLAY_HZ, self._display_loop)
        self.get_logger().info('Visualizer ready  (Q to quit)')

    # ------------------------------------------------------------------
    def _on_front(self, msg: Image) -> None:
        with self._lock:
            self._front = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_rear(self, msg: Image) -> None:
        with self._lock:
            self._rear = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def _on_radar_front(self, msg: PointCloud2) -> None:
        # vel < 0: 전방에서 접근하는 물체 (차가 다가가는 물체 포함)
        d = self._min_radar_depth(msg, vel_max=0.0)
        with self._lock:
            self._front_dist = d

    def _on_radar_rear(self, msg: PointCloud2) -> None:
        # vel < 0: 후방에서 따라오는 물체 (뒤에서 접근)
        d = self._min_radar_depth(msg, vel_max=0.0)
        with self._lock:
            self._rear_dist = d

    @staticmethod
    def _min_radar_depth(msg: PointCloud2, vel_max: float) -> Optional[float]:
        step, data = msg.point_step, msg.data
        min_d = float('inf')
        for i in range(msg.width):
            off   = i * step
            vel   = struct.unpack_from('f', data, off + 12)[0]
            depth = struct.unpack_from('f', data, off + 16)[0]
            if vel < vel_max and depth < min_d:
                min_d = depth
        return min_d if min_d < float('inf') else None

    # ------------------------------------------------------------------
    def _display_loop(self) -> None:
        with self._lock:
            front      = self._front.copy() if self._front is not None else None
            rear       = self._rear.copy()  if self._rear  is not None else None
            front_dist = self._front_dist
            rear_dist  = self._rear_dist

        p1 = self._panel(front, 'Front  |  Lane + YOLO', (0, 255, 0),   front_dist, label='전방')
        p2 = self._panel(rear,  'Rear   |  YOLO',        (0, 160, 255), rear_dist,  label='후방')

        div      = np.full((self._ph, 2, 3), 60, dtype=np.uint8)
        combined = np.hstack([p1, div, p2])
        cv2.imshow(WINDOW, combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('Q pressed — shutting down')
            rclpy.shutdown()

    def _panel(self, frame: np.ndarray | None, title: str,
               title_color: tuple,
               dist: Optional[float] = None,
               label: str = '') -> np.ndarray:
        if frame is None:
            panel = np.zeros((self._ph, self._pw, 3), dtype=np.uint8)
            cv2.putText(panel, 'Waiting...',
                        (self._pw // 4, self._ph // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (100, 100, 100), 2)
        else:
            panel = cv2.resize(frame, (self._pw, self._ph))

        # 상단 HUD 바
        cv2.rectangle(panel, (0, 0), (self._pw, 26), (15, 15, 15), cv2.FILLED)
        cv2.putText(panel, title, (8, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, title_color, 1)

        # 우측 하단 레이더 거리 표시
        if dist is not None:
            # 거리별 색상: 초록(안전) → 노랑(주의) → 빨강(위험)
            if dist > 20.0:
                dist_color = (0, 220, 0)
            elif dist > 10.0:
                dist_color = (0, 200, 220)
            else:
                dist_color = (0, 60, 255)

            dist_text = f'{label} {dist:.1f}m'
            (tw, th), _ = cv2.getTextSize(dist_text, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
            bx = self._pw - tw - 16
            by = self._ph - 16
            # 반투명 배경
            cv2.rectangle(panel, (bx - 6, by - th - 6), (self._pw - 4, by + 6),
                          (20, 20, 20), cv2.FILLED)
            cv2.putText(panel, dist_text, (bx, by),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, dist_color, 2, cv2.LINE_AA)
        else:
            no_text = f'{label} --'
            (tw, th), _ = cv2.getTextSize(no_text, cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)
            bx = self._pw - tw - 16
            by = self._ph - 16
            cv2.rectangle(panel, (bx - 6, by - th - 6), (self._pw - 4, by + 6),
                          (20, 20, 20), cv2.FILLED)
            cv2.putText(panel, no_text, (bx, by),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (80, 80, 80), 2, cv2.LINE_AA)

        return panel


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
