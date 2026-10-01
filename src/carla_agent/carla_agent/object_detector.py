from __future__ import annotations

import threading
from collections import Counter, deque
from typing import Optional, Tuple

import cv2
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

YOLO_MODEL = '/home/tmo/yolov8n.pt'

# COCO 클래스 필터: 9=traffic light 추가
_CLASSES = {0, 1, 2, 3, 5, 7, 9}
_COLORS  = {
    0: (0,   230, 230),   # person
    1: (30,  180, 255),   # bicycle
    2: (255, 180,  30),   # car
    3: (200,   0, 220),   # motorcycle
    5: (0,   200, 100),   # bus
    7: (80,   80, 255),   # truck
    9: (0,   220, 220),   # traffic light (색상은 상태에 따라 덮어씀)
}
_NAMES = {0:'person', 1:'bicycle', 2:'car', 3:'motorcycle',
          5:'bus', 7:'truck', 9:'traffic_light'}

# 신호등 색상별 BGR
_TL_BGR = {'RED': (0, 0, 255), 'YELLOW': (0, 220, 255), 'GREEN': (0, 255, 60)}

# 박스 면적 최소값 (너무 멀면 색 판별 불안정)
_TL_MIN_AREA = 200


class ObjectDetector(Node):
    """전·후방 카메라 YOLO 검출 + 신호등 색상 분류 → /carla/ego/traffic_light 퍼블리시"""

    CONF = 0.40

    def __init__(self) -> None:
        super().__init__('object_detector')

        self._bridge = CvBridge()
        self._model: Optional[YOLO] = None

        self._front_lock    = threading.Lock()
        self._rear_lock     = threading.Lock()
        self._lane_viz_lock = threading.Lock()
        self._latest_front:    Optional[np.ndarray] = None
        self._latest_rear:     Optional[np.ndarray] = None
        self._latest_lane_viz: Optional[np.ndarray] = None
        self._frame_event = threading.Event()

        self._front_buf: deque = deque(maxlen=1)
        self._rear_buf:  deque = deque(maxlen=1)
        # 신호등 상태 버퍼 (다수결로 flicker 억제)
        self._tl_buf:    deque = deque(maxlen=5)
        self._skip_count = 0

        self.create_subscription(
            Image, '/carla/ego/front_camera', self._on_front, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/carla/ego/lane_viz', self._on_lane_viz, qos_profile_sensor_data)
        self.create_subscription(
            Image, '/carla/ego/rear_camera', self._on_rear, qos_profile_sensor_data)

        self._front_pub = self.create_publisher(
            Image, '/carla/ego/front_det', qos_profile_sensor_data)
        self._rear_pub  = self.create_publisher(
            Image, '/carla/ego/rear_det',  qos_profile_sensor_data)
        self._tl_pub    = self.create_publisher(
            String, '/carla/ego/traffic_light', 10)

        threading.Thread(target=self._load_model, daemon=True,
                         name='yolo_load').start()
        threading.Thread(target=self._inference_worker, daemon=True,
                         name='yolo_infer').start()

        self.create_timer(1.0 / 30, self._publish_timer)
        self.get_logger().info('ObjectDetector ready — YOLOv8n loading...')

    # ------------------------------------------------------------------
    def _load_model(self) -> None:
        self._model = YOLO(YOLO_MODEL)
        self._model.to('cpu')
        dummy = np.zeros((480, 640, 3), dtype=np.uint8)
        self._model(dummy, verbose=False, conf=self.CONF, device='cpu')
        self.get_logger().info('YOLOv8n loaded and warmed up (CPU)')

    # ------------------------------------------------------------------
    def _on_front(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._front_lock:
            self._latest_front = frame
        self._frame_event.set()

    def _on_rear(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._rear_lock:
            self._latest_rear = frame
        self._frame_event.set()

    def _on_lane_viz(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        with self._lane_viz_lock:
            self._latest_lane_viz = frame

    # ------------------------------------------------------------------
    def _inference_worker(self) -> None:
        while True:
            try:
                got = self._frame_event.wait(timeout=0.1)
                self._frame_event.clear()
                if not got or self._model is None:
                    continue

                self._skip_count += 1
                if self._skip_count < 3:
                    continue
                self._skip_count = 0

                with self._front_lock:
                    front = self._latest_front
                with self._rear_lock:
                    rear = self._latest_rear
                with self._lane_viz_lock:
                    lane_viz = self._latest_lane_viz

                frames = [f for f in (front, rear) if f is not None]
                if not frames:
                    continue

                results = self._model(
                    frames, verbose=False,
                    conf=self.CONF, classes=sorted(_CLASSES), device='cpu',
                )

                idx = 0
                if front is not None:
                    base = lane_viz.copy() if lane_viz is not None else front.copy()
                    annotated, tl_state = self._draw(base, results[idx], 'Front')
                    self._front_buf.append(annotated)
                    self._tl_buf.append(tl_state)
                    idx += 1
                if rear is not None:
                    annotated, _ = self._draw(rear.copy(), results[idx], 'Rear')
                    self._rear_buf.append(annotated)

            except Exception as exc:
                try:
                    self.get_logger().error(f'[yolo_infer] {exc}')
                except Exception:
                    pass

    # ------------------------------------------------------------------
    @staticmethod
    def _classify_tl(img: np.ndarray, x1: int, y1: int, x2: int, y2: int) -> str:
        """바운딩박스 내부 HSV 색상 분석으로 신호등 상태 판별."""
        crop = img[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
        if crop.size == 0:
            return 'UNKNOWN'
        hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)

        r1 = cv2.inRange(hsv, np.array([0,  120, 100]), np.array([10,  255, 255]))
        r2 = cv2.inRange(hsv, np.array([160, 120, 100]), np.array([180, 255, 255]))
        mask_r = cv2.bitwise_or(r1, r2)
        mask_y = cv2.inRange(hsv, np.array([15, 120, 100]), np.array([35,  255, 255]))
        mask_g = cv2.inRange(hsv, np.array([40, 100,  80]), np.array([90,  255, 255]))

        cnt = {
            'RED':    int(mask_r.sum() / 255),
            'YELLOW': int(mask_y.sum() / 255),
            'GREEN':  int(mask_g.sum() / 255),
        }
        best_state, best_cnt = max(cnt.items(), key=lambda kv: kv[1])
        return best_state if best_cnt >= 10 else 'UNKNOWN'

    def _draw(self, img: np.ndarray, result, label: str) -> Tuple[np.ndarray, str]:
        """바운딩박스 그리기 + 신호등 상태 반환."""
        h, w = img.shape[:2]
        n = 0
        frame_tl = 'NONE'

        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf   = float(box.conf[0])
            x1, y1, x2, y2 = map(int, box.xyxy[0])

            if cls_id == 9:  # 신호등
                area = (x2 - x1) * (y2 - y1)
                if area >= _TL_MIN_AREA:
                    state = self._classify_tl(img, x1, y1, x2, y2)
                    if state in _TL_BGR:
                        color = _TL_BGR[state]
                        frame_tl = state
                    else:
                        color = _COLORS[9]
                        state = 'UNKNOWN'
                    tag = f'TL:{state} {conf:.2f}'
                else:
                    color = _COLORS[9]
                    tag = f'traffic_light {conf:.2f}'
            else:
                color = _COLORS.get(cls_id, (200, 200, 200))
                tag   = f'{_NAMES.get(cls_id, str(cls_id))} {conf:.2f}'

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2, cv2.LINE_AA)
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.50, 1)
            cv2.rectangle(img,
                          (x1, max(y1 - th - 6, 0)),
                          (x1 + tw + 6, y1), color, cv2.FILLED)
            cv2.putText(img, tag, (x1 + 3, y1 - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 0, 0), 1, cv2.LINE_AA)
            n += 1

        cv2.rectangle(img, (0, 0), (w, 26), (18, 18, 18), cv2.FILLED)
        cv2.putText(img, f'YOLO  {label}  |  {n} objects',
                    (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 1)
        return img, frame_tl

    # ------------------------------------------------------------------
    def _publish_timer(self) -> None:
        if self._front_buf:
            self._front_pub.publish(
                self._bridge.cv2_to_imgmsg(self._front_buf[-1], encoding='bgr8'))
        if self._rear_buf:
            self._rear_pub.publish(
                self._bridge.cv2_to_imgmsg(self._rear_buf[-1], encoding='bgr8'))

        # 신호등 상태: 최근 5프레임 다수결
        if self._tl_buf:
            dominant = Counter(self._tl_buf).most_common(1)[0][0]
            msg = String()
            msg.data = dominant
            self._tl_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
