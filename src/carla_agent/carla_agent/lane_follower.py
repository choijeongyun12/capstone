from __future__ import annotations

import math
import os
import struct
import sys
import threading
from collections import deque
from enum import Enum, auto
from typing import List, Optional, Tuple

import time

import carla
import cv2
import numpy as np
import torch
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String


UFLD_V2_REPO  = '/home/tmo/ros2_ws/UltraFastLaneDetection_v2'
UFLD_V2_MODEL = '/home/tmo/ros2_ws/UltraFastLaneDetection_v2/weights/culane_res18.pth'

_V2_CFG = dict(
    backbone     = '18',
    num_grid_row = 200,
    num_cls_row  = 72,
    num_grid_col = 100,
    num_cls_col  = 81,
    num_lanes    = 4,
    use_aux      = False,
    input_height = 320,
    input_width  = 1600,
    fc_norm      = True,
    crop_ratio   = 0.6,
)


# ---------------------------------------------------------------------------
# PID
# ---------------------------------------------------------------------------

class PIDController:
    def __init__(self, Kp=0.8, Ki=0.01, Kd=0.2, i_limit=1.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.i_limit    = i_limit
        self.prev_error = 0.0
        self.integral   = 0.0
        self._init      = False

    def reset(self):
        self.prev_error = 0.0
        self.integral   = 0.0
        self._init      = False

    def compute(self, error, dt=1.0):
        dt = max(1e-3, float(dt))
        if not self._init:
            self.prev_error = float(error)
            self._init = True
        self.integral = np.clip(self.integral + float(error) * dt,
                                -self.i_limit, self.i_limit)
        deriv = (float(error) - self.prev_error) / dt
        self.prev_error = float(error)
        return self.Kp * float(error) + self.Ki * self.integral + self.Kd * deriv


# ---------------------------------------------------------------------------
# Lane change FSM
# ---------------------------------------------------------------------------

class LCState(Enum):
    HOLDING  = auto()
    CHANGING = auto()


class LaneChangeFSM:
    STEP = 0.04

    def __init__(self):
        self.state             = LCState.HOLDING
        self.direction         = 'center'
        self.transition_factor = 0.0

    def request(self, direction: str):
        if self.state == LCState.HOLDING:
            self.direction         = direction
            self.transition_factor = 0.0
            self.state             = LCState.CHANGING

    def step(self):
        if self.state == LCState.CHANGING:
            self.transition_factor = min(1.0, self.transition_factor + self.STEP)
            if self.transition_factor >= 1.0:
                self.state = LCState.HOLDING

    @property
    def is_changing(self):
        return self.state == LCState.CHANGING


# ---------------------------------------------------------------------------
# Lane geometry helpers
# ---------------------------------------------------------------------------

def _pick_ego_lane_boundaries(
    lanes: List[List[Tuple[int, int]]],
    image_width: int,
) -> Tuple[Optional[float], Optional[float]]:
    cx = image_width / 2.0
    bottoms = []
    for lane in lanes:
        if lane:
            bottoms.append(float(max(lane, key=lambda p: p[1])[0]))
    left_x  = max((x for x in bottoms if x < cx), default=None)
    right_x = min((x for x in bottoms if x > cx), default=None)
    return left_x, right_x


def _fit_poly(lane: List[Tuple[int, int]]) -> Optional[np.ndarray]:
    if len(lane) < 4:
        return None
    pts = sorted(lane, key=lambda p: p[1], reverse=True)
    y = np.array([p[1] for p in pts], dtype=np.float32)
    x = np.array([p[0] for p in pts], dtype=np.float32)
    if np.unique(y).size < 4:
        return None
    try:
        return np.polyfit(y, x, 2)
    except Exception:
        return None


def _pick_ego_lane_lines(
    lanes: List[List[Tuple[int, int]]],
    image_width: int,
) -> Tuple[Optional[List], Optional[List]]:
    cx = image_width / 2.0
    left_cands, right_cands = [], []
    for lane in lanes:
        if len(lane) < 4:
            continue
        bx = float(max(lane, key=lambda p: p[1])[0])
        if bx < cx:
            left_cands.append((bx, lane))
        elif bx > cx:
            right_cands.append((bx, lane))
    left_lane  = max(left_cands,  key=lambda t: t[0])[1] if left_cands  else None
    right_lane = min(right_cands, key=lambda t: t[0])[1] if right_cands else None
    return left_lane, right_lane


def _estimate_center_and_heading(
    lanes: List[List[Tuple[int, int]]],
    img_w: int,
    img_h: int,
) -> Tuple[float, float]:
    cx = img_w / 2.0
    left_lane, right_lane = _pick_ego_lane_lines(lanes, img_w)
    if left_lane is None or right_lane is None:
        lx, rx = _pick_ego_lane_boundaries(lanes, img_w)
        if lx is None or rx is None:
            return cx, 0.0
        return (lx + rx) / 2.0, 0.0
    lf, rf = _fit_poly(left_lane), _fit_poly(right_lane)
    if lf is None or rf is None:
        lx, rx = _pick_ego_lane_boundaries(lanes, img_w)
        if lx is None or rx is None:
            return cx, 0.0
        return (lx + rx) / 2.0, 0.0
    y_eval  = img_h * 0.82
    lx      = float(np.polyval(lf, y_eval))
    rx      = float(np.polyval(rf, y_eval))
    center  = (lx + rx) / 2.0
    ldx     = float(2 * lf[0] * y_eval + lf[1])
    rdx     = float(2 * rf[0] * y_eval + rf[1])
    heading = float(math.atan((ldx + rdx) / 2.0) / 0.8)
    return center, max(-1.0, min(1.0, heading))


# ---------------------------------------------------------------------------
# Visualization — called from inference thread (no ROS calls)
# ---------------------------------------------------------------------------

# 차선 인덱스별 색상: left=cyan, right=green
_LANE_COLORS = [(0, 210, 255), (0, 255, 80)]


def _make_lane_viz(
    frame: np.ndarray,
    lanes: List[List[Tuple[int, int]]],
    target_x: float,
    steer: float,
    speed_kph: float,
    state_label: str,
) -> np.ndarray:
    out     = frame.copy()
    overlay = frame.copy()
    h, w    = out.shape[:2]

    fitted: List[Optional[np.ndarray]] = []

    for i, lane in enumerate(lanes):
        color = _LANE_COLORS[i % len(_LANE_COLORS)]

        # ① 추출된 포인트 — 눈에 잘 띄는 색 원
        for pt in lane:
            cv2.circle(out, pt, 7, color, -1, cv2.LINE_AA)
            cv2.circle(out, pt, 9, (255, 255, 255), 1, cv2.LINE_AA)  # 흰 테두리

        # ② 포인트 연결 폴리라인
        if len(lane) >= 2:
            arr = np.array(sorted(lane, key=lambda p: p[1]), dtype=np.int32)
            cv2.polylines(out, [arr], False, color, 2, cv2.LINE_AA)

        # ③ 2차 피팅 곡선 (더 매끄러운 선)
        poly = _fit_poly(lane)
        fitted.append(poly)
        if poly is not None:
            ys = np.linspace(int(h * 0.38), h - 1, 120)
            xs = np.polyval(poly, ys)
            curve = np.array(
                [(int(x), int(y)) for x, y in zip(xs, ys) if 0 <= int(x) < w],
                dtype=np.int32,
            )
            if len(curve) >= 2:
                cv2.polylines(out, [curve], False, color, 3, cv2.LINE_AA)

    # ④ 차선 사이 반투명 영역
    if (len(fitted) == 2
            and fitted[0] is not None
            and fitted[1] is not None):
        ys  = np.linspace(int(h * 0.38), h - 1, 80)
        xl  = np.polyval(fitted[0], ys)
        xr  = np.polyval(fitted[1], ys)
        lp  = np.array([(int(x), int(y)) for x, y in zip(xl, ys) if 0 <= int(x) < w], dtype=np.int32)
        rp  = np.array([(int(x), int(y)) for x, y in zip(xr, ys) if 0 <= int(x) < w], dtype=np.int32)
        if len(lp) >= 2 and len(rp) >= 2:
            fill = np.vstack([lp, rp[::-1]])
            cv2.fillPoly(overlay, [fill], (0, 160, 70))

    cv2.addWeighted(out, 0.70, overlay, 0.30, 0, out)

    # ⑤ 이미지 중심선
    cv2.line(out, (w // 2, 0), (w // 2, h), (130, 130, 130), 1)

    # ⑥ 목표 중심선 (하단)
    tx = int(np.clip(target_x, 0, w - 1))
    cv2.line(out, (tx, h // 2), (tx, h), (0, 255, 0), 3, cv2.LINE_AA)
    cv2.circle(out, (tx, h - 35), 9, (0, 255, 0), -1, cv2.LINE_AA)

    # ⑦ 스티어링 게이지
    bar_cy = h - 14
    bar_cx = w // 2
    half_w = w // 5
    cv2.rectangle(out, (bar_cx - half_w, bar_cy - 7),
                  (bar_cx + half_w,  bar_cy + 7), (40, 40, 40), -1)
    fw = int(abs(steer) * half_w)
    if steer > 0:
        cv2.rectangle(out, (bar_cx, bar_cy - 7),
                      (min(bar_cx + fw, bar_cx + half_w), bar_cy + 7), (0, 160, 255), -1)
    elif steer < 0:
        cv2.rectangle(out, (max(bar_cx - fw, bar_cx - half_w), bar_cy - 7),
                      (bar_cx, bar_cy + 7), (255, 110, 0), -1)
    cv2.line(out, (bar_cx, bar_cy - 11), (bar_cx, bar_cy + 11), (255, 255, 255), 2)

    # ⑧ HUD 상단 바
    cv2.rectangle(out, (0, 0), (w, 28), (18, 18, 18), cv2.FILLED)
    cv2.putText(out,
                f'steer:{steer:+.3f}  spd:{speed_kph:.0f}kph  [{state_label}]',
                (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 0), 2)
    return out


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class LaneFollower(Node):
    CONTROL_HZ   = 20
    TARGET_KPH   = 45.0
    RADAR_SLOW   = 15.0
    RADAR_BRAKE  = 8.0
    SMOOTH_ALPHA = 0.05

    def __init__(self):
        super().__init__('lane_follower')

        self.declare_parameter('host',          '127.0.0.1')
        self.declare_parameter('port',          2000)
        self.declare_parameter('role_name',     'ego_vehicle')
        self.declare_parameter('target_kph',    self.TARGET_KPH)
        self.declare_parameter('radar_slow_m',  self.RADAR_SLOW)
        self.declare_parameter('radar_brake_m', self.RADAR_BRAKE)
        self.declare_parameter('heading_gain',  0.15)

        self.target_kph   = self.get_parameter('target_kph').value
        self.radar_slow   = self.get_parameter('radar_slow_m').value
        self.radar_brake  = self.get_parameter('radar_brake_m').value
        self.heading_gain = self.get_parameter('heading_gain').value

        self.create_subscription(Image,       '/carla/ego/front_camera',   self._on_image, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/carla/ego/radar',          self._on_radar, qos_profile_sensor_data)
        self.create_subscription(String,      '/carla/ego/lane_change',    self._on_lc,    10)
        self.create_subscription(String,      '/carla/ego/traffic_light',  self._on_tl,    10)

        self.viz_pub = self.create_publisher(Image, '/carla/ego/lane_viz', qos_profile_sensor_data)

        self._bridge    = CvBridge()
        self._min_radar = float('inf')

        # 카메라 프레임 공유 (콜백 → 추론 스레드)
        self._frame_lock  = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._frame_event = threading.Event()

        # 추론 결과 공유 (추론 스레드 → 제어 루프)
        self._result_lock  = threading.Lock()
        self._infer_lanes: List = []
        self._infer_frame: Optional[np.ndarray] = None

        # 시각화 이미지 공유 (추론 스레드 → ROS publish 타이머)
        self._viz_deque: deque = deque(maxlen=1)

        # 제어
        self.steer_pid = PIDController(Kp=0.15, Ki=0.0, Kd=1.0)
        self.speed_pid = PIDController(Kp=0.65, Ki=0.015, Kd=0.10)
        self.lc_fsm    = LaneChangeFSM()

        self._smooth_offset:  Optional[float] = None
        self._smooth_heading: Optional[float] = None

        # 신호등 상태
        self._tl_state = 'NONE'
        self._tl_stamp = 0.0
        self._TL_TIMEOUT = 2.0

        # 시각화 HUD용 최신 제어값
        self._last_steer       = 0.0
        self._last_speed_kph   = 0.0
        self._last_state_label = 'INIT'

        # UFLD v2
        self._net         = None
        self._row_anchor  = None
        self._device      = None
        self._num_grid_row = _V2_CFG['num_grid_row']
        self._num_cls_row  = _V2_CFG['num_cls_row']
        self._train_h      = _V2_CFG['input_height']
        self._train_w      = _V2_CFG['input_width']
        self._crop_ratio   = _V2_CFG['crop_ratio']
        self._resize_h     = int(self._train_h / self._crop_ratio)

        threading.Thread(target=self._init_ufld, daemon=True,
                         name='ufld_load').start()

        self._infer_thread = threading.Thread(
            target=self._inference_worker, daemon=True, name='ufld_infer')
        self._infer_thread.start()

        self.create_timer(1.0 / 30, self._viz_publish_timer)

        self.vehicle: Optional[carla.Vehicle] = None
        self._connect_to_carla()

        self.create_timer(1.0 / self.CONTROL_HZ, self._control_loop)
        self._last_state_label = 'LOADING'

    def _init_ufld(self):
        if not os.path.exists(UFLD_V2_REPO): return
        if not os.path.exists(UFLD_V2_MODEL): return
        if UFLD_V2_REPO not in sys.path: sys.path.insert(0, UFLD_V2_REPO)
        import types as _types
        for _m in ('nvidia', 'nvidia.dali', 'nvidia.dali.pipeline',
                   'nvidia.dali.types', 'nvidia.dali.fn',
                   'nvidia.dali.plugin', 'nvidia.dali.plugin.pytorch'):
            sys.modules.setdefault(_m, _types.ModuleType(_m))
        _stub = _types.ModuleType('data.dali_data')
        _stub.TrainCollect = None
        sys.modules.setdefault('data.dali_data', _stub)
        cwd = os.getcwd()
        os.chdir(UFLD_V2_REPO)
        try: from model.model_culane import parsingNet
        finally: os.chdir(cwd)
        self._device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        cfg = _V2_CFG
        net = parsingNet(pretrained=False, backbone=cfg['backbone'],
                         num_grid_row=cfg['num_grid_row'], num_cls_row=cfg['num_cls_row'],
                         num_grid_col=cfg['num_grid_col'], num_cls_col=cfg['num_cls_col'],
                         num_lane_on_row=cfg['num_lanes'], num_lane_on_col=cfg['num_lanes'],
                         use_aux=cfg['use_aux'], input_height=cfg['input_height'],
                         input_width=cfg['input_width'], fc_norm=cfg['fc_norm']).to(self._device)
        ckpt = torch.load(UFLD_V2_MODEL, map_location='cpu')
        sd = ckpt.get('model', ckpt)
        sd = {k[7:] if k.startswith('module.') else k: v for k, v in sd.items()}
        net.load_state_dict(sd, strict=False)
        net.eval()
        self._net = net
        self._row_anchor = np.linspace(0.42, 1.0, cfg['num_cls_row'])
        if self._device.type == 'cuda':
            dummy = torch.zeros(1, 3, self._train_h, self._train_w, device=self._device)
            with torch.no_grad(): self._net(dummy)
            torch.cuda.synchronize()

    def _preprocess(self, bgr: np.ndarray) -> torch.Tensor:
        resized = cv2.resize(bgr, (self._train_w, self._resize_h), interpolation=cv2.INTER_LINEAR)
        cropped = resized[-self._train_h:]
        rgb     = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        mean    = np.array([0.485, 0.456, 0.406], dtype=np.float32)
        std     = np.array([0.229, 0.224, 0.225], dtype=np.float32)
        rgb     = (rgb - mean) / std
        tensor  = torch.from_numpy(rgb.transpose(2, 0, 1)).unsqueeze(0)
        return tensor.to(self._device)

    def _run_ufld(self, bgr: np.ndarray) -> List[List[Tuple[int, int]]]:
        if self._net is None: return []
        img_h, img_w = bgr.shape[:2]
        inp = self._preprocess(bgr)
        with torch.no_grad(): pred = self._net(inp)
        max_idx_row = pred['loc_row'].argmax(1).cpu()
        exist_prob  = pred['exist_row'][0].softmax(0)[1].cpu()
        loc_row_cpu = pred['loc_row'].cpu()
        EXIST_THRESH, MIN_PTS = 0.30, 3
        coords: List[List[Tuple[int, int]]] = []
        for lane_i in [1, 2]:
            pts: List[Tuple[int, int]] = []
            for k in range(self._num_cls_row):
                if float(exist_prob[k, lane_i]) < EXIST_THRESH: continue
                mi = int(max_idx_row[0, k, lane_i])
                lo, hi = max(0, mi-1), min(self._num_grid_row-1, mi+1)
                all_ind = torch.arange(lo, hi + 1)
                weighted = (loc_row_cpu[0, all_ind, k, lane_i].softmax(0) * all_ind.float()).sum() + 0.5
                px = float(weighted) / (self._num_grid_row-1) * img_w
                py = self._row_anchor[k] * img_h
                pts.append((int(px), int(py)))
            if len(pts) >= MIN_PTS: coords.append(pts)
        return coords

    def _inference_worker(self):
        while True:
            try:
                got = self._frame_event.wait(timeout=0.1)
                self._frame_event.clear()
                if not got: continue
                with self._frame_lock: frame = self._latest_frame
                if frame is None: continue
                lanes = self._run_ufld(frame)
                with self._result_lock:
                    self._infer_lanes = lanes
                    self._infer_frame = frame
                img_w, img_h = frame.shape[1], frame.shape[0]
                lx, rx = _pick_ego_lane_boundaries(lanes, img_w)
                if lx is not None and rx is not None:
                    raw_center, _ = _estimate_center_and_heading(lanes, img_w, img_h)
                    target_x = raw_center
                else: target_x = img_w / 2.0
                viz = _make_lane_viz(frame, lanes, target_x, self._last_steer, self._last_speed_kph, self._last_state_label)
                self._viz_deque.append(viz)
            except Exception as e: pass

    def _viz_publish_timer(self):
        if not self._viz_deque: return
        msg = self._bridge.cv2_to_imgmsg(self._viz_deque[-1], encoding='bgr8')
        self.viz_pub.publish(msg)

    def _connect_to_carla(self):
        client = carla.Client(self.get_parameter('host').value, self.get_parameter('port').value)
        client.set_timeout(10.0)
        self._world = client.get_world()
        role = self.get_parameter('role_name').value
        for actor in self._world.get_actors():
            if actor.attributes.get('role_name') == role:
                self.vehicle = actor
                return
        self._retry = self.create_timer(1.0, self._retry_find)

    def _retry_find(self):
        role = self.get_parameter('role_name').value
        for actor in self._world.get_actors():
            if actor.attributes.get('role_name') == role:
                self.vehicle = actor
                self._retry.cancel()

    def _on_image(self, msg: Image):
        with self._frame_lock: self._latest_frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        self._frame_event.set()

    def _on_radar(self, msg: PointCloud2):
        min_d = float('inf')
        for i in range(msg.width):
            off = i * msg.point_step
            vel, depth = struct.unpack_from('ff', msg.data, off + 12)
            if vel < 0 and depth < min_d: min_d = depth
        self._min_radar = min_d

    def _on_lc(self, msg: String):
        d = msg.data.strip().lower()
        if d in ('left', 'right', 'center'): self.lc_fsm.request(d)

    def _on_tl(self, msg: String):
        state = msg.data.strip().upper()
        if state in ('RED', 'GREEN', 'YELLOW', 'NONE'):
            self._tl_state, self._tl_stamp = state, time.time()

    def _smooth(self, new_offset: float, new_heading: float):
        a = self.SMOOTH_ALPHA
        if self._smooth_offset is None:
            self._smooth_offset, self._smooth_heading = new_offset, new_heading
        else:
            self._smooth_offset = (1-a)*self._smooth_offset + a*new_offset
            self._smooth_heading = (1-a)*self._smooth_heading + a*new_heading
        return self._smooth_offset, self._smooth_heading

    def _reset_smooth(self): self._smooth_offset = self._smooth_heading = None

    def _control_loop(self):
        if self.vehicle is None or not self.vehicle.is_alive: return
        dt = 1.0 / self.CONTROL_HZ
        v_ms = self._vehicle_speed_ms()
        throttle = float(np.clip(self.speed_pid.compute(self.target_kph/3.6 - v_ms, dt), 0.0, 1.0))
        brake = 0.0
        if self._min_radar < self.radar_brake: throttle, brake = 0.0, 1.0
        elif self._min_radar < self.radar_slow: throttle *= (self._min_radar - self.radar_brake) / (self.radar_slow - self.radar_brake)
        if (time.time() - self._tl_stamp) < self._TL_TIMEOUT:
            if self._tl_state == 'RED': throttle, brake = 0.0, 1.0
            elif self._tl_state == 'YELLOW': throttle = min(throttle, 0.15)
        with self._result_lock:
            lanes, frame = list(self._infer_lanes), self._infer_frame
        if frame is not None and lanes:
            img_h, img_w = frame.shape[:2]
            lx, rx = _pick_ego_lane_boundaries(lanes, img_w)
            if lx is not None and rx is not None:
                raw_center, raw_heading = _estimate_center_and_heading(lanes, img_w, img_h)
                offset, heading = self._smooth((raw_center - img_w/2.0)/(img_w/2.0), raw_heading)
                target_offset = offset
                if self.lc_fsm.is_changing:
                    shift = ((rx-lx)/(img_w/2.0)) * self.lc_fsm.transition_factor
                    target_offset += (shift if self.lc_fsm.direction == 'right' else -shift)
                raw_steer = float(np.clip(self.steer_pid.compute(target_offset + self.heading_gain*heading, dt), -1.0, 1.0))
                
                # [안정화 핵심] 조향 변화율 제한 (Slew Rate Limit)
                # 이전 조향값에서 최대 0.02(약 2%) 이상 변하지 못하도록 강제 제한
                prev_steer = getattr(self, '_last_applied_steer', 0.0)
                max_delta = 0.02 
                steer = float(np.clip(raw_steer, prev_steer - max_delta, prev_steer + max_delta))
                self._last_applied_steer = steer
                detected = True
            else:
                steer = 0.0; self._last_applied_steer = 0.0
                self._reset_smooth()
                self.steer_pid.reset()
                detected = False
        else: steer, detected = 0.0, False
        self.lc_fsm.step()
        ctrl = carla.VehicleControl()
        ctrl.steer, ctrl.throttle, ctrl.brake = steer, throttle, brake
        self.vehicle.apply_control(ctrl)
        self._last_steer, self._last_speed_kph = steer, v_ms * 3.6
        self._last_state_label = ('LC→'+self.lc_fsm.direction if self.lc_fsm.is_changing else ('OK' if detected else 'NO LANE'))

    def _vehicle_speed_ms(self) -> float:
        v = self.vehicle.get_velocity()
        return (v.x**2 + v.y**2 + v.z**2)**0.5

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
