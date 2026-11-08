#!/usr/bin/env python3
"""visualizer: sensor verilerini gorsellestirir.

Abonelikler:
  /joint_states              sensor_msgs/JointState -> 6 eklem acisi vs zaman (matplotlib)
  /wrist_camera/image_raw    sensor_msgs/Image      -> OpenCV penceresi (varsa)
  /ee_pose                   geometry_msgs/PoseStamped -> grafikte xyz metni
Parametreler:
  joint_topic, image_topic, ee_topic (str), joint_names (str[]),
  window_s (double, 20.0 s gosterilen zaman araligi), show_camera (bool, true),
  refresh_hz (double, 15.0)
rclpy arka plan thread'inde doner; GUI (plt.pause / cv2.waitKey) ana thread'de.
"""
import threading
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, JointState

import matplotlib.pyplot as plt

from rk6_control.common import safe_shutdown, start_spin_thread
from rk6_control.kinematics import JOINT_NAMES

try:
    import cv2
except ImportError:  # kamera penceresi opsiyonel
    cv2 = None
try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None


def image_to_bgr(msg):
    """cv_bridge yoksa yaygin kodlamalar icin elle donusum."""
    h, w, enc = msg.height, msg.width, msg.encoding.lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(h, msg.step)
    if enc in ('rgb8', 'bgr8'):
        img = buf[:, :w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == 'rgb8' else img.copy()
    if enc in ('rgba8', 'bgra8'):
        img = buf[:, :w * 4].reshape(h, w, 4)[:, :, :3]
        return img[:, :, ::-1].copy() if enc == 'rgba8' else img.copy()
    if enc in ('mono8', '8uc1'):
        return buf[:, :w].copy()
    raise ValueError(f'desteklenmeyen kodlama: {msg.encoding}')


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')
        p = self.declare_parameter
        joint_topic = p('joint_topic', '/joint_states').value
        image_topic = p('image_topic', '/wrist_camera/image_raw').value
        ee_topic = p('ee_topic', '/ee_pose').value
        self.joint_names = list(p('joint_names', JOINT_NAMES).value)
        self.window_s = float(p('window_s', 20.0).value)
        self.show_camera = bool(p('show_camera', True).value) and cv2 is not None
        self.refresh_hz = float(p('refresh_hz', 15.0).value)

        self.lock = threading.Lock()
        self.t0 = None
        self.t = deque(maxlen=5000)
        self.q = [deque(maxlen=5000) for _ in self.joint_names]
        self.ee = None
        self.image = None
        self.bridge = CvBridge() if CvBridge is not None else None

        self.create_subscription(JointState, joint_topic, self.on_joint, 10)
        self.create_subscription(PoseStamped, ee_topic, self.on_ee, 10)
        if self.show_camera:
            self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        elif cv2 is None:
            self.get_logger().warning('OpenCV yok, kamera penceresi kapali')

    # ---- callback'ler (arka plan thread) ----
    def on_joint(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in self.joint_names):
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp == 0.0:
            stamp = time.monotonic()
        with self.lock:
            if self.t0 is None or (self.t and stamp - self.t0 < self.t[-1] - 1e-9):
                # ilk mesaj ya da sim zamani sifirlandi
                self.t0 = stamp
                self.t.clear()
                for d in self.q:
                    d.clear()
            self.t.append(stamp - self.t0)
            for k, n in enumerate(self.joint_names):
                self.q[k].append(msg.position[idx[n]])

    def on_ee(self, msg):
        p = msg.pose.position
        with self.lock:
            self.ee = (p.x, p.y, p.z)

    def on_image(self, msg):
        try:
            if self.bridge is not None:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                img = image_to_bgr(msg)
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warning(f'Goruntu donusturulemedi: {ex}', throttle_duration_sec=5.0)
            return
        with self.lock:
            self.image = img

    # ---- GUI (ana thread) ----
    def run_gui(self):
        plt.ion()
        fig, ax = plt.subplots(figsize=(9, 5))
        fig.canvas.manager.set_window_title('rk6 - eklem acilari')
        lines = [ax.plot([], [], label=n)[0] for n in self.joint_names]
        ax.set_xlabel('zaman [s]')
        ax.set_ylabel('eklem acisi [rad]')
        ax.set_title('/joint_states')
        ax.grid(True)
        ax.legend(loc='upper left', fontsize=8)
        ee_text = fig.text(0.5, 0.01, 'EE: bekleniyor...', ha='center', family='monospace')
        fig.tight_layout(rect=(0, 0.04, 1, 1))
        period = 1.0 / self.refresh_hz

        while rclpy.ok() and plt.fignum_exists(fig.number):
            with self.lock:
                t = np.array(self.t)
                qs = [np.array(d) for d in self.q]
                ee = self.ee
                img = self.image
                self.image = None
            if t.size:
                for ln, qv in zip(lines, qs):
                    ln.set_data(t, qv)
                t_end = t[-1]
                ax.set_xlim(max(0.0, t_end - self.window_s), max(self.window_s, t_end))
                allq = np.concatenate(qs)
                lo, hi = float(allq.min()), float(allq.max())
                pad = max(0.1, 0.1 * (hi - lo))
                ax.set_ylim(lo - pad, hi + pad)
            if ee is not None:
                ee_text.set_text(f'EE (base_link): x={ee[0]:+.4f}  y={ee[1]:+.4f}  '
                                 f'z={ee[2]:+.4f} m')
            if img is not None and self.show_camera:
                cv2.imshow('wrist_camera', img)
            if self.show_camera:
                cv2.waitKey(1)
            plt.pause(period)
        if self.show_camera:
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    executor, _ = start_spin_thread(node)
    try:
        node.run_gui()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        safe_shutdown()


if __name__ == '__main__':
    main()
