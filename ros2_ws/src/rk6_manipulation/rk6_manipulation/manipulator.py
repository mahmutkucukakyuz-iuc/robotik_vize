#!/usr/bin/env python3
"""pick_place ve scan_and_stack icin ortak temel sinif: eklem durumu takibi, gripper,
IK ile noktaya gitme, Kartezyen duz cizgi hareketi ve canli kamera penceresi.

Tum pozlar gripper_tcp icindir (gripper_tcp == tool0 = parmak pad'lerinin ortasi),
base_link cercevesinde, gripper asagi bakar.
Kamera penceresi (show_camera:=true, varsayilan) gorev ayri bir thread'de calisirken ana
thread'de acilir: bilek kamerasinin canli goruntusu, algilanan kupler ve o anki adim gosterilir.

Ortak parametreler (varsayilanlar):
  ground_z (0.0 m)     kuplerin durdugu zemin, base_link'te (robot dogrudan zeminde: taban
                       plakasinin alt yuzu base_link z = 0 = zemin; kaide yok)
  grasp_h (0.017 m)    kavramada tcp'nin zeminden yuksekligi (kup merkezi 0.015). Pad carpisma
                       kutusu link6'da z = 0.125..0.161, tool0 z = 0.15013 -> pad uclari tcp'nin
                       0.011 m altinda, zeminden ~6 mm yukarida; pad kupu 6..30 mm arasinda tutar.
                       Gripper govdesi (link6 kutusu) tcp'nin >= 0.055 m ustunde kalir.
  approach_h (0.08 m)  kavrama noktasinin ustunde yaklasma yuksekligi (erisilemezse 0.06 / 0.045;
                       dik asagi bakan gripper'la tcp z = 0.097, r <= ~0.37 m'de erisilir)
  gripper_open (0.0235 m)  parmak basina acik konum (pad ic yuzleri arasi ~47 mm)
  gripper_close (0.006 m)  pad ic yuzu y = +-g'de -> 3 cm kup parmak basina g = 0.015'te temas
                       eder; 0.006 hedefi parmak basina ~9 mm sikar (eklem effort siniri 20 N)
  held_min (0.010 m)   kapattiktan sonra iki parmak da bundan acik kaldiysa nesne tutuluyor
                       (temas 0.015, bos kapanma 0.006 -> esik arada)
  segment_s (3.0 s), linear_s (2.0 s), waypoints (6), show_camera (true)
  max_joint_speed (1.0 rad/s)  eklem uzayi hareketlerinde ortalama hiz siniri: uzun hareketlerde
                       sure uzatilir (tepe hiz ~1.5 rad/s < URDF siniri 2 rad/s)
"""
import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState

from rk6_control.common import ACTION_NAME, TrajectoryClient, safe_shutdown, start_spin_thread
from rk6_control.kinematics import HOME, JOINT_NAMES
from rk6_manipulation import perception
from rk6_manipulation.ik import down_pose, fk_tcp, ik, ik_down, tcp_yaw

WINDOW = 'rk6 bilek kamerasi'

GRIPPER_JOINTS = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
GRIPPER_ACTION = '/gripper_controller/follow_joint_trajectory'


class TaskAborted(Exception):
    """Ctrl+C ile ROS kapandi; gorev thread'i sessizce biter (RuntimeError degil, adim atlanmaz)."""


class Manipulator(Node):
    def __init__(self, name):
        super().__init__(name)
        p = self.declare_parameter
        self.ground_z = float(p('ground_z', 0.0).value)
        self.grasp_h = float(p('grasp_h', 0.017).value)
        self.approach_h = float(p('approach_h', 0.08).value)
        self.g_open = float(p('gripper_open', 0.0235).value)
        self.g_close = float(p('gripper_close', 0.006).value)
        self.held_min = float(p('held_min', 0.010).value)
        self.segment_s = float(p('segment_s', 3.0).value)
        self.linear_s = float(p('linear_s', 2.0).value)
        self.n_way = int(p('waypoints', 6).value)
        self.max_joint_speed = float(p('max_joint_speed', 1.0).value)
        self.show_camera = bool(p('show_camera', True).value)

        self.lock = threading.Lock()
        self.q_arm = None
        self.q_grip = None
        self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        self.arm = TrajectoryClient(self, JOINT_NAMES, ACTION_NAME)
        self.grip = TrajectoryClient(self, GRIPPER_JOINTS, GRIPPER_ACTION)
        self.q = None          # planlanan son eklem durumu (bir sonraki IK icin baslangic)
        self.status = 'baslatiliyor'

        # Bilek kamerasi (pencere ve scan_and_stack'in algilamasi icin)
        self.bridge = CvBridge()
        self.img_lock = threading.Lock()
        self.image = None
        self.img_count = 0
        self.create_subscription(Image, '/wrist_camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)

    def on_image(self, msg):
        with self.img_lock:
            self.image = msg
            self.img_count += 1

    def latest_image(self):
        with self.img_lock:
            msg = self.image
        return None if msg is None else self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def fresh_image(self, n_new=5, timeout=5.0):
        """Cagridan sonra gelen n_new'inci yeni goruntuyu dondurur (robot durduktan sonra)."""
        with self.img_lock:
            start = self.img_count
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            with self.img_lock:
                if self.img_count - start >= n_new:
                    return self.bridge.imgmsg_to_cv2(self.image, 'bgr8')
            time.sleep(0.02)
        return None

    def set_status(self, text):
        self.status = text
        self.get_logger().info(text)

    def on_joint(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        with self.lock:
            if all(n in idx for n in JOINT_NAMES):
                self.q_arm = np.array([msg.position[idx[n]] for n in JOINT_NAMES])
            if all(n in idx for n in GRIPPER_JOINTS):
                self.q_grip = np.array([msg.position[idx[n]] for n in GRIPPER_JOINTS])

    def wait_ready(self):
        """joint_states ve iki action sunucusu hazir olana kadar bekler."""
        log = self.get_logger()
        t0 = time.monotonic()
        while self.q_arm is None and time.monotonic() - t0 < 15.0:
            time.sleep(0.05)
        if self.q_arm is None:
            log.error('/joint_states gelmiyor')
            return False
        if not (self.arm.wait_for_server(30.0) and self.grip.wait_for_server(30.0)):
            log.error('arm_controller / gripper_controller action sunuculari yok')
            return False
        self.q = self.q_arm.copy()
        return True

    # ---------------- hareket yardimcilari ----------------
    @staticmethod
    def _alive():
        """Ctrl+C ile ROS kapatildiysa gorevi durdur (kalan adimlari tek tek denemesin)."""
        if not rclpy.ok():
            raise TaskAborted()

    def gripper(self, pos, label, duration=1.5):
        self._alive()
        self.status = f'gripper {label}'
        ok, msg = self.grip.move([pos, pos], duration)
        time.sleep(0.5)
        self.get_logger().info(f'  gripper {label} ({pos:.3f} m): {msg}')

    def solve(self, x, y, z, yaw, n_sym=2, tilt=0.0):
        q, ok, ep, _ = ik_down(x, y, z, yaw, q0=self.q, n_sym=n_sym, tilt=tilt)
        if not ok:
            raise RuntimeError(f'IK cozumu yok ({x:.3f}, {y:.3f}, {z:.3f})')
        return q, ep

    def approach_height(self, x, y, yaw, n_sym=4):
        """Kavrama noktasinin ustunde erisilebilen en yuksek yaklasma yuksekligi.
        Dik asagi bakan gripper'in erisimi joint5 siniriyla kisitli (r = 0.40'ta tcp
        en fazla z ~ 0.07); uzak nesnelerde 6 ve 4.5 cm denenir (4.5 cm'de bile pad uclari
        3 cm'lik kupun ~2 cm ustundedir)."""
        z_grasp = self.ground_z + self.grasp_h
        for h in sorted({self.approach_h, 0.06, 0.045}, reverse=True):
            if h > self.approach_h:
                continue
            try:
                q, _ = self.solve(x, y, z_grasp + h, yaw, n_sym)
                return z_grasp + h, q
            except RuntimeError:
                continue
        raise RuntimeError(f'({x:.3f}, {y:.3f}) erisim disinda (yaklasma pozu bulunamadi)')

    def move_time(self, q, duration=None):
        """Hareket suresi: istenen sure, ama uzun eklem hareketlerinde (or. J1 220 derece) uzatilir.
        Tek noktali JTC yorungesinde tepe hiz ~1.5 x ortalama hiz; ortalama max_joint_speed ile
        sinirlanir ki tepe hiz URDF hiz sinirini (2 rad/s) asmasin (asarsa ros2_control limitleyicisi
        komutu kirpar ve "out of limits" hatasi yazar)."""
        t = self.segment_s if duration is None else duration
        if self.q is not None:
            t = max(t, float(np.max(np.abs(np.asarray(q) - self.q))) / self.max_joint_speed)
        return t

    def goto(self, x, y, z, yaw, label, n_sym=2, duration=None, q=None, tilt=0.0):
        """Eklem uzayinda hedef poza git (q verilmisse IK atlanir; tilt: bkz. ik.down_pose)."""
        self._alive()
        self.status = label
        ep = 0.0
        if q is None:
            q, ep = self.solve(x, y, z, yaw, n_sym, tilt)
        ok, msg = self.arm.move(q, self.move_time(q, duration))
        self.get_logger().info(f'  {label}: ({x:.3f}, {y:.3f}, {z:.3f}) e_IK={ep * 1000:.2f} mm -> {msg}')
        if not ok:
            raise RuntimeError(f'{label}: hareket basarisiz ({msg})')
        self.q = q

    def linear(self, p_to, label, duration=None):
        """Mevcut tcp pozundan p_to'ya duz cizgide (yonelim sabit) git: ara noktalar IK ile."""
        self._alive()
        self.status = label
        p_from = fk_tcp(self.q)[:3, 3]
        yaw = tcp_yaw(self.q)
        duration = self.linear_s if duration is None else duration
        pts, times, q = [], [], self.q
        for k in range(1, self.n_way + 1):
            s = 0.5 - 0.5 * math.cos(math.pi * k / self.n_way)   # yumusak baslangic/bitis
            p = (1 - s) * p_from + s * np.asarray(p_to)
            T_d = down_pose(p[0], p[1], p[2], yaw)
            q_new, ok, _, _ = ik(T_d, q0=q, use_seeds=False)
            if not ok:
                # Onceki durustan yakinsamadi: diger baslangic noktalarini da dene, ama yalnizca
                # onceki durusa yakin bir cozumu kabul et (yolun ortasinda kol durusu sicramasin)
                q_new, ok, _, _ = ik(T_d, q0=q, use_seeds=True)
                if not ok or np.max(np.abs(q_new - q)) > 0.6:
                    raise RuntimeError(f'{label}: ara nokta {k} icin IK yok')
            q = q_new
            pts.append(q)
            times.append(duration * k / self.n_way)
        ok, msg = self.arm.move_path(pts, times)
        self.get_logger().info(f'  {label}: -> ({p_to[0]:.3f}, {p_to[1]:.3f}, {p_to[2]:.3f}) {msg}')
        if not ok:
            raise RuntimeError(f'{label}: hareket basarisiz ({msg})')
        self.q = q

    def finger_state(self):
        with self.lock:
            return None if self.q_grip is None else self.q_grip.copy()

    def held_offset(self):
        """Tutulan nesnenin tcp'ye gore y ekseni boyunca kaymasi [m].
        Sol parmak link6 +y, sag parmak -y yonunde acilir (0 = pad'ler tcp'de birlesik);
        sol pad ic yuzu y = +g_sol, sag pad ic yuzu y = -g_sag'da, nesne merkezi (g_sol - g_sag) / 2'dedir."""
        g = self.finger_state()
        return 0.0 if g is None else float(g[0] - g[1]) / 2.0

    def holding(self):
        """Iki parmak da held_min'den acik mi (nesne parmaklari durdurduysa kapali hedefe inemez)."""
        g = self.finger_state()
        return g is not None and bool(np.all(g > self.held_min))

    def check_held(self):
        g = self.finger_state()
        if g is None:
            return True
        held = self.holding()
        self.get_logger().info(f'  parmaklar {np.round(g, 4)} m -> '
                               f'{"nesne tutuluyor" if held else "UYARI: nesne tutulmuyor olabilir"}')
        return held

    def release_safely(self):
        """Bir adim basarisiz olursa: elde nesne varsa havada birakma, once yere indir."""
        holding = self.holding()
        if holding and self.q is not None:
            p = fk_tcp(self.q)[:3, 3]
            try:
                self.linear([p[0], p[1], self.ground_z + self.grasp_h + 0.003], 'nesneyi yere indir')
            except RuntimeError as ex:
                self.get_logger().warning(f'  nesne yere indirilemedi: {ex}')
        self.gripper(self.g_open, 'ac')
        if holding and self.q is not None:
            p = fk_tcp(self.q)[:3, 3]
            try:
                self.linear([p[0], p[1], p[2] + 0.05], 'geri cekil')
            except RuntimeError:
                pass

    def go_home(self):
        self._alive()
        self.status = 'home'
        ok, msg = self.arm.move(HOME, self.move_time(HOME))
        self.q = np.array(HOME, dtype=float)
        self.get_logger().info(f'Home: {msg}')

    # ---------------- canli kamera penceresi ----------------
    def camera_frame(self):
        """Canli goruntu + algilanan kupler + o anki adim yazisi."""
        img = self.latest_image()
        if img is None:
            img = np.full((480, 640, 3), 40, np.uint8)
            cv2.putText(img, 'kamera goruntusu bekleniyor...', (150, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            return img
        blobs = perception.detect_blobs(img)
        img = perception.annotate(img, blobs, [b['color'] for b in blobs])
        cv2.rectangle(img, (0, 0), (img.shape[1], 26), (0, 0, 0), -1)
        cv2.putText(img, self.status, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (255, 255, 255), 1)
        return img


def run_node(node):
    """Gorevi (node.run) ayri thread'de calistirir; ana thread'de kamera penceresini gosterir.
    show_camera:=false ise ya da ekran (DISPLAY) yoksa pencere acilmaz."""
    executor, _ = start_spin_thread(node)
    gui = node.show_camera and bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))

    def task():
        try:
            node.run()
        except TaskAborted:
            pass    # Ctrl+C: gorev yarida birakildi

    worker = threading.Thread(target=task, daemon=True)
    try:
        worker.start()
        if gui:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, 640, 480)
            while worker.is_alive() and rclpy.ok():
                cv2.imshow(WINDOW, node.camera_frame())
                if cv2.waitKey(50) & 0xFF == ord('q'):    # pencere kapanir, gorev surer
                    gui = False
                    cv2.destroyAllWindows()
                    break
        worker.join()
        if gui and rclpy.ok():
            node.status = 'tamamlandi (kapatmak icin bir tusa basin)'
            cv2.imshow(WINDOW, node.camera_frame())
            cv2.waitKey(3000)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        executor.shutdown()
        node.destroy_node()
        safe_shutdown()
