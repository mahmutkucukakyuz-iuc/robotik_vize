#!/usr/bin/env python3
"""fk_error_analysis: DH duz kinematigi ile ROS/Gazebo olcumlerini karsilastirir.

Her test konfigurasyonu (varsayilan T1..T10) icin:
  1) FollowJointTrajectory ile hedefe gonderir,
  2) /joint_states hedefe tol icinde oturana kadar bekler (+ ekstra bekleme),
  3) /ee_pose (TF base_link->tool0) ve Gazebo gercek pozu (gt_topic) icin
     N ornegin ortalamasini alir,
  4) DH FK (kinematics.fk) ile p_FK hesaplar; Oklid hatalari
     e = ||p_FK - p_olculen|| bulur,
  5) Tabloyu, ortalama ve std'yi yazar; CSV + Markdown kaydeder.

Parametreler:
  configs (str[])         ['T1',...,'T10']  (kinematics.CONFIGS anahtarlari)
  joint_names (str[]), action_name (str)
  move_duration_s (4.0), settle_tol_rad (0.005), settle_timeout_s (20.0),
  extra_wait_s (1.0), samples (20), sample_timeout_s (5.0)
  ee_topic ('/ee_pose')
  gt_topic ('/gz_link_poses', config/gz_bridge.yaml ile ayni), gt_msg_type ('tf' | 'pose' | 'none')
     'tf'  : tf2_msgs/TFMessage (gz PosePublisher Pose_V koprusu). Icinden
             gt_ee_frame (yoksa gt_ee_fallback_frame) ve gt_base_frame aranir;
             isimlerdeki 'model::' oneki yok sayilir. Base bulunamazsa pozlarin
             zaten base_link'e (model cercevesine) gore oldugu varsayilir.
     'pose': geometry_msgs/PoseStamped (base_link'e gore oldugu varsayilir)
  gt_base_frame ('base_link'), gt_ee_frame ('tool0'), gt_ee_fallback_frame ('link6'),
  gt_ee_offset (double[3], [0,0,0]) fallback cercevesinde tool0 ofseti [m]
  return_home (true), results_dir ('~/rk6_results')
"""
import csv
import os
import threading
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage

from rk6_control.common import (ACTION_NAME, TrajectoryClient, safe_shutdown,
                                start_spin_thread)
from rk6_control.kinematics import (CONFIGS, HOME, JOINT_NAMES, fk_position,
                                    quat_to_rot)


def _short(name):
    """'rk6::link6' veya 'rk6/link6' (ros_gz_bridge) -> 'link6'."""
    return name.replace('::', '/').split('/')[-1] if name else name


class FkErrorAnalysis(Node):
    def __init__(self):
        super().__init__('fk_error_analysis')
        p = self.declare_parameter
        self.configs = list(p('configs', [f'T{i}' for i in range(1, 11)]).value)
        self.joint_names = list(p('joint_names', JOINT_NAMES).value)
        action_name = p('action_name', ACTION_NAME).value
        self.move_duration = float(p('move_duration_s', 4.0).value)
        self.settle_tol = float(p('settle_tol_rad', 0.005).value)
        self.settle_timeout = float(p('settle_timeout_s', 20.0).value)
        self.extra_wait = float(p('extra_wait_s', 1.0).value)
        self.n_samples = int(p('samples', 20).value)
        self.sample_timeout = float(p('sample_timeout_s', 5.0).value)
        ee_topic = p('ee_topic', '/ee_pose').value
        gt_topic = p('gt_topic', '/gz_link_poses').value
        self.gt_topic = gt_topic
        self.gt_type = p('gt_msg_type', 'tf').value.lower()
        self.gt_base = p('gt_base_frame', 'base_link').value
        self.gt_ee = p('gt_ee_frame', 'tool0').value
        self.gt_fallback = p('gt_ee_fallback_frame', 'link6').value
        self.gt_offset = np.array(p('gt_ee_offset', [0.0, 0.0, 0.0]).value, dtype=float)
        self.return_home = bool(p('return_home', True).value)
        self.results_dir = os.path.expanduser(p('results_dir', '~/rk6_results').value)

        self.lock = threading.Lock()
        self.q_meas = None            # son eklem konumlari (joint_names sirasi)
        self.collect = False          # ornek toplama bayragi
        self.ee_samples = []
        self.gt_samples = []
        self._gt_warned = set()

        self.create_subscription(JointState, '/joint_states', self.on_joint, 50)
        self.create_subscription(PoseStamped, ee_topic, self.on_ee, 50)
        if self.gt_type == 'tf':
            self.create_subscription(TFMessage, gt_topic, self.on_gt_tf, 50)
        elif self.gt_type == 'pose':
            self.create_subscription(PoseStamped, gt_topic, self.on_gt_pose, 50)
        else:
            self.gt_type = 'none'
        self.traj = TrajectoryClient(self, self.joint_names, action_name)
        self.get_logger().info(f'ee_topic={ee_topic}, gt_topic={gt_topic} ({self.gt_type})')

    # ---------------- callback'ler ----------------
    def on_joint(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in self.joint_names):
            q = np.array([msg.position[idx[n]] for n in self.joint_names])
            with self.lock:
                self.q_meas = q

    def on_ee(self, msg):
        if self.collect:
            p = msg.pose.position
            with self.lock:
                self.ee_samples.append((p.x, p.y, p.z))

    def on_gt_pose(self, msg):
        if self.collect:
            p = msg.pose.position
            with self.lock:
                self.gt_samples.append((p.x, p.y, p.z))

    def _warn_once(self, key, text):
        if key not in self._gt_warned:
            self._gt_warned.add(key)
            self.get_logger().warning(text)

    def on_gt_tf(self, msg):
        if not self.collect:
            return
        poses = {}
        for tf in msg.transforms:
            t, r = tf.transform.translation, tf.transform.rotation
            poses[_short(tf.child_frame_id)] = (np.array([t.x, t.y, t.z]),
                                                quat_to_rot(r.x, r.y, r.z, r.w))
        offset = np.zeros(3)
        if self.gt_ee in poses:
            p_ee, R_ee = poses[self.gt_ee]
        elif self.gt_fallback in poses:
            p_ee, R_ee = poses[self.gt_fallback]
            offset = self.gt_offset
        else:
            self._warn_once('noee', f'GT mesajinda {self.gt_ee}/{self.gt_fallback} yok. '
                                    f'Mevcut: {sorted(poses)}')
            return
        p_ee = p_ee + R_ee @ offset
        if self.gt_base in poses:
            # base_link'e gore konum: R_b^T (p_ee - p_b)
            p_b, R_b = poses[self.gt_base]
            p_rel = R_b.T @ (p_ee - p_b)
        else:
            self._warn_once('nobase', f'GT mesajinda {self.gt_base} yok; pozlarin '
                                      'model (= base_link) cercevesine gore oldugu varsayiliyor')
            p_rel = p_ee
        with self.lock:
            self.gt_samples.append(tuple(p_rel))

    # ---------------- yardimcilar ----------------
    def wait_settled(self, q_target):
        """max|q_meas - q_target| < tol olana kadar bekler. (ok, max_hata) doner."""
        t0 = time.monotonic()
        err = float('inf')
        while rclpy.ok() and time.monotonic() - t0 < self.settle_timeout:
            with self.lock:
                q = None if self.q_meas is None else self.q_meas.copy()
            if q is not None:
                err = float(np.max(np.abs(q - q_target)))
                if err < self.settle_tol:
                    return True, err
            time.sleep(0.05)
        return False, err

    def sample(self):
        """ee/gt orneklerini toplar ve ortalamalarini dondurur (yoksa None)."""
        with self.lock:
            self.ee_samples, self.gt_samples = [], []
        self.collect = True
        t0 = time.monotonic()
        q_acc = []
        while rclpy.ok() and time.monotonic() - t0 < self.sample_timeout:
            with self.lock:
                n_ee, n_gt = len(self.ee_samples), len(self.gt_samples)
                if self.q_meas is not None:
                    q_acc.append(self.q_meas.copy())
            gt_ok = self.gt_type == 'none' or n_gt >= self.n_samples
            if n_ee >= self.n_samples and gt_ok:
                break
            time.sleep(0.02)
        self.collect = False
        with self.lock:
            ee = np.mean(self.ee_samples, axis=0) if self.ee_samples else None
            gt = np.mean(self.gt_samples, axis=0) if self.gt_samples else None
            n_ee, n_gt = len(self.ee_samples), len(self.gt_samples)
        q_mean = np.mean(q_acc, axis=0) if q_acc else None
        if n_ee == 0:
            self.get_logger().warning('/ee_pose ornegi gelmedi (ee_pose_publisher calisiyor mu?)')
        if self.gt_type != 'none' and n_gt == 0:
            self.get_logger().warning(
                f'{self.gt_topic} uzerinden Gazebo pozu gelmedi; e_GZ NaN olacak. '
                'gz_bridge.yaml ros_topic_name ve gt_topic parametresini kontrol edin '
                '(ros2 topic echo --once ' + self.gt_topic + ').')
        return ee, gt, q_mean, n_ee, n_gt

    # ---------------- ana akis ----------------
    def run(self):
        log = self.get_logger()
        if not self.traj.wait_for_server(60.0):
            log.error('Action sunucusu bulunamadi')
            return
        rows = []
        for name in self.configs:
            if not rclpy.ok():
                return
            if name not in CONFIGS:
                log.error(f'Bilinmeyen konfigurasyon: {name}')
                continue
            q_deg = np.array(CONFIGS[name], dtype=float)
            q = np.radians(q_deg)
            log.info(f'{name}: q={q_deg.tolist()} deg gonderiliyor')
            ok, msg = self.traj.move(q, self.move_duration)
            if not ok:
                log.warning(f'{name}: action sonucu {msg} (yine de oturma bekleniyor)')
            settled, jerr = self.wait_settled(q)
            if not settled:
                log.warning(f'{name}: oturmadi, max eklem hatasi {jerr:.4f} rad')
            time.sleep(self.extra_wait)
            ee, gt, q_m, n_ee, n_gt = self.sample()

            p_fk = fk_position(q)                       # komut acilarindan FK
            p_fk_meas = fk_position(q_m) if q_m is not None else None
            e_ros = float(np.linalg.norm(p_fk - ee)) if ee is not None else float('nan')
            e_gt = float(np.linalg.norm(p_fk - gt)) if gt is not None else float('nan')
            # FK(q_olculen) - Gazebo: eklem takip hatasindan arindirilmis model/simulasyon farki
            e_gt_meas = (float(np.linalg.norm(p_fk_meas - gt))
                         if (gt is not None and p_fk_meas is not None) else float('nan'))
            rows.append(dict(name=name, q_deg=q_deg, p_fk=p_fk, p_fk_meas=p_fk_meas,
                             ee=ee, gt=gt, e_ros=e_ros, e_gt=e_gt, e_gt_meas=e_gt_meas,
                             joint_err=jerr, n_ee=n_ee, n_gt=n_gt))
            log.info(f'{name}: p_FK={np.round(p_fk, 4)}  e_ros={e_ros * 1e3:.3f} mm  '
                     f'e_gz={e_gt * 1e3:.3f} mm  e_gz_meas={e_gt_meas * 1e3:.3f} mm  '
                     f'(n_ee={n_ee}, n_gt={n_gt})')

        if self.return_home:
            log.info('Home konumuna donuluyor')
            self.traj.move(HOME, self.move_duration)
        self.report(rows)

    def report(self, rows):
        if not rows:
            return
        e_ros = np.array([r['e_ros'] for r in rows])
        e_gt = np.array([r['e_gt'] for r in rows])
        e_gm = np.array([r['e_gt_meas'] for r in rows])

        def stats(e):
            e = e[~np.isnan(e)]
            if e.size == 0:
                return float('nan'), float('nan'), float('nan')
            std_s = float(np.std(e, ddof=1)) if e.size > 1 else 0.0
            return float(np.mean(e)), std_s, float(np.std(e))

        m_ros, s_ros, sp_ros = stats(e_ros)
        m_gt, s_gt, sp_gt = stats(e_gt)
        m_gm, s_gm, sp_gm = stats(e_gm)

        def fmt(v):
            return '   -   ' if v is None else ' '.join(f'{x:+.4f}' for x in v)

        lines = [f"{'#':4} {'p_FK [m]':24} {'p_ROS /ee_pose [m]':24} {'e_ROS[mm]':>9} "
                 f"{'p_GZ [m]':24} {'e_GZ[mm]':>9} {'e_GZm[mm]':>9}"]
        for r in rows:
            lines.append(f"{r['name']:4} {fmt(r['p_fk']):24} {fmt(r['ee']):24} "
                         f"{r['e_ros'] * 1e3:9.3f} {fmt(r['gt']):24} {r['e_gt'] * 1e3:9.3f} "
                         f"{r['e_gt_meas'] * 1e3:9.3f}")
        lines.append(f'e_ROS: ortalama={m_ros * 1e3:.4f} mm, std(n-1)={s_ros * 1e3:.4f} mm, '
                     f'std(n)={sp_ros * 1e3:.4f} mm')
        lines.append(f'e_GZ : ortalama={m_gt * 1e3:.4f} mm, std(n-1)={s_gt * 1e3:.4f} mm, '
                     f'std(n)={sp_gt * 1e3:.4f} mm   [FK(q_komut) - Gazebo]')
        lines.append(f'e_GZm: ortalama={m_gm * 1e3:.4f} mm, std(n-1)={s_gm * 1e3:.4f} mm, '
                     f'std(n)={sp_gm * 1e3:.4f} mm   [FK(q_olculen) - Gazebo]')
        print('\n'.join(lines), flush=True)

        os.makedirs(self.results_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        csv_path = os.path.join(self.results_dir, f'fk_error_{stamp}.csv')
        md_path = os.path.join(self.results_dir, f'fk_error_{stamp}.md')

        def xyz(v):
            return ['', '', ''] if v is None else [f'{x:.6f}' for x in v]

        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['config'] + [f'q{i}_deg' for i in range(1, 7)]
                       + ['fk_x', 'fk_y', 'fk_z', 'fkmeas_x', 'fkmeas_y', 'fkmeas_z',
                          'ros_x', 'ros_y', 'ros_z', 'gz_x', 'gz_y', 'gz_z',
                          'e_ros_m', 'e_gz_m', 'e_gz_meas_m', 'max_joint_err_rad',
                          'n_ros', 'n_gz'])
            for r in rows:
                w.writerow([r['name']] + [f'{v:g}' for v in r['q_deg']]
                           + xyz(r['p_fk']) + xyz(r['p_fk_meas']) + xyz(r['ee'])
                           + xyz(r['gt']) + [f"{r['e_ros']:.9f}", f"{r['e_gt']:.9f}",
                                             f"{r['e_gt_meas']:.9f}",
                                             f"{r['joint_err']:.6f}", r['n_ee'], r['n_gt']])

        def mdv(v):
            return '-' if v is None else ', '.join(f'{x:.4f}' for x in v)

        with open(md_path, 'w', encoding='utf-8') as f:
            f.write('| Konf. | q (deg) | p_FK (m) | p_ROS /ee_pose (m) | e_ROS (mm) '
                    '| p_Gazebo (m) | e_Gazebo (mm) | e_Gazebo, FK(q_olc) (mm) |\n')
            f.write('|---|---|---|---|---|---|---|---|\n')
            for r in rows:
                qs = ', '.join(f'{v:g}' for v in r['q_deg'])
                f.write(f"| {r['name']} | {qs} | {mdv(r['p_fk'])} | {mdv(r['ee'])} | "
                        f"{r['e_ros'] * 1e3:.4f} | {mdv(r['gt'])} | {r['e_gt'] * 1e3:.4f} | "
                        f"{r['e_gt_meas'] * 1e3:.4f} |\n")
            f.write(f'\n**e_ROS:** ortalama = {m_ros * 1e3:.4f} mm, '
                    f'std (n-1) = {s_ros * 1e3:.4f} mm\n\n')
            f.write(f'**e_Gazebo [FK(q_komut)]:** ortalama = {m_gt * 1e3:.4f} mm, '
                    f'std (n-1) = {s_gt * 1e3:.4f} mm\n\n')
            f.write(f'**e_Gazebo [FK(q_olculen)]:** ortalama = {m_gm * 1e3:.4f} mm, '
                    f'std (n-1) = {s_gm * 1e3:.4f} mm\n')
        self.get_logger().info(f'Kaydedildi: {csv_path} , {md_path}')


def main(args=None):
    rclpy.init(args=args)
    node = FkErrorAnalysis()
    executor, _ = start_spin_thread(node)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        safe_shutdown()


if __name__ == '__main__':
    main()
