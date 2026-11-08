#!/usr/bin/env python3
"""ik_goto: gripper_tcp'yi base_link'e gore verilen noktaya ters kinematikle goturur.

Ornek:
  ros2 run rk6_manipulation ik_goto --ros-args -p use_sim_time:=true -p x:=0.3 -p y:=0.1 -p z:=0.05
  ros2 run rk6_manipulation ik_goto --ros-args -p use_sim_time:=true -p mode:=rpy \
      -p x:=0.35 -p y:=0.0 -p z:=0.25 -p roll_deg:=0.0 -p pitch_deg:=90.0 -p yaw_deg:=0.0

Not: robot dogrudan zeminde (zemin = base_link z = 0). 'down' modunda (gripper dik asagi)
joint5 siniri yuzunden tcp en fazla z ~ 0.14 m (r = 0.175..0.30 m), r = 0.35'te ~0.10 m
yukseklige cikabilir; zemin seviyesinde (z = 0.017) r = 0.18..0.42 m erisilir. Daha yuksek
hedefler icin 'rpy' modu kullanin (or. x=0.35 z=0.25 pitch 90: gripper yatay, one bakar).
gripper_tcp == tool0 (parmak pad ortasi), bu yuzden frame:=tcp ve frame:=tool0 ayni sonucu verir.

Parametreler:
  x, y, z (double, m)   hedef konum (base_link cercevesinde)
  mode (str)            'down' = gripper asagi bakar (yukaridan kavrama); 'rpy' = roll/pitch/yaw ile tam yonelim
  yaw_deg (double)      'down' modunda gripper'in dikey eksen etrafindaki acisi (varsayilan: hedefe dogru)
  roll_deg, pitch_deg   'rpy' modunda yonelim (R = Rz(yaw) Ry(pitch) Rx(roll))
  frame (str)           'tcp' (parmak ortasi, varsayilan) veya 'tool0' (ikisi ayni nokta)
  duration_s (double)   hareket suresi (3.0)
  dry_run (bool)        true ise yalnizca cozumu yazar, robotu hareket ettirmez
"""
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from rk6_control.common import (ACTION_NAME, TrajectoryClient, safe_shutdown,
                                start_spin_thread)
from rk6_control.kinematics import JOINT_NAMES
from rk6_manipulation.ik import TCP_OFFSET, fk_tcp, ik, ik_down


def rpy_matrix(r, p, y):
    """R = Rz(y) Ry(p) Rx(r)."""
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


class IkGoto(Node):
    def __init__(self):
        super().__init__('ik_goto')
        p = self.declare_parameter
        self.target = [float(p('x', 0.30).value), float(p('y', 0.0).value),
                       float(p('z', 0.10).value)]
        self.mode = p('mode', 'down').value
        self.yaw_deg = p('yaw_deg', float('nan')).value
        self.roll_deg = float(p('roll_deg', 0.0).value)
        self.pitch_deg = float(p('pitch_deg', 0.0).value)
        self.tcp = TCP_OFFSET if p('frame', 'tcp').value == 'tcp' else 0.0
        self.duration = float(p('duration_s', 3.0).value)
        self.dry_run = bool(p('dry_run', False).value)
        self.q_meas = None
        self.lock = threading.Lock()
        self.create_subscription(JointState, '/joint_states', self.on_joint, 10)
        self.traj = TrajectoryClient(self, JOINT_NAMES, ACTION_NAME)

    def on_joint(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in JOINT_NAMES):
            with self.lock:
                self.q_meas = np.array([msg.position[idx[n]] for n in JOINT_NAMES])

    def run(self):
        log = self.get_logger()
        t0 = time.monotonic()
        while self.q_meas is None and time.monotonic() - t0 < 10.0:
            time.sleep(0.05)
        q0 = self.q_meas if self.q_meas is not None else np.zeros(6)
        x, y, z = self.target
        if self.mode == 'rpy':
            yaw_deg = 0.0 if math.isnan(self.yaw_deg) else float(self.yaw_deg)
            T_d = np.eye(4)
            T_d[:3, :3] = rpy_matrix(*np.radians([self.roll_deg, self.pitch_deg, yaw_deg]))
            T_d[:3, 3] = [x, y, z]
            q, ok, ep, er = ik(T_d, q0=q0, tcp=self.tcp)
        else:
            yaw = math.atan2(y, x) if math.isnan(self.yaw_deg) else math.radians(self.yaw_deg)
            q, ok, ep, er = ik_down(x, y, z, yaw, q0=q0, tcp=self.tcp)
        p_chk = fk_tcp(q, self.tcp)[:3, 3]
        log.info(f'Hedef ({x:.3f}, {y:.3f}, {z:.3f}) m, mod={self.mode}')
        log.info(f'IK {"basarili" if ok else "BASARISIZ"}: q = '
                 f'[{", ".join(f"{v:+.3f}" for v in q)}] rad '
                 f'([{", ".join(f"{math.degrees(v):+.1f}" for v in q)}] derece)')
        log.info(f'FK kontrolu: p = ({p_chk[0]:.4f}, {p_chk[1]:.4f}, {p_chk[2]:.4f}) m, '
                 f'konum hatasi {ep * 1000:.3f} mm, yonelim hatasi {er:.1e} rad')
        if not ok:
            log.error('Hedef erisilemez (eklem sinirlari/erisim alani). Robot hareket ettirilmedi.')
            return False
        if self.dry_run:
            return True
        if not self.traj.wait_for_server(30.0):
            log.error('arm_controller action sunucusu yok')
            return False
        ok, msg = self.traj.move(q, self.duration)
        log.info(f'Hareket: {msg}')
        return ok


def main(args=None):
    rclpy.init(args=args)
    node = IkGoto()
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
