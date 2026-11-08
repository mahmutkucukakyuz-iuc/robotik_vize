#!/usr/bin/env python3
"""motion_sequence: home -> K1 -> K2 -> K3 -> home (point-to-point kontrol).

FollowJointTrajectory action'i (/arm_controller/follow_joint_trajectory) ile
her hedefi gonderir ve bir sonrakine gecmeden once sonucunu bekler.

Parametreler (tum acilar rad, sureler s):
  joint_names   (str[])   joint1..joint6
  action_name   (str)     /arm_controller/follow_joint_trajectory
  home, k1, k2, k3 (double[6])  varsayilan: rk6_fk.py CONFIGS (radyana cevrilmis)
  sequence      (str[])   ['home','k1','k2','k3','home']
  durations     (double[]) her segmentin suresi; tek eleman verilirse hepsine uygulanir
  pause_s       (double)  hedefe varinca bekleme (1.0)
  repeat        (int)     dizinin tekrar sayisi (1)
  start_delay_s (double)  baslamadan once bekleme (0.0)
  use_gripper   (bool)    her hedefe varinca gripper komutu gonderilsin mi (true)
  gripper_action (str)    /gripper_controller/follow_joint_trajectory
  gripper_open, gripper_closed (double)  parmak konumu [m] (0.0235, 0.0); Fusion gripper: 0.0235 = export acikligi
  gripper_sequence (str[]) her adim icin 'open' | 'closed' | 'keep'
                          (varsayilan: open, closed, open, closed, open)
"""
import time

import rclpy
from rclpy.node import Node

from rk6_control.common import (ACTION_NAME, TrajectoryClient, safe_shutdown,
                                start_spin_thread)
from rk6_control.kinematics import HOME, JOINT_NAMES, config_rad


class MotionSequence(Node):
    def __init__(self):
        super().__init__('motion_sequence')
        p = self.declare_parameter
        self.joint_names = list(p('joint_names', JOINT_NAMES).value)
        action_name = p('action_name', ACTION_NAME).value
        self.targets = {
            'home': list(p('home', [float(v) for v in HOME]).value),
            'k1': list(p('k1', config_rad('K1')).value),
            'k2': list(p('k2', config_rad('K2')).value),
            'k3': list(p('k3', config_rad('K3')).value),
        }
        self.sequence = list(p('sequence', ['home', 'k1', 'k2', 'k3', 'home']).value)
        self.durations = list(p('durations', [3.0, 4.0, 4.0, 4.0, 4.0]).value)
        self.pause_s = float(p('pause_s', 1.0).value)
        self.repeat = int(p('repeat', 1).value)
        self.start_delay_s = float(p('start_delay_s', 0.0).value)
        self.traj = TrajectoryClient(self, self.joint_names, action_name)

        self.use_gripper = bool(p('use_gripper', True).value)
        self.gripper_pos = {
            'open': float(p('gripper_open', 0.0235).value),
            'closed': float(p('gripper_closed', 0.0).value),
        }
        self.gripper_sequence = list(
            p('gripper_sequence', ['open', 'closed', 'open', 'closed', 'open']).value)
        self.gripper = TrajectoryClient(
            self, ['gripper_left_finger_joint', 'gripper_right_finger_joint'],
            p('gripper_action', '/gripper_controller/follow_joint_trajectory').value)

    def gripper_step(self, i):
        """i. adimin gripper komutunu gonderir (her iki parmak ayni konuma)."""
        if not self.use_gripper or i >= len(self.gripper_sequence):
            return
        state = self.gripper_sequence[i]
        if state not in self.gripper_pos:
            return
        pos = self.gripper_pos[state]
        ok, msg = self.gripper.move([pos, pos], 1.0)
        self.get_logger().info(f'  gripper -> {state} ({pos:.3f} m): {msg}')

    def duration_for(self, i):
        if not self.durations:
            return 4.0
        return float(self.durations[i] if i < len(self.durations) else self.durations[-1])

    def run(self):
        log = self.get_logger()
        if self.start_delay_s > 0:
            time.sleep(self.start_delay_s)
        log.info(f'Action sunucusu bekleniyor: {self.traj.action_name}')
        if not self.traj.wait_for_server(60.0):
            log.error('Action sunucusu bulunamadi (arm_controller calisiyor mu?)')
            return False
        if self.use_gripper and not self.gripper.wait_for_server(5.0):
            log.warning('gripper_controller bulunamadi; gripper adimlari atlaniyor')
            self.use_gripper = False
        for r in range(self.repeat):
            for i, name in enumerate(self.sequence):
                if not rclpy.ok():
                    return False
                if name not in self.targets:
                    log.error(f'Bilinmeyen hedef: {name}')
                    return False
                q = self.targets[name]
                dur = self.duration_for(i)
                log.info(f'[{r + 1}/{self.repeat}] {i + 1}/{len(self.sequence)} -> {name.upper()} '
                         f'q=[{", ".join(f"{v:+.3f}" for v in q)}] rad, sure {dur:.1f} s')
                ok, msg = self.traj.move(q, dur)
                if not ok:
                    log.error(f'{name.upper()} hedefine gidilemedi: {msg}')
                    return False
                log.info(f'{name.upper()} ulasildi ({msg})')
                self.gripper_step(i)
                time.sleep(self.pause_s)
        log.info('Hareket dizisi tamamlandi.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MotionSequence()
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
