#!/usr/bin/env python3
"""pick_place: konumu bilinen kupleri gripper ile alip baska bir noktaya birakir (ters kinematik ile).

Her gorev icin adimlar (tum pozlar gripper_tcp = tool0 icin, base_link cercevesinde, gripper asagi bakar):
  1. gripper ac, kupun 'approach_h' (0.08 m; uzakta 0.06 / 0.045) ustune git (IK, eklem uzayi)
  2. kupe dik in (Kartezyen ara noktalar, her biri IK ile)
  3. gripper kapat, kaldir (dik)
  4. birakma noktasinin ustune git
  5. in, gripper ac, kalk
Sonda home'a doner. Kup konumlarini kamerayla bulan surum: scan_and_stack.

Parametreler (ortak olanlar icin bkz. manipulator.py):
  picks  (double[])  her gorev icin [x, y, yaw_deg] (kupun konumu ve yaw'i; base_link = world xy)
                     varsayilan: kirmizi (0.17, 0.27, 20.05), mavi (0.31, -0.15, -34.95)
  places (double[])  her gorev icin [x, y, yaw_deg] (birakma noktasi)
                     varsayilan: (0.28, 0.12, 0), (0.26, -0.04, 0)
  Not: robot dogrudan zeminde (ground_z = 0); taban plakasi 0.17 x 0.17 m (|x|, |y| < 0.085).
  Dik asagi kavrama erisimi zeminde r ~ 0.18..0.42 m, sinira rahat marjla r = 0.20..0.40 m
  (bkz. python3 -m rk6_manipulation.ik).
"""
import math

import rclpy

from rk6_manipulation.manipulator import Manipulator, run_node


class PickPlace(Manipulator):
    def __init__(self):
        super().__init__('pick_place')
        p = self.declare_parameter
        # Varsayilanlar rk6_pick_world.sdf'teki kirmizi ve mavi kupun konumlaridir (bilinen konum modu).
        # Kupler zeminde (ground_z = 0); birakma noktalari diger kuplerden uzak ve dik asagi
        # erisim alaninda (r ~ 0.26..0.31 m, sinira marj > 12 derece).
        self.picks = self._triples(p('picks', [0.17, 0.27, 20.05, 0.31, -0.15, -34.95]).value)
        self.places = self._triples(p('places', [0.28, 0.12, 0.0, 0.26, -0.04, 0.0]).value)

    @staticmethod
    def _triples(flat):
        flat = list(flat)
        return [tuple(flat[i:i + 3]) for i in range(0, len(flat) - len(flat) % 3, 3)]

    def pick_and_place(self, i, pick, place):
        z_grasp = self.ground_z + self.grasp_h
        (px, py, pyaw), (lx, ly, lyaw) = pick, place
        self.set_status(f'[Gorev {i}] kup ({px:.2f}, {py:.2f}) -> ({lx:.2f}, {ly:.2f})')

        z_up, q_up = self.approach_height(px, py, math.radians(pyaw))
        self.gripper(self.g_open, 'ac')
        self.goto(px, py, z_up, math.radians(pyaw), 'kupun ustu', n_sym=4, q=q_up)
        self.linear([px, py, z_grasp], 'inis')
        self.gripper(self.g_close, 'kapat')
        self.linear([px, py, z_up], 'kaldir')
        self.check_held()

        z_up, q_up = self.approach_height(lx, ly, math.radians(lyaw))
        self.goto(lx, ly, z_up, math.radians(lyaw), 'birakma ustu', n_sym=4, q=q_up)
        self.linear([lx, ly, z_grasp + 0.005], 'birakma inisi')
        self.gripper(self.g_open, 'ac (birak)')
        self.linear([lx, ly, z_up], 'geri cekil')

    def run(self):
        if not self.wait_ready():
            return False
        n = min(len(self.picks), len(self.places))
        self.get_logger().info(f'{n} al-birak gorevi baslatiliyor')
        try:
            for i in range(n):
                self.pick_and_place(i + 1, self.picks[i], self.places[i])
            self.go_home()
            self.gripper(0.0, 'kapat (home)')
            self.get_logger().info('Tum gorevler tamamlandi.')
            return True
        except RuntimeError as ex:
            self.get_logger().error(str(ex))
            return False


def main(args=None):
    rclpy.init(args=args)
    run_node(PickPlace())


if __name__ == '__main__':
    main()
