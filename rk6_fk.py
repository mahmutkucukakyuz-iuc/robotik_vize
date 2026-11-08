#!/usr/bin/env python3
"""rk6 - 6 DoF robot kol icin DH tabanli duz kinematik (ROS'tan bagimsiz).

DH tablosu ogrencinin Fusion 360'ta tasarladigi koldan (Fusion export) turetildi
(yontem: rapor/rapor_taslak.md 1.1).

Standart DH: A_i = Rz(theta_i + offset_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
0T6 = A1 * A2 * ... * A6 (= URDF base_link -> tool0). Birimler: metre, radyan.
Sifir poz: kol dimdik yukari; pozitif q2/q3/q5 kolu +x (one) eger.
"""
import numpy as np

# (offset [rad], d [m], a [m], alpha [rad])
DH = [
    (0.0,         0.1313,   0.0,     -np.pi / 2),
    (-np.pi / 2,  0.0,      0.22995,  0.0),
    (np.pi / 2,   0.0652,   0.0,      np.pi / 2),
    (0.0,         0.22995,  0.0,     -np.pi / 2),
    (0.0,        -0.0660,   0.0,      np.pi / 2),
    (0.0,         0.20093,  0.0,      0.0),
]

def dh(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca,  st * sa, a * ct],
                     [st,  ct * ca, -ct * sa, a * st],
                     [0.0,      sa,       ca,      d],
                     [0.0,     0.0,      0.0,    1.0]])

def fk(q, upto=6):
    """q: 6 eklem acisi [rad]. upto eklemine kadar 0T_upto doner."""
    T = np.eye(4)
    for i in range(upto):
        off, d, a, al = DH[i]
        T = T @ dh(q[i] + off, d, a, al)
    return T

# Test (T) ve hareket (K) konfigurasyonlari, derece
# Limitler: J1 +-135, J2 +-90, J3 +-135, J4/J5/J6 +-90
CONFIGS = {
    "T1": [0, 0, 0, 0, 0, 0],
    "T2": [45, 20, 0, 0, 0, 0],
    "T3": [0, 30, 0, 0, 0, 0],
    "T4": [0, 0, 60, 0, 0, 0],
    "T5": [0, 0, 0, 0, 60, 0],
    "T6": [-45, 20, 45, 0, -30, 0],
    "T7": [60, -20, 70, 45, 30, 80],
    "T8": [-30, 45, 45, -60, 60, -45],
    "T9": [85, 10, 80, 30, -45, 30],
    "T10": [-75, -30, 100, -85, 85, 60],
    "K1": [45, 30, 60, 0, 60, 0],
    "K2": [-60, 20, 70, 45, 45, 60],
    "K3": [0, -20, 100, 0, 70, 0],
}

if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    print("Sifir konumda 0T6:\n", fk(np.zeros(6)), "\n")
    print(f"{'#':4} {'q [rad]':52} {'x':>7} {'y':>7} {'z':>7}")
    for name, deg in CONFIGS.items():
        q = np.radians(deg)
        p = fk(q)[:3, 3]
        print(f"{name:4} {np.array2string(q, precision=4):52} {p[0]:7.3f} {p[1]:7.3f} {p[2]:7.3f}")
