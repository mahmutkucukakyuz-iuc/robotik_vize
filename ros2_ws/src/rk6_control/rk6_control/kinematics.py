#!/usr/bin/env python3
"""rk6 - 6 DoF robot kol icin DH tabanli duz kinematik (ROS'tan bagimsiz).

Depo kokundeki rk6_fk.py dosyasinin paket ici kopyasi (+ yardimci fonksiyonlar).
DH tablosu ogrencinin gercek kolundan (Fusion export) turetildi
(yontem: rapor/rapor_taslak.md 1.1).

Standart DH: A_i = Rz(theta_i + offset_i) * Tz(d_i) * Tx(a_i) * Rx(alpha_i)
base_link_T_tool0 = A1 * A2 * ... * A6 (sabit base/tool donusumu yok: ikisi de birim).
Birimler: metre, radyan. URDF q_i == DH theta_i, tool0 == DH 6. cerceve.
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

# Eklem limitleri [rad] (URDF / ros2_control ile ayni)
LIMITS = [(-2.356194, 2.356194), (-1.570796, 1.570796), (-2.356194, 2.356194),
          (-1.570796, 1.570796), (-1.570796, 1.570796), (-1.570796, 1.570796)]

JOINT_NAMES = [f'joint{i}' for i in range(1, 7)]


def dh(theta, d, a, alpha):
    """Tek bir DH satirinin 4x4 homojen donusumu (i-1)T_i."""
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


def fk_position(q):
    """Uc-efektor konumu 0r_P = 0T6 [0 0 0 1]^T (ilk 3 eleman) [m]."""
    return fk(q)[:3, 3].copy()


# Test (T) ve hareket (K) konfigurasyonlari, derece.
# Limitler icinde, zemine/tabana carpmayan (robot dogrudan zeminde).
# Sifir poz dimdik yukari oldugu icin T/K'lar kolu one/yana egerek calisma uzayini tarar.
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

HOME = [0.0] * 6


def config_rad(name):
    """CONFIGS icindeki bir konfigurasyonu radyan listesi olarak dondurur."""
    return [float(v) for v in np.radians(CONFIGS[name])]


# ---------- kuaternion / rotasyon yardimcilari (x, y, z, w sirasi) ----------
def quat_to_rot(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R):
    """3x3 rotasyon matrisi -> (x, y, z, w)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


if __name__ == "__main__":
    np.set_printoptions(precision=4, suppress=True)
    print("Sifir konumda 0T6:\n", fk(np.zeros(6)), "\n")
    print(f"{'#':4} {'q [rad]':52} {'x':>7} {'y':>7} {'z':>7}")
    for name, deg in CONFIGS.items():
        q = np.radians(deg)
        p = fk(q)[:3, 3]
        print(f"{name:4} {np.array2string(q, precision=4):52} "
              f"{p[0]:7.3f} {p[1]:7.3f} {p[2]:7.3f}")
