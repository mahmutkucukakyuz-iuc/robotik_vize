#!/usr/bin/env python3
"""Bilek kamerasi ile kup algilama (ROS'tan bagimsiz yardimci fonksiyonlar).

1) Renk esikleme (HSV) ile kirmizi/yesil/mavi bloblar bulunur, her blobun en kucuk
   alanli dikdortgeni (minAreaRect) alinir.
2) Piksel -> isin -> duzlem kesisimi: kamera pozu FK'den (0T6 = tool0) hesaplanir; blob
   merkezinden cikan isin kup MERKEZI duzlemiyle (z = zemin + kup/2) kesistirilir. Egik
   bakista blob ust yuz + yan yuzlerden olusur; bu siluetin merkezi kup merkezinin
   izdusumune yakindir (ust yuz duzlemi kullanilsaydi kup robota dogru kayik bulunurdu).
3) Dikdortgenin koseleri ust yuz duzlemine izdusurulerek kupun base_link'teki yaw acisi bulunur.

Kamera (Gazebo) kurali: camera_link +x bakis yonu, goruntude saga dogru = -y, asagi dogru = -z.
Kamera gripper'in -x plakasinda, tool0'in yaklasma (+z) yonune bakar; goruntunun yatay
ekseni parmak acilma yonu (link6 y).
"""
import math

import cv2
import numpy as np

from rk6_control.kinematics import fk

# camera_link'in link6'ya gore yeri (rk6.urdf.xacro camera_joint: xyz -0.0256 0 0.075, rpy 0 -pi/2 0)
CAM_XYZ_IN_LINK6 = (-0.0256, 0.0, 0.075)
TOOL_Z_IN_LINK6 = 0.15013   # tool0 = link6 * Tz(0.15013) (rk6.urdf.xacro tool0_joint, tool_z)

# HSV araliklari (OpenCV: H 0-179). Kirmizi iki uctadir.
COLOR_RANGES = {
    'red': [((0, 120, 60), (8, 255, 255)), ((170, 120, 60), (179, 255, 255))],
    'green': [((40, 100, 50), (85, 255, 255))],
    'blue': [((100, 120, 50), (130, 255, 255))],
}


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def camera_pose(q):
    """base_link -> camera_link (4x4), eklem acilarindan."""
    T_link6 = fk(q).copy()
    T_link6[:3, 3] -= T_link6[:3, 2] * TOOL_Z_IN_LINK6  # tool0 * Tz(-tool_z)
    T_cam_in6 = np.eye(4)
    T_cam_in6[:3, :3] = _ry(-math.pi / 2)
    T_cam_in6[:3, 3] = CAM_XYZ_IN_LINK6
    return T_link6 @ T_cam_in6


def intrinsics(width, height, hfov=1.047, K=None):
    """(fx, fy, cx, cy). CameraInfo K varsa onu, yoksa yatay gorus acisini kullanir."""
    if K is not None and K[0] > 0:
        return K[0], K[4], K[2], K[5]
    fx = (width / 2.0) / math.tan(hfov / 2.0)
    return fx, fx, width / 2.0, height / 2.0


def pixel_to_plane(u, v, T_cam, intr, z_plane):
    """Pikselden cikan isinin z = z_plane duzlemiyle kesisimi (base_link'te [x, y, z])."""
    fx, fy, cx, cy = intr
    d_cam = np.array([1.0, -(u - cx) / fx, -(v - cy) / fy])
    d = T_cam[:3, :3] @ d_cam
    o = T_cam[:3, 3]
    if abs(d[2]) < 1e-6:
        return None
    s = (z_plane - o[2]) / d[2]
    if s <= 0:
        return None
    return o + s * d


def detect_blobs(bgr, min_area=150, border=4):
    """Renkli bloblari bulur. Goruntu kenarina degen (kesik) bloblar atlanir.
    Donus: [{'color', 'center': (u, v), 'box': 4x2 kose pikselleri, 'area'}]"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, w = hsv.shape[:2]
    kernel = np.ones((3, 3), np.uint8)
    out = []
    for color, ranges in COLOR_RANGES.items():
        mask = np.zeros((h, w), np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, np.array(lo), np.array(hi))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area:
                continue
            x, y, bw, bh = cv2.boundingRect(c)
            if x <= border or y <= border or x + bw >= w - border or y + bh >= h - border:
                continue
            rect = cv2.minAreaRect(c)
            out.append({'color': color, 'center': rect[0],
                        'box': cv2.boxPoints(rect), 'area': area})
    return out


# Kapali parmaklarin tarama goruntusundeki golgesi (640x480 icin piksel; goruntunun alt ortasi)
FINGER_SHADOW = np.array([(60, 480), (165, 435), (265, 340), (375, 340), (475, 435), (580, 480)],
                         np.float32)


def touches_fingers(blob, width=640, height=480, margin=6.0):
    """Blob kapali parmaklarin golgesine degiyor mu (kup kismen ortulu -> merkez hatali olur).
    Yalnizca tarama pozlari icin (parmaklar kapali, kamera disari egik)."""
    poly = FINGER_SHADOW * np.float32([width / 640.0, height / 480.0])
    box = np.asarray(blob['box'], np.float32)
    pts = list(box) + [(box[k] + box[(k + 1) % 4]) / 2 for k in range(4)]
    return any(cv2.pointPolygonTest(poly, (float(u), float(v)), True) > -margin for u, v in pts)


def blob_to_cube(blob, T_cam, intr, z_top, z_center=None):
    """Blobu kup duzlemlerine izdusurur: (x, y, yaw_rad) base_link'te.
    Merkez z_center duzleminde (verilmezse z_top), koseler z_top duzleminde kesistirilir.
    yaw, kupun bir kenarinin acisidir (90 derece belirsizligiyle, [-45, 45) araliginda)."""
    p = pixel_to_plane(*blob['center'], T_cam, intr, z_top if z_center is None else z_center)
    corners = [pixel_to_plane(u, v, T_cam, intr, z_top) for u, v in blob['box']]
    if p is None or any(c is None for c in corners):
        return None
    e = corners[1] - corners[0]
    yaw = math.atan2(e[1], e[0])
    yaw = (yaw + math.pi / 4) % (math.pi / 2) - math.pi / 4
    return float(p[0]), float(p[1]), yaw


def merge_detections(dets, radius=0.04):
    """Ayni kupun farkli goruntulerdeki tespitlerini birlestirir.
    dets: [{'color', 'x', 'y', 'yaw', 'pix_dist'}]. Ayni renkte ve radius icindeki tespitler
    tek kupe sayilir; konum, goruntu merkezine en yakin tespitten alinir (en az perspektif hatasi)."""
    cubes = []
    for d in sorted(dets, key=lambda d: d['pix_dist']):
        for c in cubes:
            if c['color'] == d['color'] and math.hypot(c['x'] - d['x'], c['y'] - d['y']) < radius:
                c['n'] += 1
                break
        else:
            cubes.append(dict(d, n=1))
    return cubes


def annotate(bgr, blobs, labels):
    """Hata ayiklama/rapor icin tespitleri goruntu uzerine cizer."""
    img = bgr.copy()
    for b, text in zip(blobs, labels):
        cv2.drawContours(img, [np.int32(b['box'])], 0, (0, 255, 255), 2)
        u, v = map(int, b['center'])
        cv2.circle(img, (u, v), 3, (0, 255, 255), -1)
        cv2.putText(img, text, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
        cv2.putText(img, text, (u + 6, v - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    return img
