#!/usr/bin/env python3
"""scan_and_stack: bilek kamerasiyla etrafi tarar, bulunan butun kupleri robotun onune ust uste dizer.

1. Tarama: iki halka. Her halkada tcp 'scan_radius' yaricapli yayda, 'scan_z' yuksekliginde
   'scan_yaws_deg' acilarina gider; gripper dikeyden 'scan_tilt_deg' kadar disari egiktir.
   1. halka (r 0.14, z 0.125, 10 derece) yakin bandi (r ~ 0.16..0.25 m), 2. halka (r 0.14,
   z 0.18, 30 derece) uzak bandi (r ~ 0.22..0.43 m) gorur. Goruntunun alt ~1/3'unu kapali
   parmaklar kapatir; kup ancak tamami gorunur ve parmak golgesi disindaysa sayilir.
   Yaw: ortada 20 derece aralik, kenarlarda +-115 / +-125 (J1 siniri 135). 15 x 2 = 30 durak,
   calisma alaninin (r 0.18..0.40 m, phi -133..+133 derece) her noktasi en az 2 goruntude
   (ortalama 2.8; tek halkali 9 duraklik taramada r 0.29..0.37 disi hic, ici cogunlukla 1 goruntu).
   Tum duraklar IK ile erisilebilir, sinira marj >= 9 derece (bkz. python3 -m rk6_manipulation.ik).
   Her durakta kamera goruntusunde renkli bloblar bulunur (perception.detect_blobs),
   kamera pozu FK'den hesaplanir ve blob merkezi kup merkezi duzlemine, koseleri ust yuz
   duzlemine izdusurulerek base_link'teki (x, y, yaw) bulunur.
2. Birlestirme: ayni kupun farkli goruntulerdeki tespitleri tek kupe indirgenir.
3. Dizme: kupler sirayla alinir ve 'stack_xy' noktasina, her biri bir oncekinin ustune konur.
   Kupun ustune gelince (yaklasma pozu, kamera dik asagi) kup yeniden goruntulenip konum ve
   yaw duzeltilir (refine). Kup parmaklar arasinda tam ortalanmadiysa (parmak konumlarindan
   olculur) birakma noktasi bu kaymaya gore duzeltilir.

Taranan goruntuler isaretlenmis olarak 'save_dir' klasorune kaydedilir (rapor icin) ve
/scan_and_stack/detections topic'inde yayinlanir (rqt_image_view ile canli izlenebilir).

Parametreler (ortak olanlar icin bkz. manipulator.py):
  scan_radius ([0.14, 0.14] m), scan_z ([0.125, 0.18] m, tcp yuksekligi, zemin z = 0),
  scan_tilt_deg ([10, 30]): halka basina bir deger (tek sayi verilirse tek halka)
  scan_yaws_deg ([-125, -115, -100, -80 .. 80, 100, 115, 125]) her halkada ayni acilar
  scan_move_s (1.2 s)    ayni halkadaki komsu duraklar arasi hareket suresi (ilk durak ve halka
                         gecisi segment_s); Gazebo'da tarama ~77 s
  stack_xy ([0.25, 0.0]) robotun onunde; 3 kat icin birakma/ustu pozlari sinira >= 8 derece marjla
  stack_yaw_deg (0), cube_size (0.03 m)
  place_up (0.045 m)     kule ustunde birakma yaklasma yuksekligi (3. katta tcp z = 0.126; tasinan
                         kupun alti kulenin ~3 cm ustunde)
  release_open (0.021 m) birakirken parmak acikligi (kuleyi itmemek icin az acilir; temas 0.015)
  refine (true)          kupun ustunde yeniden bakip konumu duzelt
  colors (['red','green','blue']) dizme sirasi (bulunanlar bu sirayla dizilir)
  save_dir (~/rk6_results/scan)
"""
import math
import os
import time

import cv2
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rk6_manipulation import perception
from rk6_manipulation.ik import fk_tcp
from rk6_manipulation.manipulator import Manipulator, run_node

# Varsayilan tarama: 2 halka x 15 yaw = 30 durak (kapsama analizi: bkz. modul aciklamasi, madde 1)
SCAN_RADIUS = (0.14, 0.14)      # tcp yay yaricapi [m], halka basina
SCAN_Z = (0.125, 0.18)          # tcp yuksekligi [m] (zemin z = 0)
SCAN_TILT_DEG = (10.0, 30.0)    # gripper'in dikeyden disari egimi [derece]
# Ortada 20 derece aralik; kenarlarda +-115 ve +-125 (J1 siniri 135, kenardaki kup de 2 goruntude)
SCAN_YAWS_DEG = (-125.0, -115.0, -100.0, -80.0, -60.0, -40.0, -20.0, 0.0,
                 20.0, 40.0, 60.0, 80.0, 100.0, 115.0, 125.0)


def _as_list(v):
    """Parametre tek sayi ya da liste olabilir -> float listesi."""
    try:
        return [float(x) for x in v]      # list / tuple / array.array
    except TypeError:
        return [float(v)]


class ScanAndStack(Manipulator):
    def __init__(self):
        super().__init__('scan_and_stack')
        p = self.declare_parameter
        # Halka parametreleri: her biri tek sayi ya da halka basina bir deger (liste)
        dyn = ParameterDescriptor(dynamic_typing=True)
        radii = _as_list(p('scan_radius', list(SCAN_RADIUS), dyn).value)
        zs = _as_list(p('scan_z', list(SCAN_Z), dyn).value)
        tilts = _as_list(p('scan_tilt_deg', list(SCAN_TILT_DEG), dyn).value)
        n = max(len(radii), len(zs), len(tilts))
        pick = (lambda v, i: v[min(i, len(v) - 1)])   # kisa liste: son deger tekrarlanir
        self.scan_rings = [(pick(radii, i), pick(zs, i), math.radians(pick(tilts, i)))
                           for i in range(n)]
        self.scan_yaws = [math.radians(a) for a in
                          p('scan_yaws_deg', list(SCAN_YAWS_DEG)).value]
        self.scan_move_s = float(p('scan_move_s', 1.2).value)
        self.stack_xy = list(p('stack_xy', [0.25, 0.0]).value)
        self.stack_yaw = math.radians(float(p('stack_yaw_deg', 0.0).value))
        self.cube = float(p('cube_size', 0.03).value)
        self.place_up = float(p('place_up', 0.045).value)
        self.release_open = float(p('release_open', 0.021).value)
        self.refine = bool(p('refine', True).value)
        self.colors = list(p('colors', ['red', 'green', 'blue']).value)
        self.save_dir = os.path.expanduser(p('save_dir', '~/rk6_results/scan').value)

        self.cam_K = None
        self.create_subscription(CameraInfo, '/wrist_camera/camera_info', self.on_info,
                                 qos_profile_sensor_data)
        # Her tarama duragindaki isaretli goruntu (rqt_image_view ile izlenebilir)
        self.debug_pub = self.create_publisher(Image, '/scan_and_stack/detections', 1)

    def on_info(self, msg):
        self.cam_K = list(msg.k)

    # ---------------- 1-2. tarama ----------------
    def scan(self):
        log = self.get_logger()
        os.makedirs(self.save_dir, exist_ok=True)
        z_top = self.ground_z + self.cube
        z_mid = self.ground_z + self.cube / 2
        dets = []
        # Duraklar: 1. halka yaw listesinde ileri, 2. halka geri (yilan gibi; halka gecisi kisa)
        stops = []
        for k, (r, z, tilt) in enumerate(self.scan_rings):
            yaws = self.scan_yaws if k % 2 == 0 else self.scan_yaws[::-1]
            stops += [(r, z, tilt, a) for a in yaws]
        log.info(f'Tarama: {len(stops)} durak, ' + ', '.join(
            f'halka {k + 1}: r {r} m z {z} m egim {math.degrees(t):.0f} der'
            for k, (r, z, t) in enumerate(self.scan_rings)))
        prev = None
        for i, (r, z, tilt, a) in enumerate(stops):
            x, y = r * math.cos(a), r * math.sin(a)
            # komsu duraklar arasi kucuk hareket kisa surer; ilk durak ve halka gecisi segment_s
            dur = self.scan_move_s if prev is not None and prev[:3] == (r, z, tilt) else self.segment_s
            prev = (r, z, tilt)
            try:
                self.goto(x, y, z, a, f'tarama {i + 1}/{len(stops)} (r {r:.2f}, {math.degrees(a):+.1f} derece)',
                          duration=dur, tilt=tilt)
            except RuntimeError as ex:
                log.warning(f'  tarama duragi atlandi: {ex}')
                prev = None    # sonraki hareket uzun olabilir
                continue
            time.sleep(0.25)
            img = self.fresh_image(n_new=3)
            if img is None:
                log.warning('  kamera goruntusu gelmedi')
                continue
            with self.lock:
                q_meas = self.q_arm.copy()
            T_cam = perception.camera_pose(q_meas)
            h, w = img.shape[:2]
            intr = perception.intrinsics(w, h, K=self.cam_K)
            blobs = perception.detect_blobs(img)
            labels = []
            for b in blobs:
                if perception.touches_fingers(b, w, h):
                    labels.append('parmak arkasi')   # kismen ortulu: baska goruntude tam gorunur
                    continue
                cube = perception.blob_to_cube(b, T_cam, intr, z_top, z_mid)
                if cube is None:
                    labels.append('?')
                    continue
                cx, cy, cyaw = cube
                pix = math.hypot(b['center'][0] - w / 2, b['center'][1] - h / 2)
                dets.append({'color': b['color'], 'x': cx, 'y': cy, 'yaw': cyaw, 'pix_dist': pix})
                labels.append(f"{b['color']} ({cx:.3f},{cy:.3f})")
                log.info(f"    {b['color']}: ({cx:.3f}, {cy:.3f}) yaw {math.degrees(cyaw):+.1f} derece")
            marked = perception.annotate(img, blobs, labels)
            cv2.imwrite(os.path.join(self.save_dir, f'scan_{i + 1:02d}.png'), marked)
            self.debug_pub.publish(self.bridge.cv2_to_imgmsg(marked, 'bgr8'))
        cubes = perception.merge_detections(dets)
        log.info(f'Bulunan kup sayisi: {len(cubes)}')
        for c in cubes:
            log.info(f"  {c['color']:5s}: ({c['x']:.3f}, {c['y']:.3f}) yaw "
                     f"{math.degrees(c['yaw']):+.1f} derece ({c['n']} goruntude)")
        return cubes

    # ---------------- 3. dizme ----------------
    def stack(self, cubes):
        log = self.get_logger()
        sx, sy = self.stack_xy
        # Zaten dizme noktasinda duran kup varsa kulenin ilk kati sayilir
        base = [c for c in cubes if math.hypot(c['x'] - sx, c['y'] - sy) < 0.03]
        todo = [c for c in cubes if c not in base]
        order = {c: i for i, c in enumerate(self.colors)}
        todo.sort(key=lambda c: order.get(c['color'], len(order)))
        level = len(base)
        for c in todo:
            self.set_status(f"[Kat {level + 1}] {c['color']} kup ({c['x']:.3f}, {c['y']:.3f}) -> kule")
            try:
                self.pick_one(c)
                self.to_transport(c, level)
                self.place_on_stack(level)
                level += 1
            except RuntimeError as ex:
                log.warning(f'  bu kup atlandi: {ex}')
                self.release_safely()
        return level

    def pick_one(self, c):
        # Kupu alirken yaklasma yuksekligi kule yuksekligine bagli degil; uzak kuplerde
        # erisilebilen en yuksek deger secilir (approach_height)
        z_grasp = self.ground_z + self.grasp_h
        z_up, q_up = self.approach_height(c['x'], c['y'], c['yaw'])
        self.gripper(self.g_open, 'ac')
        self.goto(c['x'], c['y'], z_up, c['yaw'], 'kupun ustu', n_sym=4, q=q_up)
        if self.refine and self.refine_cube(c):
            # duzeltilmis konum/yaw ile yaklasma pozunu yeniden coz (kucuk hareket)
            z_up, q_up = self.approach_height(c['x'], c['y'], c['yaw'])
            self.goto(c['x'], c['y'], z_up, c['yaw'], 'kupun ustu (duzeltildi)', n_sym=4, q=q_up,
                      duration=1.5)
        self.linear([c['x'], c['y'], z_grasp], 'inis')
        self.gripper(self.g_close, 'kapat')
        self.linear([c['x'], c['y'], z_up], 'kaldir')
        if not self.check_held():
            raise RuntimeError('kup kavranamadi')

    def refine_cube(self, c, max_shift=0.03):
        """Yaklasma pozunda (kamera dik asagi, kup goruntunun ortasina yakin) kupu yeniden bul;
        konum ve yaw'i guncelle. Duzeltme yapildiysa True."""
        log = self.get_logger()
        time.sleep(0.3)
        img = self.fresh_image()
        if img is None:
            return False
        with self.lock:
            q_meas = self.q_arm.copy()
        T_cam = perception.camera_pose(q_meas)
        h, w = img.shape[:2]
        intr = perception.intrinsics(w, h, K=self.cam_K)
        best = None
        for b in perception.detect_blobs(img):
            if b['color'] != c['color']:
                continue
            cube = perception.blob_to_cube(b, T_cam, intr, self.ground_z + self.cube,
                                           self.ground_z + self.cube / 2)
            if cube is None:
                continue
            d = math.hypot(cube[0] - c['x'], cube[1] - c['y'])
            if d < max_shift and (best is None or d < best[0]):
                best = (d, cube)
        if best is None:
            log.info('  yeniden bakis: kup goruntude bulunamadi, tarama konumu kullaniliyor')
            return False
        d, (x, y, yaw) = best
        log.info(f"  yeniden bakis: ({c['x']:.3f}, {c['y']:.3f}) -> ({x:.3f}, {y:.3f}), "
                 f"kayma {d * 1000:.1f} mm, yaw {math.degrees(c['yaw']):+.1f} -> {math.degrees(yaw):+.1f}")
        moved = d > 0.002 or abs((yaw - c['yaw'] + math.pi / 4) % (math.pi / 2) - math.pi / 4) > math.radians(3)
        c['x'], c['y'], c['yaw'] = x, y, yaw
        return moved

    def to_transport(self, c, level):
        """Kupu tasima yuksekligine cikar: uzak kup alcaktan alindiysa kuleye giderken carpmasin.
        Tabana dogru en fazla 0.28 m yaricapa cekilir (orada daha yuksege erisilebilir) ve
        tutulan kupun alti (tcp - 0.017) zemindeki kuplerin ve kulenin (level kat) en az 3 cm
        ustunden gececek kadar yukseltilir: kat 0-1'de tcp z = 0.097, kat 2'de 0.107
        (dik asagi bakan gripper r = 0.28'de en fazla z ~ 0.13'e cikabilir)."""
        z_t = self.ground_z + self.grasp_h + max(self.approach_h, level * self.cube + 0.03)
        r = math.hypot(c['x'], c['y'])
        k = min(1.0, 0.28 / r) if r > 1e-6 else 1.0
        tx, ty = c['x'] * k, c['y'] * k
        try:
            self.linear([tx, ty, z_t], 'tasima yuksekligi')
        except RuntimeError as ex:
            # Duz cizgi yolu erisim sinirina takildi: eklem uzayinda tam IK cozumuyle git
            self.get_logger().info(f'  {ex}; eklem uzayinda gidiliyor')
            self.goto(tx, ty, z_t, math.atan2(ty, tx), 'tasima yuksekligi', n_sym=4)

    def place_on_stack(self, level):
        # Kulenin ustune yaklasma, o anki kule yuksekligine gore (kule robota yakin, erisilebilir)
        sx, sy = self.stack_xy
        z_place = self.ground_z + self.grasp_h + level * self.cube + 0.004
        z_up = z_place + self.place_up
        # Kup parmaklar arasinda kaymissa tcp hedefini kaydir (kup merkezi kuleye otursun)
        off = self.held_offset()
        q, _ = self.solve(sx, sy, z_up, self.stack_yaw, n_sym=4)
        y_axis = fk_tcp(q)[:3, 1]
        tx, ty = sx - y_axis[0] * off, sy - y_axis[1] * off
        self.get_logger().info(f'  kup kaymasi {off * 1000:+.1f} mm -> birakma hedefi ({tx:.3f}, {ty:.3f})')
        self.goto(tx, ty, z_up, self.stack_yaw, 'kule ustu', n_sym=4)
        self.linear([tx, ty, z_place], 'birakma inisi', duration=self.linear_s * 1.5)
        self.gripper(self.release_open, 'birak', duration=1.0)
        self.linear([tx, ty, z_up], 'geri cekil')

    def run(self):
        log = self.get_logger()
        if not self.wait_ready():
            return False
        try:
            self.gripper(0.0, 'kapat (tarama)')
            cubes = self.scan()
            if not cubes:
                log.warning('Hic kup bulunamadi.')
                self.go_home()
                return False
            n = self.stack(cubes)
            self.go_home()
            self.gripper(0.0, 'kapat (home)')
            log.info(f'Tamamlandi: kule {n} kat. Tarama goruntuleri: {self.save_dir}')
            return True
        except RuntimeError as ex:
            log.error(str(ex))
            return False


def main(args=None):
    rclpy.init(args=args)
    run_node(ScanAndStack())


if __name__ == '__main__':
    main()
