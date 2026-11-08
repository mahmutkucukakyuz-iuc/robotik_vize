#!/usr/bin/env python3
"""rk6 icin sayisal ters kinematik (ROS'tan bagimsiz).

Bilek kacikli (d5 = -0.066 m, kuresel degil) oldugu icin kapali formda analitik cozum
pratik degil. Bu yuzden Jacobian tabanli sonumlu en kucuk kareler (Damped Least
Squares, Levenberg-Marquardt) kullanilir:

    e  = [p_d - p ;  aci * eksen (Rd R^T)]                  (6x1 konum + yonelim hatasi)
    dq = J^T (J J^T + lambda^2 I)^-1 e
    q <- q + dq   (eklem sinirlarina kirpilir), |e| < tol olana kadar

Geometrik Jacobian DH cercevelerinden kurulur (her eklem z_(i-1) ekseni etrafinda doner):
    J_i = [ z_(i-1) x (o_n - o_(i-1)) ;  z_(i-1) ]

Hedef cerceve: 0T_tcp = 0T6 * Tz(TCP_OFFSET). tool0 zaten parmak pad'lerinin ortasi
(kavrama noktasi), bu yuzden TCP_OFFSET = 0 (gripper_tcp == tool0). Parametre korunuyor.
Birimler: metre, radyan.

Erisim notu (asagi bakan gripper, numerik harita, bkz. __main__ / python3 -m rk6_manipulation.ik):
  Robot dogrudan zeminde: zemin = base_link z = 0. joint5 sinirinin (+-90 derece) yuzunden
  gripper dik asagi bakarken tool0 en fazla z ~ 0.14 m (r = 0.175..0.30 m), r = 0.35'te
  ~0.10 m, r = 0.375'te ~0.08 m'ye cikabilir. Zemindeki 3 cm kup icin kavrama (tcp z = 0.017)
  r = 0.175..0.425 m'de mumkun; sinira 10 dereceden fazla marj r = 0.20..0.40 m'de.
  Asagi bakan cozumler 'dirsek one bukulu' ailede: q2 ~ -20..50, q3 ~ 55..125, q4 ~ 0,
  q5 ~ 50..88, q6 ~ 0 derece (q2 + q3 + q5 ~ 180). Tarama pozlari icin gripper disari
  egilebilir (down_pose tilt): tarama halkalari r = 0.14, z = 0.125 / 10 derece ve z = 0.18 /
  30 derece egik pozlar J1 = +-125'e kadar >= 9 derece marjla erisilir.
"""
import numpy as np

from rk6_control.kinematics import LIMITS, fk

# gripper_tcp = tool0 (rk6_gripper.xacro: gripper_tcp_joint xyz 0 0 0)
TCP_OFFSET = 0.0

# URDF eklem sinirlari [rad] (kinematics.LIMITS: J1 +-135, J2 +-90, J3 +-135, J4..J6 +-90 derece)
JOINT_LIMITS = np.array(LIMITS, dtype=float)


# Cozumler eklem sinirlarindan bu kadar uzak tutulur. Sinira yapisik bir cozumde fizik motoru
# konumu sinirin milimetrik otesinde raporlayabilir; ros2_control limit korumasi da bu durumda
# sonraki komutlari engeller.
LIMIT_MARGIN = 0.03
SAFE_LIMITS = JOINT_LIMITS + np.array([LIMIT_MARGIN, -LIMIT_MARGIN])


def tz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def fk_tcp(q, tcp=TCP_OFFSET):
    """0T_tcp = 0T6 * Tz(tcp)."""
    return fk(q) @ tz(tcp)


def jacobian(q, tcp=TCP_OFFSET):
    """6x6 geometrik Jacobian (ust 3 satir dogrusal, alt 3 satir acisal hiz)."""
    frames = [fk(q, upto=i) for i in range(7)]          # 0T0 ... 0T6
    o_n = (frames[6] @ tz(tcp))[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        z = frames[i][:3, 2]
        o = frames[i][:3, 3]
        J[:3, i] = np.cross(z, o_n - o)
        J[3:, i] = z
    return J


def pose_error(T, T_d):
    """Konum ve yonelim hatasi (6x1). Yonelim hatasi = eksen * aci (Re = Rd R^T'nin logaritmasi).
    Not: 1/2 (n x n_d + s x s_d + a x a_d) = sin(aci) * eksen formulu buyuk acilarda yaniltir
    (or. 180 derece yaw farki + egik z ekseni icin sifir verir); bu yuzden tam log kullanilir."""
    e = np.zeros(6)
    e[:3] = T_d[:3, 3] - T[:3, 3]
    Re = T_d[:3, :3] @ T[:3, :3].T
    v = 0.5 * np.array([Re[2, 1] - Re[1, 2], Re[0, 2] - Re[2, 0], Re[1, 0] - Re[0, 1]])
    c = np.clip((np.trace(Re) - 1.0) / 2.0, -1.0, 1.0)
    th = np.arccos(c)
    if th < 1e-6:
        e[3:] = v
    elif th > np.pi - 1e-3:
        # ~180 derece: eksen kosegenden (isaret en buyuk bilesene gore)
        d = np.sqrt(np.maximum((np.diag(Re) + 1.0) / 2.0, 0.0))
        i = int(np.argmax(d))
        axis = (Re[:, i] + np.eye(3)[:, i]) / (2.0 * d[i])
        e[3:] = th * axis / np.linalg.norm(axis)
    else:
        e[3:] = th / np.sin(th) * v
    return e


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def down_pose(x, y, z, yaw, tilt=0.0, tilt_dir=0.0):
    """Yukaridan kavrama pozu: tcp z ekseni asagi (0,0,-1), x ekseni yatayda yaw acisinda.
    Parmaklar tcp'nin y ekseni boyunca kapanir: y = (sin yaw, -cos yaw, 0).
    tilt > 0: z ekseni dikeyden tilt kadar, yatayda tilt_dir yonune (disari) egilir
    (tarama pozlarinda kamera daha genis alan gorsun diye). tilt = 0 ise tam dik asagi."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, s, 0.0], [s, -c, 0.0], [0.0, 0.0, -1.0]])
    if tilt:
        R = _rz(tilt_dir) @ _ry(-tilt) @ _rz(-tilt_dir) @ R
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def _solve(T_d, q0, tcp, max_iter, lam, pos_tol, rot_tol):
    q = np.clip(np.array(q0, dtype=float), SAFE_LIMITS[:, 0], SAFE_LIMITS[:, 1])
    for it in range(max_iter):
        e = pose_error(fk_tcp(q, tcp), T_d)
        if np.linalg.norm(e[:3]) < pos_tol and np.linalg.norm(e[3:]) < rot_tol:
            return q, e, it, True
        J = jacobian(q, tcp)
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        step = np.max(np.abs(dq))
        if step > 0.3:                       # buyuk sicramalari sinirla
            dq *= 0.3 / step
        q = np.clip(q + dq, SAFE_LIMITS[:, 0], SAFE_LIMITS[:, 1])
    return q, pose_error(fk_tcp(q, tcp), T_d), max_iter, False


# Ek baslangic noktalari (q1 hedefe cevrilir). Sifir poz dimdik yukari oldugu icin
# asagi bakan cozumler "dirsek one bukulu" ailesindedir: q2 + q3 + q5 ~ 180 derece, q4 = q6 = 0.
# Erisim haritasindan (__main__) secildi (zemin z = 0): yakin/orta/uzak kavrama, yaklasma ve
# tarama bolgeleri. Yorumdaki [q2, q3, q5] derece.
SEEDS = [
    [0.0, 0.28, 1.92, 0.0, 0.95, 0.0],    # yakin kavrama (r ~ 0.25 m, z ~ 0.02)  [16, 110, 54]
    [0.0, 0.66, 1.34, 0.0, 1.14, 0.0],    # orta kavrama (r ~ 0.35, z ~ 0.02)     [38, 77, 65]
    [0.0, 0.88, 0.95, 0.0, 1.31, 0.0],    # uzak kavrama (r ~ 0.40, z ~ 0.02)     [51, 54, 75]
    [0.0, 0.33, 1.45, 0.0, 1.36, 0.0],    # yaklasma/tasima (r ~ 0.30, z ~ 0.10)  [19, 83, 78]
    [0.0, -0.35, 2.00, 0.0, 1.13, 0.0],   # tarama (r ~ 0.22, z ~ 0.14, 20 egik)  [-20, 115, 65]
]


def elbow_forward(q):
    """'Dirsek one bukulu' aile (q3 > 0). Asagi bakan pozlarda kolun arkadan dolanan
    (q3 < 0, taban ters yone donuk) cozumu de vardir; hareket surekli olsun diye o elenir."""
    return q[2] > 0.0


def ik(T_d, q0=None, tcp=TCP_OFFSET, max_iter=400, lam=0.05,
       pos_tol=1e-4, rot_tol=1e-3, use_seeds=True, prefer=elbow_forward):
    """Hedef poz 0T_tcp (4x4) icin eklem acilari.

    q0'dan (genelde mevcut durus) ve SEEDS'ten ayri ayri baslar; yakinsayan cozumler
    arasindan q0'a en yakin olani secilir (hareket kisa ve surekli olur).
    prefer(q) True olan cozum varsa yalnizca onlar arasindan secilir (None: hepsi).
    Donus: (q, basarili_mi, konum_hatasi_m, yonelim_hatasi_rad)
    """
    ref = np.zeros(6) if q0 is None else np.array(q0, dtype=float)
    starts = [ref.copy()]
    # use_seeds=False: yalnizca q0'dan baslar (yakin ara noktalar icin hizli ve surekli)
    for seed in (SEEDS if use_seeds else []):
        s = np.array(seed, dtype=float)
        s[0] = np.arctan2(T_d[1, 3], T_d[0, 3])   # tabani hedefe cevir
        starts.append(s)
    best = None
    for s in starts:
        q, e, _, ok = _solve(T_d, s, tcp, max_iter, lam, pos_tol, rot_tol)
        if not ok:
            continue
        dist = np.linalg.norm(q - ref)
        if prefer is not None and not prefer(q):
            dist += 100.0                     # tercih edilmeyen aile: ancak baska cozum yoksa
        if best is None or dist < best[1]:
            best = (q, dist, e)
    if best is None:
        q, e, _, _ = _solve(T_d, starts[0], tcp, max_iter, lam, pos_tol, rot_tol)
        return q, False, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))
    q, _, e = best
    return q, True, float(np.linalg.norm(e[:3])), float(np.linalg.norm(e[3:]))


def ik_down(x, y, z, yaw, q0=None, n_sym=2, tilt=0.0, tilt_dir=None, **kw):
    """Yukaridan kavrama icin IK. Iki parmakli gripper 180 derece simetrik oldugu icin
    yaw ve yaw+pi ikisi de ayni kavramayi verir; q0'a en yakin cozum secilir.
    (Pratikte yaw+pi, joint4 = 0 ve joint6 = 0 olan 'temiz' bilek durusunu verir.)
    n_sym=4: kup gibi 90 derece simetrik nesnelerde yaw + k*90 derecenin hepsi denenir.
    tilt/tilt_dir: bkz. down_pose (tilt_dir verilmezse hedefin yonu, yani disari)."""
    ref = np.zeros(6) if q0 is None else np.array(q0, dtype=float)
    if tilt_dir is None:
        tilt_dir = float(np.arctan2(y, x))
    prefer = kw.get('prefer', elbow_forward)

    def cost(q):
        return np.linalg.norm(q - ref) + (100.0 if prefer is not None and not prefer(q) else 0.0)

    best = None
    for yw in (yaw + k * 2 * np.pi / n_sym for k in range(n_sym)):
        q, ok, ep, er = ik(down_pose(x, y, z, yw, tilt, tilt_dir), q0=ref, **kw)
        if ok and (best is None or cost(q) < cost(best[0])):
            best = (q, ok, ep, er)
    if best is None:
        return ik(down_pose(x, y, z, yaw, tilt, tilt_dir), q0=ref, **kw)
    return best


def tcp_yaw(q):
    """tcp x ekseninin yatay duzlemdeki acisi (ik_down'un sectigi yaw'i bulmak icin)."""
    x = fk_tcp(q)[:3, 0]
    return float(np.arctan2(x[1], x[0]))


def link_heights(q):
    """Her DH cercevesi orijininin ve tcp'nin z yuksekligi (zemine carpma kontrolu icin)."""
    zs = [fk(q, upto=i)[2, 3] for i in range(1, 7)]
    zs.append(fk_tcp(q)[2, 3])
    return zs


def limit_margin(q):
    """En yakin eklem sinirina uzaklik [rad]."""
    q = np.asarray(q, dtype=float)
    return float(np.min(np.minimum(q - JOINT_LIMITS[:, 0], JOINT_LIMITS[:, 1] - q)))


if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)
    rng = np.random.default_rng(0)
    # 1) FK -> IK -> FK gidis-donus testi (rastgele erisilebilir pozlar)
    n_ok, worst = 0, 0.0
    for _ in range(50):
        q_true = rng.uniform(JOINT_LIMITS[:, 0] * 0.8, JOINT_LIMITS[:, 1] * 0.8)
        T_d = fk_tcp(q_true)
        q, ok, ep, er = ik(T_d, q0=np.zeros(6))
        n_ok += ok
        worst = max(worst, ep if ok else 0.0)
    print(f'rastgele 50 poz: {n_ok} yakinsadi, en buyuk konum hatasi {worst * 1000:.4f} mm')

    def show(label, x, y, z, yaw, q0, **kw):
        q, ok, ep, er = ik_down(x, y, z, yaw, q0=q0, **kw)
        print(f'  {label:22s} ({x:+.3f},{y:+.3f},{z:+.3f}) ok={ok} e_p={ep * 1000:.3f} mm '
              f'q={np.degrees(q).round(1)} marj={np.degrees(limit_margin(q)):.1f}')
        return q

    # 2) erisim haritasi: gripper dik asagi, temiz yaw (r, z) izgarasi; hucre = sinir marji [derece]
    print('\nerisim haritasi (asagi bakan gripper, hucre: en yakin sinira marj [derece], -- = yok)')
    radii = np.arange(0.15, 0.451, 0.05)
    print('   z \\ r ' + ' '.join(f'{r:5.2f}' for r in radii))
    for z in (0.16, 0.14, 0.12, 0.10, 0.08, 0.06, 0.047, 0.03, 0.017):
        row = []
        for r in radii:
            q, ok, _, _ = ik_down(r, 0.0, z, np.pi, q0=np.zeros(6))
            row.append(f'{np.degrees(limit_margin(q)):5.1f}' if ok else '   --')
        print(f'{z:+8.3f} ' + ' '.join(row))

    # 3) senaryo pozlari (zemin z = 0, kup 3 cm, grasp_h 0.017, approach_h 0.08) - varsayilanlarla ayni
    G, GH, AH, CUBE = 0.0, 0.017, 0.08, 0.03
    print('\nkavrama / yaklasma (rk6_pick_world.sdf kupleri, pick_place birakma noktalari):')
    for name, x, y, yaw in [('kirmizi', 0.17, 0.27, 0.35), ('mavi', 0.31, -0.15, -0.61),
                            ('yesil', -0.06, -0.29, 0.17), ('birakma 1', 0.28, 0.12, 0.0),
                            ('birakma 2', 0.26, -0.04, 0.0)]:
        for z, lbl in ((G + GH, 'kavrama'), (G + GH + AH, 'yaklasma')):
            show(f'{name} {lbl}', x, y, z, yaw, np.zeros(6), n_sym=4)

    # kule (0.25, 0): birakma, ustu (place_up 0.045), tasima (scan_and_stack.to_transport ile ayni)
    print('\nkule (0.25, 0), kat 0..2:')
    for lv in range(3):
        z_pl = G + GH + lv * CUBE + 0.004
        z_tr = G + GH + max(AH, lv * CUBE + 0.03)
        for name, z in (('birak', z_pl), ('ustu', z_pl + 0.045)):
            show(f'kat {lv} {name}', 0.25, 0.0, z, 0.0, np.zeros(6), n_sym=4)
        show(f'kat {lv} tasima', 0.28 * np.cos(0.6), 0.28 * np.sin(0.6), z_tr, 0.6, np.zeros(6), n_sym=4)

    # ik_goto rpy ornegi (gripper yatay, one bakar)
    T = np.eye(4)
    T[:3, :3] = _ry(np.pi / 2)
    T[:3, 3] = [0.35, 0.0, 0.25]
    q, ok, ep, er = ik(T, q0=np.zeros(6))
    print(f'\nrpy ornegi (0.35, 0, 0.25, pitch 90): ok={ok} e_p={ep * 1000:.3f} mm q={np.degrees(q).round(1)}')

    # 4) tarama pozlari (scan_and_stack varsayilani: 2 halka, 1. halka ileri 2. halka geri), sirayla
    yaws = [-125, -115, -100, -80, -60, -40, -20, 0, 20, 40, 60, 80, 100, 115, 125]
    q = np.zeros(6)
    for k, (r, z, tilt) in enumerate([(0.14, 0.125, 10), (0.14, 0.18, 30)]):
        print(f'\ntarama halka {k + 1} (r={r}, z={z}, tilt={tilt}):')
        for a in (yaws if k % 2 == 0 else yaws[::-1]):
            ar = np.radians(a)
            q_new, ok, ep, _ = ik_down(r * np.cos(ar), r * np.sin(ar), z, ar, q0=q,
                                       tilt=np.radians(tilt))
            print(f'  {a:+4d} ok={ok} e_p={ep * 1000:.3f} mm q={np.degrees(q_new).round(1)} '
                  f'marj={np.degrees(limit_margin(q_new)):.1f} max|dq|={np.degrees(np.max(np.abs(q_new - q))):.0f}')
            q = q_new

