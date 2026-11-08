#!/usr/bin/env python3
"""Sekil 3: rk6 DH cerceveleri (sifir poz, q = 0), yandan (x-z) ve onden (y-z) gorunus.

Cerceveler tablodan elle yazilmaz: DH parametrelerinden duz kinematikle hesaplanir,
sonra rapordaki 3.2 tablosuyla 1e-6 icinde karsilastirilir (assert).

Kullanim: python3 sekil3_dh_cerceveleri.py [cikti.png]
"""
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Circle, Rectangle  # noqa: E402

OUT = sys.argv[1] if len(sys.argv) > 1 else 'sonuclar/sekil3_dh_cerceveleri.png'

# ---------------------------------------------------------------- renkler (sekil4 ile ayni aile)
TEXT = '#0b0b0b'
TEXT2 = '#52514e'
GRID = '#e4e3df'
SURFACE = '#fcfcfb'
CX = '#d62728'      # x ekseni (kirmizi)
CY = '#2ca02c'      # y ekseni (yesil)
CZ = '#1f5fd6'      # z ekseni (mavi)
LINK = '#6e6d69'    # iskelet (gri)
THETA = '#8a4fbf'   # eklem acisi isaretleri (mor)
DIM = '#3a3936'     # olcu cizgileri
LINK_FILL = ['#f2b134', '#4fb3bf', '#e8743b', '#7cc36a', '#c879c8', '#5a8fe0']

# ---------------------------------------------------------------- DH ve FK
# (ofset, d, a, alpha) -- standart DH, T_i = Rz(q+ofset) Tz(d) Tx(a) Rx(alpha)
DH = [
    (0.0, 0.1313, 0.0, -np.pi / 2),
    (-np.pi / 2, 0.0, 0.22995, 0.0),
    (+np.pi / 2, 0.0652, 0.0, +np.pi / 2),
    (0.0, 0.22995, 0.0, -np.pi / 2),
    (0.0, -0.0660, 0.0, +np.pi / 2),
    (0.0, 0.20093, 0.0, 0.0),
]


def dh_matrix(theta, d, a, alpha):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0.0, sa, ca, d],
                     [0.0, 0.0, 0.0, 1.0]])


def fk_frames(q):
    T = np.eye(4)
    frames = [T.copy()]
    for qi, (off, d, a, al) in zip(q, DH):
        T = T @ dh_matrix(qi + off, d, a, al)
        frames.append(T.copy())
    return frames


F = fk_frames(np.zeros(6))
O = [T[:3, 3] for T in F]
X = [T[:3, 0] for T in F]
Y = [T[:3, 1] for T in F]
Z = [T[:3, 2] for T in F]

# Rapor 3.2 tablosu: orijin, x, y, z
TABLE = [
    ((0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
    ((0, 0, 0.1313), (1, 0, 0), (0, 0, -1), (0, 1, 0)),
    ((0, 0, 0.36125), (0, 0, 1), (1, 0, 0), (0, 1, 0)),
    ((0, 0.0652, 0.36125), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
    ((0, 0.0652, 0.5912), (1, 0, 0), (0, 0, -1), (0, 1, 0)),
    ((0, -0.0008, 0.5912), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
    ((0, -0.0008, 0.79213), (1, 0, 0), (0, 1, 0), (0, 0, 1)),
]
max_err = 0.0
for i, (o, x, y, z) in enumerate(TABLE):
    for name, calc, ref in (('O', O[i], o), ('x', X[i], x), ('y', Y[i], y), ('z', Z[i], z)):
        err = float(np.max(np.abs(calc - np.array(ref, float))))
        max_err = max(max_err, err)
        assert err < 1e-6, f'{{{i}}} {name}: hesaplanan {calc} != tablo {ref} (hata {err:.3e})'
print(f'assert OK: 7 cerceve x (orijin, x, y, z) rapor tablosuyla eslesti, max |fark| = {max_err:.2e}')
for i in range(7):
    print(f'  {{{i}}} O = ({O[i][0]: .5f}, {O[i][1]: .5f}, {O[i][2]: .5f})  '
          f'x = {np.round(X[i], 6) + 0.0}  z = {np.round(Z[i], 6) + 0.0}')

# ---------------------------------------------------------------- gorunusler
# h, v: yatay/dusey dunya eksen indeksleri; out: ekrandan izleyiciye dogru birim vektor
VIEWS = {
    'yan': dict(h=0, v=2, out=np.array([0.0, -1.0, 0.0])),   # x saga, z yukari -> -y izleyiciye
    'on': dict(h=1, v=2, out=np.array([1.0, 0.0, 0.0])),     # y saga, z yukari -> +x izleyiciye
}
L_ARROW = 0.055   # eksen oku boyu [m]
R_SYM = 0.0115    # dairesel sembol yaricapi [m]


def p2(view, p):
    return np.array([p[view['h']], p[view['v']]])


def draw_arrow(ax, base, vec2, color, length=L_ARROW, lw=2.2):
    tip = base + vec2 * length
    ax.annotate('', xy=tip, xytext=base, zorder=6,
                arrowprops=dict(arrowstyle='-|>', color=color, lw=lw, mutation_scale=16,
                                shrinkA=0, shrinkB=0))
    return tip


def draw_perp(ax, c, sign, color, r=R_SYM):
    """sign > 0: ekrandan izleyiciye dogru (⊙), sign < 0: ekrandan iceri (⊗)."""
    ax.add_patch(Circle(c, r, facecolor=SURFACE, edgecolor=color, lw=1.8, zorder=7))
    if sign > 0:
        ax.add_patch(Circle(c, r * 0.28, facecolor=color, edgecolor='none', zorder=8))
    else:
        k = r * 0.68
        ax.plot([c[0] - k, c[0] + k], [c[1] - k, c[1] + k], color=color, lw=1.6, zorder=8)
        ax.plot([c[0] - k, c[0] + k], [c[1] + k, c[1] - k], color=color, lw=1.6, zorder=8)


def label(ax, xy, s, color=TEXT, fs=11, ha='center', va='center', **kw):
    ax.text(xy[0], xy[1], s, color=color, fontsize=fs, ha=ha, va=va, zorder=10,
            bbox=dict(facecolor=SURFACE, edgecolor='none', pad=0.6, alpha=0.9), **kw)


def draw_axis(ax, view, origin2, vec3, color, name, lab_off, perp_r=R_SYM):
    """Eksen: duzlem icindeyse ok, duzleme dikse ⊙/⊗. lab_off: etiketin konumu icin kaydirma."""
    v2 = p2(view, vec3)
    if np.linalg.norm(v2) > 0.5:
        tip = draw_arrow(ax, origin2, v2 / np.linalg.norm(v2), color)
        label(ax, tip + lab_off, name, color=color)
    else:
        draw_perp(ax, origin2, float(vec3 @ view['out']), color, r=perp_r)
        label(ax, origin2 + lab_off, name, color=color)


def theta_circle(ax, c, sign, name, r=0.03, start=200, span=250, lab_off=(0, 0)):
    """Duzleme dik eksen etrafinda donme isareti. sign>0 (⊙) -> saat yonu tersi, sign<0 -> saat yonu."""
    t = np.radians(start + np.linspace(0, span, 60) * (1 if sign > 0 else -1))
    xs, ys = c[0] + r * np.cos(t), c[1] + r * np.sin(t)
    ax.plot(xs[:-2], ys[:-2], color=THETA, lw=1.4, zorder=5)
    ax.annotate('', xy=(xs[-1], ys[-1]), xytext=(xs[-4], ys[-4]), zorder=5,
                arrowprops=dict(arrowstyle='-|>', color=THETA, lw=1.4, mutation_scale=11,
                                shrinkA=0, shrinkB=0))
    label(ax, np.array(c) + np.array(lab_off), name, color=THETA, fs=11)


def theta_ellipse(ax, c, name, a=0.028, b=0.009, lab_off=(0.045, 0)):
    """Dusey (duzlem ici, yukari) z ekseni etrafinda donme: on (izleyiciye yakin) yarim elips,
    sag-el kuralina gore soldan saga ilerler."""
    t = np.radians(np.linspace(160, 380, 60))
    xs, ys = c[0] + a * np.cos(t), c[1] + b * np.sin(t)
    ax.plot(xs[:-2], ys[:-2], color=THETA, lw=1.4, zorder=9)
    ax.annotate('', xy=(xs[-1], ys[-1]), xytext=(xs[-4], ys[-4]), zorder=9,
                arrowprops=dict(arrowstyle='-|>', color=THETA, lw=1.4, mutation_scale=11,
                                shrinkA=0, shrinkB=0))
    label(ax, np.array(c) + np.array(lab_off), name, color=THETA, fs=11)


def dim_line(ax, p, q, text, off, text_side=1, rot=None, fs=9.5, text_pos=None, ha='center'):
    """Iki uclu olcu oku. p, q: olculen noktalar (2B), off: olcu cizgisinin kaydirma vektoru."""
    p, q, off = np.array(p, float), np.array(q, float), np.array(off, float)
    a, b = p + off, q + off
    n = off / (np.linalg.norm(off) + 1e-12)
    for s, e in ((p, a), (q, b)):    # uzatma cizgileri
        ax.plot([s[0] + n[0] * 0.006, e[0] + n[0] * 0.008], [s[1] + n[1] * 0.006, e[1] + n[1] * 0.008],
                color=DIM, lw=0.6, zorder=3)
    ax.annotate('', xy=b, xytext=a, zorder=4,
                arrowprops=dict(arrowstyle='<|-|>', color=DIM, lw=0.9, mutation_scale=8,
                                shrinkA=0, shrinkB=0))
    mid = (a + b) / 2
    if rot is None:
        rot = 90 if abs(q[1] - p[1]) > abs(q[0] - p[0]) else 0
    tpos = mid + n * 0.016 * text_side if text_pos is None else np.array(text_pos, float)
    ax.text(tpos[0], tpos[1], text, rotation=rot, ha=ha, va='center', fontsize=fs,
            color=DIM, zorder=10, bbox=dict(facecolor=SURFACE, edgecolor='none', pad=0.8))


def skeleton(ax, view, pts):
    # zemin
    ax.axhline(0, color=TEXT2, lw=1.4, zorder=2)
    xl = ax.get_xlim()
    for xx in np.arange(xl[0], xl[1], 0.025):
        ax.plot([xx, xx - 0.015], [0, -0.015], color=GRID, lw=0.9, zorder=1)
    # taban plakasi
    ax.add_patch(Rectangle((-0.06, 0), 0.12, 0.018, facecolor='#d9d7d0', edgecolor=TEXT2,
                           lw=0.8, zorder=2))
    # hafif renkli link seritleri + kalin gri iskelet
    for i in range(6):
        a, b = pts[i], pts[i + 1]
        if np.linalg.norm(b - a) > 1e-9:
            ax.plot([a[0], b[0]], [a[1], b[1]], color=LINK_FILL[i], lw=16, alpha=0.30,
                    solid_capstyle='round', zorder=2)
            ax.plot([a[0], b[0]], [a[1], b[1]], color=LINK, lw=4.5, solid_capstyle='round', zorder=3)
    for pt in pts:
        ax.plot(*pt, 'o', color=TEXT, ms=4.5, zorder=9)


def style(ax, title):
    ax.set_aspect('equal')
    ax.set_facecolor(SURFACE)
    ax.set_title(title, color=TEXT, fontsize=11, loc='left', pad=8)
    ax.grid(color=GRID, lw=0.6, zorder=0)
    ax.set_axisbelow(True)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.tick_params(colors=TEXT2, labelsize=8.5, length=0)


fig, (axs, axf) = plt.subplots(1, 2, figsize=(14.5, 10.6), dpi=220,
                               gridspec_kw=dict(width_ratios=[1.0, 0.86]))
fig.patch.set_facecolor(SURFACE)

# ================================================================ (a) yandan, x-z
V = VIEWS['yan']
axs.set_xlim(-0.30, 0.43)
axs.set_ylim(-0.04, 0.90)
style(axs, '(a) Yandan görünüş: x–z düzlemi  (bakış +y yönünde; +y ekrandan içeri)')
axs.set_xlabel('$x_0$ [m]', color=TEXT, fontsize=10)
axs.set_ylabel('$z_0$ [m]', color=TEXT, fontsize=10)
P = [p2(V, o) for o in O]
skeleton(axs, V, P)

GHOST = np.array([0.175, 0.0])   # izdusumde cakisan cercevelerin yana kaydirilmasi
# her cerceve icin: (cizim orijini, O etiketi kaydirmasi, x etiketi kaydirmasi, z etiketi kaydirmasi)
side_cfg = {
    0: dict(o_off=(-0.085, 0.030), x_off=(0.022, 0.0), z_off=(-0.022, 0.012)),
    1: dict(o_off=(-0.050, 0.020), x_off=(0.022, 0.0), z_off=(-0.012, -0.030)),
    2: dict(o_off=(-0.050, 0.020), x_off=(-0.022, 0.006), z_off=(-0.012, -0.030)),
    3: dict(o_off=(0.0, -0.040), x_off=(0.022, 0.0), z_off=(-0.020, 0.010)),
    4: dict(o_off=(-0.050, 0.020), x_off=(0.012, 0.024), z_off=(-0.012, -0.030)),
    5: dict(o_off=(0.0, -0.040), x_off=(0.022, 0.0), z_off=(-0.020, 0.010)),
    6: dict(o_off=(-0.045, 0.022), x_off=(0.022, 0.0), z_off=(-0.020, 0.012)),
}
for i in range(7):
    c = P[i]
    coincide = [j for j in range(i) if np.linalg.norm(P[j] - P[i]) < 1e-9]
    cfg = side_cfg[i]
    if coincide:
        j = coincide[0]
        g = c + GHOST
        axs.plot([c[0] + 0.012, g[0] - 0.012], [c[1], g[1]], color=TEXT2, lw=0.9,
                 ls=(0, (2, 2)), zorder=4)
        axs.plot(*g, 'o', ms=4.5, mfc=SURFACE, mec=TEXT, zorder=9)
        label(axs, g + np.array(cfg['o_off']), f'$O_{i}$', fs=12)
        axs.text(g[0] + 0.005, g[1] - 0.064, f'($O_{j}$ ile çakışık;\n y = {O[i][1]:+.4f} m)',
                 ha='center', va='top', fontsize=8, color=TEXT2, zorder=10)
        c = g
    else:
        label(axs, c + np.array(cfg['o_off']), f'$O_{i}$', fs=12)
    draw_axis(axs, V, c, X[i], CX, f'$x_{i}$', np.array(cfg['x_off']))
    draw_axis(axs, V, c, Z[i], CZ, f'$z_{i}$', np.array(cfg['z_off']))

# theta: x-z duzlemine dik z1, z2, z4 eksenleri etrafinda (theta2, theta3, theta5)
for i_axis, name, lab in ((1, r'$\theta_2$', (0.040, 0.034)), (2, r'$\theta_3$', (0.040, -0.034)),
                          (4, r'$\theta_5$', (0.040, 0.034))):
    sgn = float(Z[i_axis] @ V['out'])
    theta_circle(axs, P[i_axis], sgn, name, r=0.028, start=110, span=230, lab_off=lab)

# olcu cizgileri (sol tarafta)
zs = [O[0][2], O[1][2], O[2][2], O[4][2], O[6][2]]
dim_line(axs, (0, zs[0]), (0, zs[1]), '$d_1$ = 0.1313', (-0.115, 0), text_side=1)
dim_line(axs, (0, zs[1]), (0, zs[2]), '$a_2$ = 0.22995', (-0.115, 0), text_side=1)
dim_line(axs, (0, zs[2]), (0, zs[3]), '$d_4$ = 0.22995', (-0.115, 0), text_side=1)
dim_line(axs, (0, zs[3]), (0, zs[4]), '$d_6$ = 0.20093', (-0.115, 0), text_side=1)
dim_line(axs, (0, zs[0]), (0, zs[4]), 'toplam yükseklik = 0.79213 m', (-0.205, 0), text_side=1)

# ================================================================ (b) onden, y-z
V = VIEWS['on']
axf.set_xlim(-0.24, 0.32)
axf.set_ylim(-0.04, 0.90)
style(axf, '(b) Önden görünüş: y–z düzlemi  (bakış −x yönünde; +x ekrana doğru)')
axf.set_xlabel('$y_0$ [m]', color=TEXT, fontsize=10)
axf.set_ylabel('$z_0$ [m]', color=TEXT, fontsize=10)
P = [p2(V, o) for o in O]
skeleton(axf, V, P)

front_cfg = {
    0: dict(o_off=(-0.085, 0.030), x_off=(0.020, -0.022), z_off=(-0.022, 0.010)),
    1: dict(o_off=(-0.045, 0.022), x_off=(-0.004, -0.030), z_off=(0.020, 0.012)),
    2: dict(o_off=(-0.045, 0.022), x_off=(-0.022, 0.0), z_off=(-0.026, -0.026)),
    3: dict(o_off=(0.045, -0.026), x_off=(0.030, 0.002), z_off=(0.022, 0.010)),
    4: dict(o_off=(0.010, -0.032), x_off=(0.004, 0.030), z_off=(0.022, 0.012)),
    5: dict(o_off=(-0.045, -0.022), x_off=(-0.030, 0.004), z_off=(-0.022, 0.010)),
    6: dict(o_off=(-0.045, 0.022), x_off=(0.030, -0.006), z_off=(-0.022, 0.010)),
}
for i in range(7):
    c = P[i]
    cfg = front_cfg[i]
    label(axf, c + np.array(cfg['o_off']), f'$O_{i}$', fs=12)
    draw_axis(axf, V, c, X[i], CX, f'$x_{i}$', np.array(cfg['x_off']))
    draw_axis(axf, V, c, Z[i], CZ, f'$z_{i}$', np.array(cfg['z_off']))

# theta: duzlem icinde dusey z0, z3, z5 eksenleri etrafinda (theta1, theta4, theta6)
for i_axis, name, h in ((0, r'$\theta_1$', 0.085), (3, r'$\theta_4$', 0.100), (5, r'$\theta_6$', 0.115)):
    theta_ellipse(axf, P[i_axis] + np.array([0, h]), name)

# yanal kaciklik olculeri
dim_line(axf, P[2], P[3], '$d_3$ = +0.0652', (0, -0.060), text_pos=(0.082, P[2][1] - 0.060), ha='left')
dim_line(axf, P[4], P[5], '$d_5$ = −0.066', (0, -0.075), text_pos=(0.082, P[4][1] - 0.075), ha='left')
dim_line(axf, (P[0][0], 0), (P[0][0], O[6][2]), 'toplam yükseklik = 0.79213 m', (-0.16, 0),
         text_side=1)

# ================================================================ baslik + lejant
fig.suptitle('rk6 – DH çerçeveleri (sıfır poz, q = 0)', color=TEXT, fontsize=15,
             x=0.012, ha='left', y=0.985)
handles = [
    Line2D([], [], color=CX, lw=2.4, marker='>', ms=7, label='$x_i$ ekseni (kırmızı)'),
    Line2D([], [], color=CZ, lw=2.4, marker='>', ms=7, label='$z_i$ ekseni (mavi) = eklem $i+1$ dönme ekseni'),
    Line2D([], [], ls='none', marker=r'$\odot$', ms=13, color=TEXT,
           label='eksen ekrandan izleyiciye doğru çıkar'),
    Line2D([], [], ls='none', marker=r'$\otimes$', ms=13, color=TEXT,
           label='eksen ekrandan içeri girer'),
    Line2D([], [], color=THETA, lw=1.6, label=r'$\theta_i$: $z_{i-1}$ etrafında pozitif dönme (sağ el kuralı)'),
    Line2D([], [], color=LINK, lw=4.5, label='iskelet $O_0 \\to O_6$ (DH orijinleri)'),
    Line2D([], [], color=DIM, lw=0.9, marker='<', ms=4, label='ölçü [m]'),
    Line2D([], [], ls='none', marker='o', ms=6, mfc=SURFACE, mec=TEXT,
           label='izdüşümde çakışan çerçeve (yana kaydırılarak çizildi)'),
]
fig.legend(handles=handles, loc='lower center', ncol=3, fontsize=9.5, frameon=False,
           bbox_to_anchor=(0.5, 0.005), labelcolor=TEXT)
fig.tight_layout(rect=(0, 0.075, 1, 0.965))
fig.savefig(OUT, facecolor=SURFACE)
print('kaydedildi:', OUT)
