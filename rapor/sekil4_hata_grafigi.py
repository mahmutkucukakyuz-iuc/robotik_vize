#!/usr/bin/env python3
"""Sekil 4: T1-T10 icin FK - Gazebo Oklid hatasi (cubuk grafik + ortalama cizgisi).

Kullanim: python3 sekil4_hata_grafigi.py sonuclar/fk_error_<tarih>.csv [cikti.png]
Ayrica rapor icin ortalama ve standart sapmayi (n-1) yazdirir.
"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

SERIES = '#2a78d6'      # tek seri rengi
TEXT = '#0b0b0b'
TEXT2 = '#52514e'
GRID = '#e4e3df'
SURFACE = '#fcfcfb'


def mean_std(v):
    m = sum(v) / len(v)
    s = (sum((x - m) ** 2 for x in v) / (len(v) - 1)) ** 0.5
    return m, s


csv_path = sys.argv[1]
out = sys.argv[2] if len(sys.argv) > 2 else 'sekil4_hata.png'
rows = list(csv.DictReader(open(csv_path)))
names = [r['config'] for r in rows]
e_gz = [float(r['e_gz_m']) * 1e6 for r in rows]           # um
e_meas = [float(r['e_gz_meas_m']) * 1e6 for r in rows]
e_ros = [float(r['e_ros_m']) * 1e6 for r in rows]

for label, v in (('e_Gazebo [FK(q_komut)]', e_gz), ('e_Gazebo [FK(q_olculen)]', e_meas),
                 ('e_ROS (/ee_pose)', e_ros)):
    m, s = mean_std(v)
    print(f'{label:26s} ortalama = {m:.4f} um ({m / 1000:.7f} mm), std(n-1) = {s:.4f} um, '
          f'min = {min(v):.4f}, max = {max(v):.4f}')
print('konfigurasyon basina e_Gazebo [um]:', ', '.join(f'{n}={e:.3f}' for n, e in zip(names, e_gz)))

m, _ = mean_std(e_gz)
fig, ax = plt.subplots(figsize=(7.2, 3.6), dpi=200)
fig.patch.set_facecolor(SURFACE)
ax.set_facecolor(SURFACE)
ax.bar(names, e_gz, width=0.55, color=SERIES, zorder=3)
ax.axhline(m, color=TEXT2, linewidth=1.2, linestyle=(0, (4, 3)), zorder=4)
# Ortalama etiketi sol ust koseye (cubuk etiketleriyle cakismasin)
ax.text(0.01, 0.97, f'kesikli çizgi: ortalama {m:.3f} µm', transform=ax.transAxes,
        va='top', ha='left', fontsize=8.5, color=TEXT2, zorder=5,
        bbox=dict(facecolor=SURFACE, edgecolor='none', pad=1.5))
for i, e in enumerate(e_gz):
    ax.text(i, e + max(e_gz) * 0.01, f'{e:.2f}', ha='center', va='bottom', fontsize=7.5,
            color=TEXT2, zorder=5, bbox=dict(facecolor=SURFACE, edgecolor='none', pad=0.8))

ax.set_ylabel('Öklid hatası $e_k$ [µm]', color=TEXT, fontsize=9)
ax.set_xlabel('Test konfigürasyonu', color=TEXT, fontsize=9)
ax.set_title('DH düz kinematiği ile Gazebo ölçümü arasındaki hata (1 µm = 0.001 mm)',
             color=TEXT, fontsize=10, loc='left')
ax.grid(axis='y', color=GRID, linewidth=0.8, zorder=0)
ax.set_axisbelow(True)
ax.set_ylim(0, max(e_gz) * 1.25)
for side in ('top', 'right', 'left'):
    ax.spines[side].set_visible(False)
ax.spines['bottom'].set_color(GRID)
ax.tick_params(colors=TEXT2, labelsize=8, length=0)
fig.tight_layout()
fig.savefig(out, facecolor=SURFACE)
print('kaydedildi:', out)
