#!/usr/bin/env python3
"""Sekil 5: Home -> K1 -> K2 -> K3 -> Home hareketinde /joint_states eklem acilari.

Kullanim: python3 sekil5_eklem_grafigi.py sonuclar/demo_joints.csv [sonuclar/demo_ee.csv] [cikti.png]
CSV sutunlari: t, joint1..joint6, gripper_left_finger_joint (t = mesaj zaman damgasi, sim zamani).
"""
import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

jpath = sys.argv[1]
epath = sys.argv[2] if len(sys.argv) > 2 and sys.argv[2].endswith('.csv') else None
out = sys.argv[-1] if sys.argv[-1].endswith('.png') else 'sekil5_eklem_grafigi.png'

rows = [list(map(float, r)) for r in list(csv.reader(open(jpath)))[1:]]
t0 = rows[0][0]
t = [r[0] - t0 for r in rows]
COLORS = ['#2a78d6', '#e8710a', '#1e8e3e', '#d93025', '#9334e6', '#7a5c3e']

fig, (ax, axg) = plt.subplots(2, 1, figsize=(8, 5.2), dpi=200, sharex=True,
                              gridspec_kw={'height_ratios': [4, 1]})
for k in range(6):
    ax.plot(t, [r[k + 1] for r in rows], color=COLORS[k], linewidth=1.4, label=f'joint{k + 1}')
ax.set_ylabel('eklem açısı [rad]')
ax.set_title('/joint_states: Home → K1 → K2 → K3 → Home', loc='left', fontsize=10)
ax.grid(True, color='#e4e3df')
ax.legend(loc='lower right', fontsize=7.5, ncol=3, frameon=True)
axg.plot(t, [r[7] * 1000 for r in rows], color='#52514e', linewidth=1.2)
axg.set_ylabel('parmak\n[mm]', fontsize=8)
axg.set_xlabel('zaman [s] (kayıt başlangıcından, sim zamanı)')
axg.grid(True, color='#e4e3df')
if epath:
    ee = [list(map(float, r)) for r in list(csv.reader(open(epath)))[1:]]
    x, y, z = ee[-1][1:]
    fig.text(0.5, 0.012, f'Son /ee_pose (base_link): x={x:+.4f}  y={y:+.4f}  z={z:+.4f} m',
             ha='center', family='monospace', fontsize=8)
fig.tight_layout(rect=(0, 0.04, 1, 1))
fig.savefig(out)
print('kaydedildi:', out, 'ornek:', len(rows), 'sure:', round(t[-1], 2), 's')
