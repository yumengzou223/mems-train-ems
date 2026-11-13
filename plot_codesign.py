"""Render the co-design front: fuel-cell size vs battery-pack size.

    python plot_codesign.py

Reads results/codesign_pareto.csv (written by experiment_codesign.py) and
writes results/codesign_pareto.png. Colour is the life-cycle cost, so the
cheapest corner of the design space is visually obvious.
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results", "codesign_pareto.csv")
OUT = os.path.join(HERE, "results", "codesign_pareto.png")

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#d9d8d4"
RAMP = ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]

d = np.genfromtxt(CSV, delimiter=",", names=True)
fc_kw = d["FC_nom_scale"] * 820.3
bat_x = d["bat_scale"]
tlc = d["TLC_EUR"] / 1e6

n = len(tlc)
k_min = int(np.argmin(tlc))
base_fc, base_bat, base_tlc = 820.3, 1.0, 9.332345

cmap = colors.LinearSegmentedColormap.from_list("blue_ramp", RAMP)
norm = colors.Normalize(vmin=tlc.min(), vmax=tlc.max())

fig, ax = plt.subplots(figsize=(7.2, 4.8), dpi=160)
fig.patch.set_facecolor(SURFACE)
ax.set_facecolor(SURFACE)

sc = ax.scatter(fc_kw, bat_x, c=tlc, cmap=cmap, norm=norm, s=26,
                edgecolors=SURFACE, linewidths=0.6, alpha=0.9, zorder=3)

ax.scatter([base_fc], [base_bat], marker="*", s=280, c=INK,
           edgecolors=SURFACE, linewidths=1.5, zorder=5)
ax.annotate("EMS1 baseline (paper pack)\n820 kW, 9.33M EUR",
            xy=(base_fc, base_bat), xytext=(640, 1.25),
            fontsize=8.5, color=INK_2, ha="left", va="bottom",
            arrowprops=dict(arrowstyle="-", color=INK_2, linewidth=1))

ax.annotate(f"cheapest design\n{fc_kw[k_min]:.0f} kW, "
            f"{bat_x[k_min]:.2f}x pack, {tlc[k_min]:.2f}M EUR",
            xy=(fc_kw[k_min], bat_x[k_min]), xytext=(330, 2.1),
            fontsize=8.5, color=INK_2, ha="left", va="bottom",
            arrowprops=dict(arrowstyle="-", color=INK_2, linewidth=1))

ax.set_xlabel("Fuel-cell nominal power  [kW]", fontsize=9.5, color=INK_2)
ax.set_ylabel("Battery pack size  [x paper pack, 1178.61 Ah]", fontsize=9.5,
              color=INK_2)
ax.set_title("Co-design pushes the optimum to a smaller stack and a bigger pack",
             fontsize=13, color=INK, loc="left", pad=26)
ax.text(0, 1.015,
        f"{n} non-dominated designs from 6 runs; colour = life-cycle cost",
        transform=ax.transAxes, fontsize=9, color=INK_2)

ax.grid(True, color=GRID, linewidth=0.8, alpha=0.7)
ax.set_axisbelow(True)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
for side in ("left", "bottom"):
    ax.spines[side].set_color(GRID)
ax.tick_params(colors=INK_2, labelsize=8.5)
ax.set_xlim(200, 880)
ax.set_ylim(0.7, 4.15)

cbar = fig.colorbar(sc, ax=ax, pad=0.03, aspect=26)
cbar.set_label("Life-cycle cost TLC  [million EUR]", fontsize=9, color=INK_2)
cbar.outline.set_visible(False)
cbar.ax.tick_params(colors=INK_2, labelsize=8)

fig.tight_layout()
fig.savefig(OUT, facecolor=SURFACE, bbox_inches="tight")
print(f"Saved {os.path.relpath(OUT, HERE)}")
