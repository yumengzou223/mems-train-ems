"""Render the TLC vs battery-replacement trade-off from the saved Pareto front.

    python plot_tradeoff.py

Reads results/pareto_front_v2.csv, highlights the paper EMS1 baseline (recomputed
with the same model) and writes results/tradeoff_v2.png.
"""

import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import colors

import mrfo_multiobjective_v2 as m

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "results", "pareto_front_v2.csv")
OUT = os.path.join(HERE, "results", "tradeoff_v2.png")

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#d9d8d4"
# Single-hue sequential ramp (blue) from the reference palette.
RAMP = ["#cde2fb", "#86b6ef", "#3987e5", "#1c5cab", "#0d366b"]

d = np.genfromtxt(CSV, delimiter=",", names=True)
tlc = d["TLC_EUR"] / 1e6
bat = d["N_rep_BAT"]
fc_kw = d["FC_nom_scale"] * 820.3

base_x = np.array([0.30, 0.70, 1.0, 200000.0 / 820300.0, 0.90])
base_o, base_s = m.evaluate(base_x)
base_tlc, base_bat, base_fc = base_o[0] / 1e6, base_o[2], base_x[2] * 820.3

cmap = colors.LinearSegmentedColormap.from_list("blue_ramp", RAMP)
norm = colors.Normalize(vmin=fc_kw.min(), vmax=fc_kw.max())

fig, ax = plt.subplots(figsize=(7.2, 4.8), dpi=160)
fig.patch.set_facecolor(SURFACE)
ax.set_facecolor(SURFACE)

order = np.argsort(tlc)
ax.plot(tlc[order], bat[order], color="#b7d3f6", linewidth=2, zorder=1)

sc = ax.scatter(tlc, bat, c=fc_kw, cmap=cmap, norm=norm, s=64,
                edgecolors=SURFACE, linewidths=1.5, zorder=3)

ax.scatter([base_tlc], [base_bat], marker="*", s=260, c=INK,
           edgecolors=SURFACE, linewidths=1.5, zorder=5)

ax.annotate("EMS1 baseline (paper)\n9.33M EUR, 7.03 replacements",
            xy=(base_tlc, base_bat), xytext=(9.06, 6.53),
            fontsize=8.5, color=INK_2, ha="left", va="bottom",
            arrowprops=dict(arrowstyle="-", color=INK_2, linewidth=1))

k_min = int(np.argmin(tlc))
ax.annotate(f"min TLC  {tlc[k_min]:.2f}M EUR",
            xy=(tlc[k_min], bat[k_min]), xytext=(8.70, 7.57),
            fontsize=8.5, color=INK_2,
            arrowprops=dict(arrowstyle="-", color=INK_2, linewidth=1))

ax.set_xlabel("Life-cycle cost TLC  [million EUR]", fontsize=9.5, color=INK_2)
ax.set_ylabel("Battery replacements over 20 years", fontsize=9.5, color=INK_2)
ax.set_title("Cheaper EMS costs more battery wear",
             fontsize=13.5, color=INK, loc="left", pad=26)
ax.text(0, 1.015,
        "40 non-dominated designs, reduced-order model; colour = fuel-cell nominal power",
        transform=ax.transAxes, fontsize=9, color=INK_2)
ax.set_ylim(6.48, 7.66)

ax.grid(True, color=GRID, linewidth=0.8, alpha=0.7)
ax.set_axisbelow(True)
for side in ("top", "right"):
    ax.spines[side].set_visible(False)
for side in ("left", "bottom"):
    ax.spines[side].set_color(GRID)
ax.tick_params(colors=INK_2, labelsize=8.5)

cbar = fig.colorbar(sc, ax=ax, pad=0.03, aspect=26)
cbar.set_label("Fuel-cell nominal power  [kW]", fontsize=9, color=INK_2)
cbar.outline.set_visible(False)
cbar.ax.tick_params(colors=INK_2, labelsize=8)

fig.tight_layout()
fig.savefig(OUT, facecolor=SURFACE, bbox_inches="tight")
print(f"Saved {os.path.relpath(OUT, HERE)}")
