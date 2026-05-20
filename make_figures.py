"""
Generate proposal-ready figures from the FAA analysis of sample_slope.xyzn.

Reproduces the web app's stereonet (lower-hemisphere, equal-angle) and a
3-D view of the slope coloured by failure mode, using the real algorithms
in faa_core.py. Output PNGs go in ./figures.

Run:  python make_figures.py
"""

import os
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

import faa_core as fc

# ── app theme ────────────────────────────────────────────────────────────────
NAVY   = "#15172b"
PANEL  = "#0d0d1e"
RIM    = "#aaaacc"
RED    = "#ff3333"   # sliding
GREEN  = "#33cc55"   # toppling
CYAN   = "#22ccff"   # wedge
GREY   = "#aaaacc"
TEXT   = "#f0f0f0"

# ── parameters (match README) ──────────────────────────────────────────────────
SLOPE_DIP, SLOPE_DIR = 70.0, 140.0
FRICTION = 30.0
LAT_SLIDE, LAT_TOPPLE = 20.0, 20.0
MIN_ANGLE, K = 30.0, 16

OUTDIR = os.path.join(os.path.dirname(__file__), "figures")
os.makedirs(OUTDIR, exist_ok=True)


def load_xyzn(path):
    pts, nrm = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = line.replace(",", " ").split()
            if len(p) < 6:
                continue
            pts.append([float(p[0]), float(p[1]), float(p[2])])
            nrm.append([float(p[3]), float(p[4]), float(p[5])])
    return np.array(pts), np.array(nrm)


def pole_to_stereo(plunge, trend):
    """Equal-angle lower-hemisphere projection — matches faa_core.js poleToStereo."""
    p, t = np.radians(plunge), np.radians(trend)
    r = np.cos(p) / (1 + np.sin(p))
    return r * np.sin(t), r * np.cos(t)


def normals_to_stereo(n):
    v = n.copy()
    v[v[:, 2] > 0] *= -1
    v /= np.linalg.norm(v, axis=1, keepdims=True)
    pt = fc.vector_to_plunge_and_trend(v)        # [plunge, trend]
    return pole_to_stereo(pt[:, 0], pt[:, 1])


def run_analysis(normals, points):
    slide = fc.sliding_check(normals, SLOPE_DIP, SLOPE_DIR, FRICTION, LAT_SLIDE)
    topple = fc.toppling_check(normals, SLOPE_DIP, SLOPE_DIR, FRICTION, LAT_TOPPLE)
    wedge_mid = fc.wedge_check_point(points, normals, SLOPE_DIP, SLOPE_DIR,
                                     FRICTION, MIN_ANGLE, K)
    slide = np.zeros(len(normals), bool) if slide is None else slide
    topple = np.zeros(len(normals), bool) if topple is None else topple
    wedge_mid = np.empty((0, 3)) if wedge_mid is None else wedge_mid
    return slide, topple, wedge_mid


# ── Figure 1: Stereonet ─────────────────────────────────────────────────────────
def fig_stereonet(normals, slide, topple):
    sx, sy = normals_to_stereo(normals)
    fig, ax = plt.subplots(figsize=(6, 6), dpi=160)
    fig.patch.set_facecolor(PANEL)
    ax.set_facecolor(PANEL)

    rim = plt.Circle((0, 0), 1.0, fill=False, color=RIM, lw=1.6)
    ax.add_patch(rim)
    # cross-hairs
    ax.plot([-1, 1], [0, 0], color=RIM, lw=0.4, alpha=0.35)
    ax.plot([0, 0], [-1, 1], color=RIM, lw=0.4, alpha=0.35)

    stable = ~(slide | topple)
    ax.scatter(sx[stable], sy[stable], s=5, c=GREY, alpha=0.18, linewidths=0)
    ax.scatter(sx[slide], sy[slide], s=8, c=RED, alpha=0.75, linewidths=0,
               label=f"Sliding ({int(slide.sum())})")
    ax.scatter(sx[topple], sy[topple], s=8, c=GREEN, alpha=0.75, linewidths=0,
               label=f"Toppling ({int(topple.sum())})")

    # slope pole marker (white ring) — matches app's drawSlopeMarker
    sp = fc.dip_dip_direction_to_pole(np.array([SLOPE_DIP, SLOPE_DIR]))[0]
    px, py = pole_to_stereo(sp[0], sp[1])
    ax.scatter([px], [py], s=120, facecolors="none", edgecolors="#ffffff",
               linewidths=2, label="Slope pole")

    for txt, x, y in [("N", 0, 1.08), ("S", 0, -1.08), ("E", 1.08, 0), ("W", -1.08, 0)]:
        ax.text(x, y, txt, color=RIM, ha="center", va="center", fontsize=12)

    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("Stereonet — Lower Hemisphere, Equal Angle",
                 color=TEXT, fontsize=12, pad=10)
    leg = ax.legend(loc="lower right", framealpha=0.2, fontsize=9,
                    facecolor=NAVY, edgecolor=RIM)
    for t in leg.get_texts():
        t.set_color(TEXT)

    out = os.path.join(OUTDIR, "fig1_stereonet.png")
    fig.savefig(out, facecolor=PANEL, bbox_inches="tight")
    plt.close(fig)
    return out


# ── Figure 2: 3-D view coloured by failure mode ──────────────────────────────────
def fig_3dview(points, slide, topple, wedge_mid):
    fig = plt.figure(figsize=(8, 6), dpi=160)
    fig.patch.set_facecolor(NAVY)
    ax = fig.add_subplot(111, projection="3d")
    ax.set_facecolor(NAVY)

    stable = ~(slide | topple)
    ax.scatter(points[stable, 0], points[stable, 1], points[stable, 2],
               s=3, c="#5b6184", alpha=0.30, linewidths=0, label="Stable face")
    ax.scatter(points[slide, 0], points[slide, 1], points[slide, 2],
               s=9, c=RED, alpha=0.9, linewidths=0,
               label=f"Sliding ({int(slide.sum())})")
    ax.scatter(points[topple, 0], points[topple, 1], points[topple, 2],
               s=9, c=GREEN, alpha=0.9, linewidths=0,
               label=f"Toppling ({int(topple.sum())})")
    if len(wedge_mid):
        ax.scatter(wedge_mid[:, 0], wedge_mid[:, 1], wedge_mid[:, 2],
                   s=11, c=CYAN, alpha=0.9, linewidths=0,
                   label=f"Wedge ({len(wedge_mid)})")

    ax.view_init(elev=22, azim=-72)
    try:
        ax.set_box_aspect((1, 1, 0.8))
    except Exception:
        pass
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.set_facecolor(NAVY)
        axis.pane.set_edgecolor("#2e3257")
        axis.pane.set_alpha(1.0)
        axis._axinfo["grid"]["color"] = (0.18, 0.20, 0.34, 1.0)
        axis._axinfo["grid"]["linewidth"] = 0.4
        axis.label.set_color(TEXT)
    ax.tick_params(colors="#8892b0", labelsize=7)
    ax.set_xlabel("East"); ax.set_ylabel("North"); ax.set_zlabel("Up")
    ax.set_title(f"FAA Result — Slope {SLOPE_DIP:.0f}/{SLOPE_DIR:.0f}, φ={FRICTION:.0f}",
                 color=TEXT, fontsize=12, pad=8)
    leg = ax.legend(loc="upper left", framealpha=0.2, fontsize=9,
                    facecolor=PANEL, edgecolor="#2e3257")
    for t in leg.get_texts():
        t.set_color(TEXT)

    out = os.path.join(OUTDIR, "fig2_3dview.png")
    fig.savefig(out, facecolor=NAVY, bbox_inches="tight")
    plt.close(fig)
    return out


def main():
    src = os.path.join(os.path.dirname(__file__), "sample_slope.xyzn")
    points, normals = load_xyzn(src)
    slide, topple, wedge_mid = run_analysis(normals, points)
    print(f"points={len(points)}  sliding={int(slide.sum())} "
          f"toppling={int(topple.sum())}  wedge={len(wedge_mid)}")
    f1 = fig_stereonet(normals, slide, topple)
    f2 = fig_3dview(points, slide, topple, wedge_mid)
    print("wrote", f1)
    print("wrote", f2)


if __name__ == "__main__":
    main()
