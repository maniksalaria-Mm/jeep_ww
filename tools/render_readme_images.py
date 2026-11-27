"""Render README images for jeep_ww: the SUV from its STL meshes, and the saved maps."""
import struct
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from PIL import Image

PKG = Path(sys.argv[1])
OUT = Path(sys.argv[2])
OUT.mkdir(parents=True, exist_ok=True)


def load_stl(path):
    data = path.read_bytes()
    if data[:5].lower() == b"solid" and b"facet" in data[:400]:
        tris, cur = [], []
        for line in data.decode(errors="ignore").splitlines():
            p = line.split()
            if p and p[0] == "vertex":
                cur.append([float(v) for v in p[1:4]])
                if len(cur) == 3:
                    tris.append(cur)
                    cur = []
        return np.array(tris)
    n = struct.unpack("<I", data[80:84])[0]
    arr = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return arr["v"].reshape(-1, 3, 3).astype(float)


COL = {
    "paint": (0.36, 0.40, 0.18), "glass": (0.10, 0.13, 0.18), "trim": (0.08, 0.08, 0.08),
    "chrome": (0.65, 0.65, 0.68), "headlight": (0.98, 0.98, 0.90), "taillight": (0.85, 0.06, 0.06),
    "tyre": (0.07, 0.07, 0.07), "rim": (0.42, 0.42, 0.45),
}
M = PKG / "meshes"
parts = [("suv_body.stl", "paint"), ("suv_glass.stl", "glass"), ("suv_trim.stl", "trim"),
         ("suv_grille.stl", "chrome"), ("suv_headlights.stl", "headlight"),
         ("suv_taillights.stl", "taillight"), ("suv_spare_tire.stl", "tyre"), ("suv_spare_rim.stl", "rim")]
WB, TR = 3.022, 1.70
meshes = [(load_stl(M / f), c) for f, c in parts]
tire, rim = load_stl(M / "suv_tire.stl"), load_stl(M / "suv_rim.stl")
rot180 = np.diag([-1.0, -1.0, 1.0])
for x in (0.0, WB):
    for side in (1, -1):
        for mesh, c in ((tire, "tyre"), (rim, "rim")):
            m = mesh @ rot180.T if side == -1 else mesh
            meshes.append((m + np.array([x, side * TR / 2, 0.0]), c))


def shade(tris, base, light):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    k = 0.45 + 0.55 * np.abs(n @ light)
    return np.clip(np.outer(k, base), 0, 1)


def suv_view(ax, elev, azim):
    az, el = np.radians(azim), np.radians(elev)
    light = np.array([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    light = 0.6 * light + 0.4 * np.array([0.3, -0.2, 0.93])
    light /= np.linalg.norm(light)
    allv = []
    for tris, c in meshes:
        pc = Poly3DCollection(tris, facecolors=shade(tris, COL[c], light), edgecolors="none", linewidths=0)
        ax.add_collection3d(pc)
        allv.append(tris.reshape(-1, 3))
    v = np.vstack(allv)
    lo, hi = v.min(0), v.max(0)
    ctr, r = (lo + hi) / 2, (hi - lo).max() / 2
    ax.set_xlim(ctr[0] - r, ctr[0] + r); ax.set_ylim(ctr[1] - r, ctr[1] + r); ax.set_zlim(ctr[2] - r, ctr[2] + r)
    ax.set_box_aspect((1, 1, 1))
    # ground shadow plate
    gz = -0.407
    xx, yy = np.meshgrid([lo[0] - 0.6, hi[0] + 0.6], [-1.6, 1.6])
    ax.plot_surface(xx, yy, np.full_like(xx, gz), color=(0.85, 0.85, 0.82), alpha=0.6, zorder=-1)
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


fig = plt.figure(figsize=(16, 6), dpi=110)
fig.patch.set_facecolor("white")
for i, (el, az, t) in enumerate([(18, -55, "front three-quarter"), (2, -90, "side"), (22, 140, "rear three-quarter")]):
    ax = fig.add_subplot(1, 3, i + 1, projection="3d", computed_zorder=False)
    suv_view(ax, el, az)
    ax.set_title(t, fontsize=12, color="#444444", y=0.9)
fig.suptitle("jeep_ww SUV  ·  5.0 m long · 3.022 m wheelbase · 20\" tyres · roof + bumper lidar",
             fontsize=15, fontweight="bold", y=0.97)
plt.subplots_adjust(left=0, right=1, top=0.95, bottom=0, wspace=-0.12)
fig.savefig(OUT / "suv_model.png", facecolor="white")
plt.close(fig)


def map_img(name):
    im = Image.open(PKG / "maps" / f"{name}.pgm").convert("L")
    a = np.array(im)
    rows = np.where((a != 205).any(1))[0]; cols = np.where((a != 205).any(0))[0]
    pad = 20
    a = a[max(rows[0] - pad, 0):rows[-1] + pad, max(cols[0] - pad, 0):cols[-1] + pad]
    rgb = np.zeros(a.shape + (3,), np.uint8)
    rgb[a == 205] = (226, 230, 236)      # unknown
    rgb[a >= 250] = (255, 255, 255)      # free
    rgb[a <= 10] = (40, 52, 70)          # occupied
    other = (a > 10) & (a < 250) & (a != 205)
    rgb[other] = np.stack([a[other]] * 3, -1)
    return rgb


maps = [("mountain_road", "Mountain road  (5 cm)"), ("college_corridor", "College corridor, floor 2  (5 cm)"),
        ("chitkara_campus_roads", "Campus roads  (10 cm)")]
imgs = [map_img(n) for n, _ in maps]
fig, axes = plt.subplots(1, 3, figsize=(16, 7), dpi=110, gridspec_kw={"width_ratios": [i.shape[1] / i.shape[0] for i in imgs]})
for ax, im, (_, t) in zip(axes, imgs, maps):
    ax.imshow(im, interpolation="lanczos")
    ax.set_title(t, fontsize=12, color="#333333")
    ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_color("#bbbbbb")
fig.suptitle("Ready-made occupancy maps in maps/  (SLAM Toolbox · used by Nav2)", fontsize=15, fontweight="bold")
plt.tight_layout(rect=(0, 0, 1, 0.95))
fig.savefig(OUT / "maps_preview.png", facecolor="white")
print("done")
