#!/usr/bin/env python3
"""Generate the jeep_ww SUV visual meshes (STL, metres).

Not needed at runtime - the STL files are already in meshes/.
Re-run only if you want to change the body shape:
    pip install trimesh shapely numpy manifold3d mapbox_earcut
    python3 tools/make_suv_meshes.py

Frame: base_link = centre of the REAR axle (x forward, y left, z up).
Heights are written as height-above-ground and shifted by the wheel radius.

Size / weight follow a full-size 4-door off-road SUV:
  length 5.02 m, body width 2.01 m, height 1.97 m, wheelbase 3.022 m,
  track 1.70 m, 255/60 R20 tyres (radius 0.407 m), ~2.3 t, 12.84 m turning circle.
The body styling is an original generic design.
"""
import math
import os

import numpy as np
import trimesh
from shapely.geometry import Point, Polygon

WHEEL_R = 0.407
WHEELBASE = 3.022
TRACK = 1.70
TW = 0.255                 # tyre width
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'meshes')

X_REAR, X_FRONT = -0.80, 3.80      # body ends (bumpers / spare tyre stick out further)
BODY_W = 0.94                      # half width of the body sides (flares go to 1.004)
SILL, BELT, ROOF = 0.52, 1.18, 1.92
CAB_W_BELT, CAB_W_ROOF = 0.91, 0.82
COWL_X, WS_TOP_X = 2.12, 1.82      # windshield bottom / top x
ARCH_R = 0.49


def zg(z):
    """height above ground -> base_link z"""
    return z - WHEEL_R


def extrude_xz(poly, y0, y1):
    """Extrude a polygon drawn in the (x, z-above-ground) plane between y0..y1."""
    m = trimesh.creation.extrude_polygon(poly, y1 - y0)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    m.apply_translation([0, y1, -WHEEL_R])
    return m


def box(x0, x1, y0, y1, z0, z1):
    b = trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0])
    b.apply_translation([(x0 + x1) / 2, (y0 + y1) / 2, zg((z0 + z1) / 2)])
    return b


def hull(points):
    return trimesh.convex.convex_hull(np.array(points))


def mirror_y(m):
    c = m.copy()
    c.apply_transform(np.diag([1, -1, 1, 1]))
    c.invert()
    return c


def both_sides(m):
    return [m, mirror_y(m)]


def save(name, parts):
    mesh = trimesh.util.concatenate(parts)
    mesh.export(os.path.join(OUT, name))
    print(f'{name:22s} {len(mesh.faces):6d} faces  bounds {np.round(mesh.bounds, 3).tolist()}')


def ws_x(z):
    """x of the windshield line at height z"""
    return COWL_X - (COWL_X - WS_TOP_X) * (z - BELT) / (ROOF - 0.03 - BELT)


# --------------------------------------------------------------------------
# BODY (paint colour)
# --------------------------------------------------------------------------
lower = Polygon([(X_REAR, SILL), (X_FRONT, SILL), (X_FRONT, BELT - 0.09), (X_FRONT - 0.06, BELT - 0.04),
                 (COWL_X, BELT), (X_REAR, BELT)])
for cx in (0.0, WHEELBASE):
    lower = lower.difference(Point(cx, WHEEL_R).buffer(ARCH_R, 64))
body = [extrude_xz(lower, -BODY_W, BODY_W)]

prof = [(X_REAR + 0.02, BELT), (COWL_X - 0.02, BELT), (WS_TOP_X, ROOF - 0.03), (WS_TOP_X - 0.06, ROOF),
        (X_REAR + 0.10, ROOF), (X_REAR + 0.02, ROOF - 0.05)]
pts = []
for x, z in prof:
    w = CAB_W_BELT if z < (BELT + ROOF) / 2 else CAB_W_ROOF
    pts += [(x, w, zg(z)), (x, -w, zg(z))]
body.append(hull(pts))
save('suv_body.stl', body)

# --------------------------------------------------------------------------
# GLASS (dark tinted)
# --------------------------------------------------------------------------
glass = []
WZ0, WZ1 = BELT + 0.07, ROOF - 0.09


def side_y(z):          # cabin side surface (tumblehome line)
    return CAB_W_BELT + (CAB_W_ROOF - CAB_W_BELT) * (z - BELT) / (ROOF - BELT)


def side_window(poly_xz):
    p = []
    for x, z in poly_xz:
        y = side_y(z)
        p += [(x, y + 0.004, zg(z)), (x, y + 0.014, zg(z))]
    return hull(p)


for win in ([(1.06, WZ0), (ws_x(WZ0) - 0.10, WZ0), (ws_x(WZ1) - 0.10, WZ1), (1.06, WZ1)],
            [(0.04, WZ0), (0.96, WZ0), (0.96, WZ1), (0.04, WZ1)],
            [(X_REAR + 0.10, WZ0), (-0.06, WZ0), (-0.06, WZ1), (X_REAR + 0.14, WZ1)]):
    glass += both_sides(side_window(win))

dx, dz = -(COWL_X - WS_TOP_X), (ROOF - 0.03 - BELT)
nrm = np.array([dz, -dx]) / math.hypot(dx, dz)          # outward normal of the windshield
ws = []
for z in (BELT + 0.06, ROOF - 0.08):
    for off in (0.004, 0.014):
        for y in (-0.78, 0.78):
            ws.append((ws_x(z) + nrm[0] * off, y, zg(z + nrm[1] * off)))
glass.append(hull(ws))
glass.append(box(X_REAR + 0.005, X_REAR + 0.015, -0.70, 0.70, BELT + 0.12, ROOF - 0.12))  # rear window
save('suv_glass.stl', glass)

# --------------------------------------------------------------------------
# TRIM (black plastic / steel)
# --------------------------------------------------------------------------
trim = []
for cx in (0.0, WHEELBASE):                                             # wheel-arch flares
    ring = Point(cx, WHEEL_R).buffer(ARCH_R + 0.09, 64).difference(Point(cx, WHEEL_R).buffer(ARCH_R, 64))
    ring = ring.intersection(Polygon([(cx - 1, WHEEL_R + 0.02), (cx + 1, WHEEL_R + 0.02),
                                      (cx + 1, 2), (cx - 1, 2)]))
    trim += both_sides(extrude_xz(ring, BODY_W - 0.08, 1.004))

trim.append(box(X_FRONT - 0.04, 3.98, -0.99, 0.99, 0.40, 0.66))         # front bumper
trim.append(box(X_REAR - 0.10, X_REAR + 0.04, -0.97, 0.97, 0.42, 0.62))  # rear bumper
trim += both_sides(box(0.55, 2.47, BODY_W - 0.04, BODY_W + 0.08, 0.42, 0.51))   # side steps
trim += both_sides(box(X_REAR + 0.20, WS_TOP_X - 0.25, 0.72, 0.76, ROOF, ROOF + 0.03))  # roof rails
for x in (-0.35, 0.55, 1.40):
    trim.append(box(x - 0.03, x + 0.03, -0.78, 0.78, ROOF + 0.03, ROOF + 0.05))      # cross bars
trim += both_sides(box(COWL_X - 0.16, COWL_X - 0.02, BODY_W - 0.02, BODY_W + 0.20, 1.26, 1.44))  # mirrors
trim.append(box(X_REAR + 0.05, X_FRONT - 0.10, -0.52, 0.52, 0.23, 0.52))  # frame / skid plates
for cx in (0.0, WHEELBASE):                                              # axles + diffs
    ax = trimesh.creation.cylinder(radius=0.065, height=TRACK - 0.2, sections=24)
    ax.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    ax.apply_translation([cx, 0, 0])
    trim.append(ax)
    diff = trimesh.creation.icosphere(subdivisions=2, radius=0.14)
    diff.apply_translation([cx, 0.1, 0])
    trim.append(diff)
for x in (2.04, 1.01, -0.01):                                            # door seams
    trim += both_sides(box(x - 0.004, x + 0.004, BODY_W - 0.001, BODY_W + 0.006, SILL + 0.06, BELT - 0.01))
for x in (1.14, 0.12):                                                   # door handles
    trim += both_sides(box(x - 0.08, x + 0.08, BODY_W, BODY_W + 0.025, BELT - 0.14, BELT - 0.10))
trim.append(box(X_REAR - 0.05, X_REAR, -0.12, 0.12, 0.92, 1.22))         # spare-tyre carrier
trim.append(box(X_FRONT, X_FRONT + 0.015, -0.54, 0.54, 0.64, 1.04))      # grille frame
save('suv_trim.stl', trim)

bars = [box(X_FRONT + 0.015, X_FRONT + 0.035, -0.52, 0.52, z - 0.015, z + 0.015)
        for z in (0.70, 0.78, 0.86, 0.94)]
save('suv_grille.stl', bars)

# --------------------------------------------------------------------------
# LIGHTS
# --------------------------------------------------------------------------
head = both_sides(box(X_FRONT, X_FRONT + 0.025, 0.60, 0.90, 0.90, 1.02))
head += both_sides(box(3.98, 3.99, 0.64, 0.82, 0.50, 0.56))             # fog lights
save('suv_headlights.stl', head)
tail = both_sides(box(X_REAR - 0.015, X_REAR, 0.76, 0.92, 0.82, 1.10))
save('suv_taillights.stl', tail)

# --------------------------------------------------------------------------
# WHEEL: all-terrain tyre + 5-spoke rim (wheel frame, axis = +y, outer face = +y;
# right-hand wheels are rotated 180 deg about z in the URDF)
# --------------------------------------------------------------------------
R_IN = WHEEL_R - 0.03
R_RIM = 0.254                      # 20" rim


def ycyl(r, h, y0, sections=64):
    c = trimesh.creation.cylinder(radius=r, height=h, sections=sections)
    c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    c.apply_translation([0, y0 + h / 2, 0])
    return c


ring = trimesh.creation.annulus(r_min=R_RIM, r_max=R_IN, height=TW, sections=64)
ring.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
tyre = [ring]
n_lugs = 32
for i in range(n_lugs):
    for row, (y0, y1) in enumerate(((-TW / 2, -0.005), (0.005, TW / 2))):
        a0 = 2 * math.pi * (i + 0.5 * row) / n_lugs
        a1 = a0 + 2 * math.pi / n_lugs * 0.62
        p = []
        for a in (a0, a1):
            for r in (R_IN - 0.007, WHEEL_R):
                for y in (y0, y1):
                    p.append((r * math.sin(a), y, r * math.cos(a)))
        tyre.append(hull(p))
save('suv_tire.stl', tyre)

rim = [ycyl(R_RIM, 0.22, -0.11)]                       # barrel
rim.append(ycyl(0.08, 0.06, 0.08))                     # hub
lip = trimesh.creation.annulus(r_min=R_RIM - 0.05, r_max=R_RIM + 0.005, height=0.025, sections=64)
lip.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
lip.apply_translation([0, 0.112, 0])
rim.append(lip)
for k in range(5):
    sp = trimesh.creation.box(extents=[0.055, 0.02, R_RIM - 0.09])
    sp.apply_translation([0, 0, 0.08 + (R_RIM - 0.09) / 2 - 0.01])
    sp.apply_transform(trimesh.transformations.rotation_matrix(2 * math.pi * k / 5, [0, 1, 0]))
    sp.apply_translation([0, 0.112, 0])
    rim.append(sp)
save('suv_rim.stl', rim)

# Spare wheel on the tailgate (axis along x)
for part, out in (('suv_tire.stl', 'suv_spare_tire.stl'), ('suv_rim.stl', 'suv_spare_rim.stl')):
    m = trimesh.load(os.path.join(OUT, part))
    m.apply_transform(trimesh.transformations.rotation_matrix(-math.pi / 2, [0, 0, 1]))
    m.apply_translation([X_REAR - 0.18, 0, zg(1.08)])
    save(out, [m])
