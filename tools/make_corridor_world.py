#!/usr/bin/env python3
"""Generate the 2nd-floor college world (from photos of the Mechatronics department floor).

Writes:
  worlds/college_corridor.sdf        Gazebo world
  textures/corridor_*.png            signs / posters / banners (see corridor_textures.py)
  maps/college_corridor.pgm/.yaml    2D map at lidar height (~0.3 m) for Nav2 / AMCL

    python3 tools/make_corridor_world.py

Layout (x east, y north, metres; robot start (0, 0) facing east = down the main corridor):
  Lobby       x -10..0,   y -4..4    hanging floor-2 direction sign, lift, fire exit
  Corridor A  x 0..34,    y -1.8..1.8  Mechatronics department: faculty room 217, two groups of
                                     lit display cabinets, NB-208 lab, benches, posters, banner
  Stair hall  x 34..41,   y -2.5..2.5  flight up (north half) and flight down (south half)
  Corridor C  x -38..-10, y -1.4..1.4  water cooler, fire exit, chair, posters, cable tray
  Corridor D  x -6.4..-3.6, y 4..30    Tinkerers' Lab, code of conduct, posters
The photos do not show how the corridors join, so the joins are an assumption.
"""
import math
import os
import sys

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
sys.path.insert(0, HERE)
from corridor_textures import make_all  # noqa: E402

make_all(os.path.join(PKG, 'textures'))

H = 3.3              # ceiling
HS = 6.5             # stair-hall ceiling (double height)
WAIN = 1.05          # beige dado height
T = 0.25             # wall thickness
LIDAR_Z = 0.3        # map slice height

WHITE = (0.93, 0.93, 0.91)
BEIGE = (0.82, 0.70, 0.52)
WOOD = (0.80, 0.70, 0.55)
PINK = (0.86, 0.76, 0.78)
FRAME = (0.78, 0.63, 0.42)
CAB = (0.90, 0.84, 0.70)
GREY_CAB = (0.70, 0.68, 0.74)
WARM = (1.0, 0.62, 0.25)
DARK_GLASS = (0.18, 0.19, 0.21)
WIN_GLASS = (0.40, 0.44, 0.48)
FLOOR = (0.90, 0.86, 0.78)
STEP = (0.17, 0.16, 0.16)

items, lights, occ, free = [], [], [], []
_n = [0]


# ------------------------------------------------------------------ SDF primitives
def mat(c, emissive=None, tex=None, transparency=0.0, spec=0.1):
    s = (f'<material><ambient>{c[0]} {c[1]} {c[2]} 1</ambient><diffuse>{c[0]} {c[1]} {c[2]} 1</diffuse>'
         f'<specular>{spec} {spec} {spec} 1</specular>')
    if emissive:
        s += f'<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>'
    if tex:
        em = f'<emissive_map>model://jeep_ww/textures/{tex}</emissive_map>' if emissive else ''
        s += (f'<pbr><metal><albedo_map>model://jeep_ww/textures/{tex}</albedo_map>{em}'
              '<roughness>0.6</roughness><metalness>0.0</metalness></metal></pbr>')
    s += '</material>'
    if transparency:
        s += f'<transparency>{transparency}</transparency>'
    return s


def box(x0, x1, y0, y1, z0, z1, col, collide=False, yaw=0.0, rpy=None, **kw):
    _n[0] += 1
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1)); z0, z1 = sorted((z0, z1))
    r = rpy or (0, 0, yaw)
    pose = f'{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} {(z0 + z1) / 2:.3f} {r[0]:.4f} {r[1]:.4f} {r[2]:.4f}'
    g = f'<geometry><box><size>{x1 - x0:.3f} {y1 - y0:.3f} {z1 - z0:.3f}</size></box></geometry>'
    s = f'<visual name="v{_n[0]}"><pose>{pose}</pose>{g}{mat(col, **kw)}</visual>'
    if collide:
        s += f'<collision name="c{_n[0]}"><pose>{pose}</pose>{g}</collision>'
        if z0 <= LIDAR_Z <= z1 and rpy is None:
            occ.append((x0, x1, y0, y1))
    items.append(s)


def collision_only(x0, x1, y0, y1, z0, z1, mark=True):
    _n[0] += 1
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1))
    items.append(f'<collision name="c{_n[0]}"><pose>{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} {(z0 + z1) / 2:.3f} 0 0 0</pose>'
                 f'<geometry><box><size>{x1 - x0:.3f} {y1 - y0:.3f} {z1 - z0:.3f}</size></box></geometry></collision>')
    if mark:
        occ.append((x0, x1, y0, y1))


def cyl(x, y, z0, z1, r, col, collide=False, rpy='0 0 0', **kw):
    _n[0] += 1
    pose = f'{x:.3f} {y:.3f} {(z0 + z1) / 2:.3f} {rpy}'
    g = f'<geometry><cylinder><radius>{r}</radius><length>{abs(z1 - z0):.3f}</length></cylinder></geometry>'
    s = f'<visual name="v{_n[0]}"><pose>{pose}</pose>{g}{mat(col, **kw)}</visual>'
    if collide:
        s += f'<collision name="c{_n[0]}"><pose>{pose}</pose>{g}</collision>'
    items.append(s)


def sphere(x, y, z, r, col):
    _n[0] += 1
    items.append(f'<visual name="v{_n[0]}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
                 f'<geometry><sphere><radius>{r}</radius></sphere></geometry>{mat(col)}</visual>')


# ------------------------------------------------------------------ wall frames
class Wall:
    """A wall face. axis='x': face is the plane y=c; axis='y': plane x=c. n = +1/-1 points into the room."""

    def __init__(self, axis, c, n):
        self.axis, self.c, self.n = axis, c, n

    def xy(self, a0, a1, d0, d1):
        p0, p1 = self.c + self.n * d0, self.c + self.n * d1
        return (a0, a1, p0, p1) if self.axis == 'x' else (p0, p1, a0, a1)


def bw(w, a0, a1, d0, d1, z0, z1, col, **kw):
    x0, x1, y0, y1 = w.xy(a0, a1, d0, d1)
    box(x0, x1, y0, y1, z0, z1, col, **kw)


def plate(w, a0, a1, z0, z1, tex, d=0.0, t=0.015, col=(1, 1, 1), emissive=None):
    """Textured sign on the wall face, readable from inside the room."""
    x0, x1, y0, y1 = w.xy(a0, a1, d, d + t)
    box(x0, x1, y0, y1, z0, z1, col, yaw=(math.pi if w.n > 0 else 0.0), tex=tex, emissive=emissive)


def wall(w, a0, a1, z_top=H, z_bot=0.0, dado=True):
    """Solid two-tone wall behind face w from a0 to a1."""
    if dado and z_bot < WAIN:
        bw(w, a0, a1, -T, 0, z_bot, WAIN, BEIGE, collide=True)
        bw(w, a0, a1, -T, 0, WAIN, z_top, WHITE, collide=True)
    else:
        bw(w, a0, a1, -T, 0, z_bot, z_top, WHITE, collide=True)
    bw(w, a0, a1, 0, 0.015, 0, 0.1, (0.86, 0.78, 0.6))          # skirting


def door(w, a0, a1, style='wood_glass', tag=None):
    fr = 0.08
    bw(w, a0 - fr, a0, 0, 0.05, 0, 2.25, FRAME)
    bw(w, a1, a1 + fr, 0, 0.05, 0, 2.25, FRAME)
    bw(w, a0 - fr, a1 + fr, 0, 0.05, 2.17, 2.25, FRAME)
    leaf = PINK if style == 'pink' else WOOD
    bw(w, a0, a1, 0, 0.03, 0, 2.17, leaf)
    if style == 'wood_glass':          # faculty-room style: big frosted pane + latch
        bw(w, a0 + 0.12, a0 + 0.12 + (a1 - a0) * 0.55, 0.03, 0.04, 1.15, 1.95, (0.88, 0.9, 0.88))
        bw(w, a1 - 0.28, a1 - 0.06, 0.03, 0.06, 1.0, 1.04, (0.12, 0.12, 0.12))
    elif style == 'pink':              # narrow vertical glass strip
        bw(w, a0 + (a1 - a0) * 0.62, a0 + (a1 - a0) * 0.72, 0.03, 0.04, 0.5, 1.9, (0.85, 0.88, 0.88))
    elif style == 'transom':           # door with glazed transom above (Tinkerers' lab)
        bw(w, a0 - fr, a1 + fr, 0, 0.05, 2.25, 2.85, FRAME)
        bw(w, a0, a1, 0.01, 0.02, 2.3, 2.8, DARK_GLASS)
        bw(w, a0 + 0.25, a0 + 0.3, 0.03, 0.04, 0.4, 1.8, (0.85, 0.88, 0.88))
    else:                              # dark glass panel
        bw(w, a0 + 0.12, a0 + 0.12 + (a1 - a0) * 0.5, 0.03, 0.04, 1.05, 1.95, DARK_GLASS)
        bw(w, a0 + 0.3, a0 + 0.42, 0.04, 0.045, 1.55, 1.67, (0.85, 0.15, 0.15))   # red room tag
    bw(w, a1 - 0.14, a1 - 0.1, 0.03, 0.08, 0.95, 1.15, (0.1, 0.1, 0.1))           # handle
    if tag:
        plate(w, a1 - 0.18, a1 - 0.04, 1.98, 2.12, tag, d=0.035)


def window(w, a0, a1, z0=1.15, z1=2.1, glass=WIN_GLASS):
    bw(w, a0 - 0.07, a1 + 0.07, 0, 0.03, z0 - 0.07, z1 + 0.07, (0.88, 0.86, 0.74))
    bw(w, a0, a1, 0.03, 0.035, z0, z1, glass)
    for k in range(1, max(2, round((a1 - a0) / 0.6))):
        a = a0 + (a1 - a0) * k / max(2, round((a1 - a0) / 0.6))
        bw(w, a - 0.02, a + 0.02, 0.03, 0.045, z0, z1, (0.88, 0.86, 0.74))


def cabinet(w, a0, a1, depth, height, lit=True, side=CAB, back=0.0, seed=0):
    """Glass display cabinet standing against wall w (its back at distance `back` from the face)."""
    t = 0.035
    d0, d1 = back, back + depth
    x0, x1, y0, y1 = w.xy(a0, a1, d0, d1)
    collision_only(x0, x1, y0, y1, 0, height)
    inner = WARM if lit else (0.86, 0.82, 0.72)
    bw(w, a0, a1, d0, d0 + t, 0, height, inner, emissive=(0.95, 0.48, 0.12) if lit else None)
    bw(w, a0, a0 + t, d0, d1, 0, height, side)
    bw(w, a1 - t, a1, d0, d1, 0, height, side)
    bw(w, a0, a1, d0, d1, height - 0.06, height, side)
    bw(w, a0, a1, d0, d1, 0, 0.08, side)
    nd = max(1, round((a1 - a0) / 0.7))
    for i in range(nd + 1):
        a = a0 + (a1 - a0) * i / nd
        bw(w, a - 0.025, a + 0.025, d1 - 0.03, d1, 0.08, height - 0.06, side)
    bw(w, a0, a1, d1 - 0.02, d1 - 0.01, 0.08, height - 0.06, (0.9, 0.9, 0.85), transparency=0.88)
    if lit:
        bw(w, a0 + t, a0 + t + 0.005, d0, d1 - 0.03, 0.08, height - 0.06, WARM, emissive=(0.8, 0.4, 0.1))
        bw(w, a1 - t - 0.005, a1 - t, d0, d1 - 0.03, 0.08, height - 0.06, WARM, emissive=(0.8, 0.4, 0.1))
        bw(w, a0 + t, a1 - t, d0, d1 - 0.03, height - 0.07, height - 0.06, (1, 0.9, 0.7), emissive=(1, 0.85, 0.6))
    rng = np.random.default_rng(seed)
    for z in np.arange(0.45, height - 0.3, 0.5):
        bw(w, a0 + t, a1 - t, d0 + t, d1 - 0.03, z, z + 0.012, (0.95, 0.8, 0.55), transparency=0.3)
        for _ in range(rng.integers(1, 4)):
            s = min(rng.uniform(0.08, 0.22), (a1 - a0 - 0.25) / 2)
            a = rng.uniform(a0 + 0.1 + s, a1 - 0.1 - s)
            hh = rng.uniform(0.06, 0.26)
            col = [(0.85, 0.2, 0.15), (0.2, 0.5, 0.85), (0.95, 0.8, 0.2), (0.2, 0.2, 0.2), (0.75, 0.75, 0.78),
                   (0.2, 0.65, 0.3)][rng.integers(0, 6)]
            bw(w, a - s / 2, a + s / 2, d0 + 0.1, d0 + 0.1 + s * 0.8, z + 0.012, z + 0.012 + hh, col)


def bench(w, a0, a1):
    bw(w, a0 + 0.05, a1 - 0.05, 0.0, 0.62, 0.0, 0.3, (0.62, 0.61, 0.58), collide=True)
    bw(w, a0, a1, 0.0, 0.65, 0.3, 0.45, (0.72, 0.71, 0.68))
    bw(w, a0, a1, 0.0, 0.16, 0.45, 0.9, (0.72, 0.71, 0.68))


def plant(x, y, big=False):
    r = 0.2 if big else 0.16
    cyl(x, y, 0.0, 0.34, r, (0.62, 0.30, 0.18), collide=True)
    occ.append((x - r, x + r, y - r, y + r))
    for dx, dy, dz, rr in ((0, 0, 0.55, 0.25), (0.12, 0.08, 0.72, 0.18), (-0.1, -0.06, 0.74, 0.17)):
        sphere(x + dx, y + dy, dz, rr * (1.2 if big else 1.0), (0.18, 0.45, 0.16))


def chair(x, y):
    for dx in (-0.2, 0.2):
        for dy in (-0.2, 0.2):
            cyl(x + dx, y + dy, 0, 0.46, 0.012, (0.1, 0.1, 0.1))
    box(x - 0.22, x + 0.22, y - 0.22, y + 0.22, 0.44, 0.47, (0.12, 0.12, 0.12))
    for k in range(4):
        box(x - 0.22, x + 0.22, y + 0.2, y + 0.22, 0.6 + k * 0.12, 0.63 + k * 0.12, (0.12, 0.12, 0.12))
    collision_only(x - 0.22, x + 0.22, y - 0.22, y + 0.22, 0, 0.47)


def tube_light(x, y, along='x'):
    if along == 'x':
        box(x - 0.6, x + 0.6, y - 0.05, y + 0.05, H - 0.05, H, (1, 1, 1), emissive=(1, 1, 1))
    else:
        box(x - 0.05, x + 0.05, y - 0.6, y + 0.6, H - 0.05, H, (1, 1, 1), emissive=(1, 1, 1))
    lights.append((x, y, H - 0.4, (1.0, 1.0, 0.97), 0.85, 9.0))


def fan(x, y, top=H):
    cyl(x, y, 2.75, top, 0.022, (0.95, 0.95, 0.95))
    cyl(x, y, 2.66, 2.78, 0.12, (0.95, 0.95, 0.95))
    for k in range(3):
        a = 2 * math.pi * k / 3 + 0.4
        _n[0] += 1
        items.append(f'<visual name="v{_n[0]}"><pose>{x + 0.6 * math.cos(a):.3f} {y + 0.6 * math.sin(a):.3f} 2.7 0 0 {a:.3f}</pose>'
                     f'<geometry><box><size>1.0 0.12 0.01</size></box></geometry>{mat((0.95, 0.95, 0.95))}</visual>')


def floor(x0, x1, y0, y1, z=0.0, col=FLOOR, grid=0.8):
    box(x0, x1, y0, y1, z - 0.06, z, col, collide=True, spec=0.6)
    free.append((x0, x1, y0, y1))
    for x in np.arange(math.ceil(x0 / grid) * grid, x1, grid):
        box(x - 0.006, x + 0.006, y0, y1, z, z + 0.002, (0.75, 0.71, 0.63))
    for y in np.arange(math.ceil(y0 / grid) * grid, y1, grid):
        box(x0, x1, y - 0.006, y + 0.006, z, z + 0.002, (0.75, 0.71, 0.63))


def ceiling(x0, x1, y0, y1, z=H):
    box(x0, x1, y0, y1, z, z + 0.15, (0.96, 0.96, 0.95))


def cable_tray(x0, x1, y0, y1, z=2.85):
    box(x0, x1, y0, y1, z, z + 0.08, (0.62, 0.64, 0.66))
    along_x = (x1 - x0) > (y1 - y0)
    for s in np.arange(x0 if along_x else y0, x1 if along_x else y1, 1.5):
        if along_x:
            cyl(s, (y0 + y1) / 2, z, H, 0.008, (0.4, 0.4, 0.4))
        else:
            cyl((x0 + x1) / 2, s, z, H, 0.008, (0.4, 0.4, 0.4))


def banner_beam_x(x, y0, y1, tex, face=-1, z0=2.55):
    """Beam across a corridor that runs along x; banner readable from -x (face=-1) or +x (face=+1)."""
    box(x - 0.15, x + 0.15, y0, y1, z0, H, (0.97, 0.97, 0.97))
    xp = x - 0.17 if face < 0 else x + 0.16
    box(xp, xp + 0.012, y0 + 0.1, y1 - 0.1, z0 + 0.05, H - 0.05, (1, 1, 1), yaw=(math.pi if face > 0 else 0), tex=tex)


def banner_beam_y(y, x0, x1, tex, face=-1, z0=2.55):
    box(x0, x1, y - 0.15, y + 0.15, z0, H, (0.97, 0.97, 0.97))
    yp = y - 0.17 if face < 0 else y + 0.16
    box(x0 + 0.1, x1 - 0.1, yp, yp + 0.012, z0 + 0.05, H - 0.05, (1, 1, 1), yaw=(math.pi if face > 0 else 0), tex=tex)


# =================================================================== FLOORS & CEILINGS
floor(0, 34, -1.8, 1.8)                       # corridor A
floor(5.6, 28, -2.3, -1.8)                    # cabinet alcove floor
floor(34, 35.5, -2.5, 2.5)                    # stair-hall landing
floor(-10, 0, -4, 4)                          # lobby
floor(-38, -10, -1.4, 1.4)                    # corridor C
floor(-6.4, -3.6, 4, 30)                      # corridor D
floor(-12.6, -10.9, -2.0, -1.4)               # water-cooler alcove
for x0, x1, y0, y1 in ((0, 34, -2.6, 1.8), (-10, 0, -4, 4), (-38, -10, -1.4, 1.4), (-6.4, -3.6, 4, 30)):
    ceiling(x0, x1, y0, y1)
ceiling(34, 41, -2.5, 2.5, HS)
box(0, 34, -2.3, -1.3, 2.75, H, (0.95, 0.95, 0.94))       # deep side beam over the cabinets (corridor A)
box(-10, 0, -0.15, 0.15, 3.0, H, (0.95, 0.95, 0.94))       # lobby beams
box(-6.4, -3.6, 15, 15.3, 2.9, H, (0.95, 0.95, 0.94))

# =================================================================== CORRIDOR A (Mechatronics)
AS_FRONT = Wall('x', -1.8, +1)       # south wall face (right-hand side walking east)
AS_BACK = Wall('x', -2.3, +1)        # alcove back wall
AN = Wall('x', 1.8, -1)              # north wall face
wall(AS_FRONT, 0, 5.6)
wall(AS_BACK, 5.6, 28)
wall(AS_FRONT, 28, 34)
wall(Wall('y', 5.6, +1), -2.55, -1.8)         # alcove ends
wall(Wall('y', 28, -1), -2.55, -1.8)
bw(AS_BACK, 18.0, 18.6, 0, 0.5, 0, WAIN, BEIGE, collide=True)     # pier between the two cabinet groups
bw(AS_BACK, 18.0, 18.6, 0, 0.5, WAIN, H, WHITE, collide=True)
wall(AN, 0, 34)

# south side, walking east
door(AS_FRONT, 1.0, 1.95, 'wood_glass', 'corridor_room_tag.png')
window(AS_FRONT, 2.5, 3.3, 1.2, 2.1)
door(AS_FRONT, 4.0, 5.0, 'wood_glass', 'corridor_room_217.png')                  # faculty room 217
plate(AS_FRONT, 5.15, 5.45, 2.0, 2.3, 'corridor_faculty_sign.png')
plate(AS_FRONT, 3.5, 3.85, 1.55, 1.65, 'corridor_cctv.png')
cyl(5.5, -1.7, 3.12, 3.22, 0.05, (0.95, 0.95, 0.95))                              # CCTV dome
# cabinet group 1 (photo 1)
cabinet(AS_BACK, 5.65, 7.15, 0.5, 2.45, seed=1)
plate(AS_BACK, 5.95, 6.85, 2.02, 2.3, 'corridor_clock.png', d=0.38, emissive=(1, 1, 1))
plate(AS_BACK, 5.95, 6.85, 1.86, 1.96, 'corridor_lab_strip.png', d=0.38)
cabinet(AS_BACK, 7.15, 7.8, 0.5, 2.45, seed=2)
window(AS_BACK, 8.1, 9.4, 1.2, 2.1)
cabinet(AS_BACK, 8.0, 9.5, 0.45, 0.95, seed=3)
door(AS_BACK, 9.75, 10.65, 'dark')
window(AS_BACK, 10.95, 12.2, 1.2, 2.1)
cabinet(AS_BACK, 10.9, 12.3, 0.45, 0.92, seed=4)
cabinet(AS_BACK, 12.3, 12.95, 0.5, 2.3, lit=False, side=GREY_CAB)
cabinet(AS_BACK, 12.95, 14.05, 0.5, 2.3, seed=5)
door(AS_BACK, 14.4, 15.3, 'dark')
window(AS_BACK, 15.6, 16.1, 1.2, 2.1)
door(AS_BACK, 16.5, 17.4, 'dark')
# cabinet group 2 (photos 4 and 6)
cabinet(AS_BACK, 18.7, 20.0, 0.5, 2.45, seed=6)
sphere(18.95, -2.0, 0.55, 0.05, (1.0, 0.95, 0.3))                                 # bulb on a shelf
window(AS_BACK, 20.25, 21.3, 1.2, 2.1)
cabinet(AS_BACK, 20.2, 21.6, 0.45, 0.95, seed=7)
door(AS_BACK, 21.9, 22.8, 'dark')
window(AS_BACK, 23.1, 24.2, 1.2, 2.1)
cabinet(AS_BACK, 24.2, 25.2, 0.45, 0.95, side=GREY_CAB, seed=8)
cabinet(AS_BACK, 25.2, 25.8, 0.5, 2.35, lit=False, side=GREY_CAB)
cabinet(AS_BACK, 25.8, 26.9, 0.5, 2.35, seed=9)
plate(AS_BACK, 26.0, 26.7, 2.0, 2.22, 'corridor_clock.png', d=0.38, emissive=(1, 1, 1))
door(AS_BACK, 27.05, 27.85, 'wood_glass')
# past the banner (south)
plate(AS_FRONT, 28.3, 28.75, 2.0, 2.16, 'corridor_exit.png')
door(AS_FRONT, 28.9, 29.8, 'dark')
window(AS_FRONT, 30.1, 30.8, 1.2, 2.1)
door(AS_FRONT, 31.2, 32.1, 'wood_glass')
window(AS_FRONT, 32.4, 33.0, 1.2, 2.1, glass=(0.9, 0.92, 0.9))
plant(33.5, -1.5)

# north side, walking east
plate(AN, 0.1, 0.95, 1.2, 2.0, 'corridor_cork.png', d=0.02)
door(AN, 1.15, 2.05, 'pink')
plate(AN, 2.2, 2.55, 1.9, 2.25, 'corridor_nb208.png')
plate(AN, 2.8, 4.4, 1.7, 3.15, 'corridor_mech_poster.png', d=0.02)
plate(AN, 4.7, 6.3, 1.45, 3.15, 'corridor_recruiters.png', d=0.03)
bench(AN, 3.2, 5.1)
window(AN, 6.7, 7.6, 1.2, 2.1)
bw(AN, 7.9, 8.7, 0, 0.25, 1.25, 2.1, (0.88, 0.86, 0.74))                          # wall display box
bw(AN, 7.95, 8.65, 0.25, 0.26, 1.3, 2.05, (0.9, 0.9, 0.85), transparency=0.6)
plate(AN, 8.05, 8.55, 1.4, 1.95, 'corridor_cork.png', d=0.05)
window(AN, 9.0, 9.8, 1.2, 2.1)
door(AN, 10.5, 11.4, 'pink')
door(AN, 12.2, 13.1, 'dark')
plant(14.85, 1.55)
window(AN, 15.0, 17.6, 0.95, 2.3)                                                 # big lab window
bw(AN, 15.6, 16.2, -0.2, -0.15, 1.9, 2.1, (0.1, 0.05, 0.05), emissive=(0.9, 0.1, 0.1))  # lab clock inside
bench(AN, 15.3, 17.3)
plant(17.75, 1.55)
window(AN, 18.3, 20.1, 1.2, 2.1)
plate(AN, 20.4, 21.2, 1.3, 2.4, 'corridor_mech_poster.png', d=0.02)
door(AN, 21.6, 22.5, 'pink')
window(AN, 23.0, 23.8, 1.2, 2.1)
door(AN, 24.3, 25.2, 'dark')
window(AN, 25.6, 26.2, 1.2, 2.1)
door(AN, 26.6, 27.5, 'dark')
door(AN, 29.0, 29.9, 'dark')
window(AN, 30.3, 31.0, 1.2, 2.1)
plate(AN, 31.2, 31.9, 1.3, 1.9, 'corridor_cork.png', d=0.02)
door(AN, 32.2, 33.1, 'wood_glass')
cable_tray(10, 26, 1.2, 1.6)
banner_beam_x(28.0, -1.3, 1.8, 'corridor_banner.png', face=-1)     # south end sits under the deep beam
cyl(27.7, 1.6, 2.5, 2.6, 0.05, (0.95, 0.95, 0.95))                                # CCTV under the banner
for x in (3, 9, 15, 21, 26, 31.5):
    tube_light(x, 0.3)
for x in (5.5, 12.5, 19.5, 30.5):
    fan(x, -0.4)

# =================================================================== STAIR HALL (east end)
SN = Wall('x', 2.5, -1)
SS = Wall('x', -2.5, +1)
SE = Wall('y', 41, -1)
wall(Wall('y', 34, +1), 1.8, 2.5 + T, z_top=HS)
wall(Wall('y', 34, +1), -2.5 - T, -1.8, z_top=HS)
box(34, 34.3, -1.8, 1.8, H, HS, WHITE)                                             # wall above the opening
wall(SN, 34, 41.25, z_top=HS, z_bot=-2.5)
wall(SS, 34, 41.25, z_top=HS, z_bot=-2.5)
wall(SE, -2.75, 2.75, z_top=HS, z_bot=-2.5, dado=False)
# flight up (north half) and landing
NSTEP, RUN, RISE = 15, 0.28, 0.15
for i in range(NSTEP):
    x0 = 35.5 + i * RUN
    box(x0, x0 + RUN, 0.25, 2.5, 0, RISE * (i + 1), STEP, collide=True)
    box(x0, x0 + 0.02, 0.25, 2.5, RISE * (i + 1) - 0.02, RISE * (i + 1) + 0.004, (0.3, 0.3, 0.3))
TOP = 35.5 + NSTEP * RUN
box(TOP, 41, -2.5, 2.5, RISE * NSTEP - 0.2, RISE * NSTEP, STEP, collide=True)       # half landing
# flight down (south half) into a stairwell
for i in range(NSTEP):
    x0 = 35.5 + i * RUN
    box(x0, x0 + RUN, -2.5, -0.1, -2.5, -RISE * (i + 1), STEP, collide=True)
box(TOP, 41, -2.5, -0.1, -2.5, -RISE * NSTEP, STEP, collide=True)
occ.append((35.5, 41, -2.5, 2.5))                                                 # stairs: keep-out for robots
# parapet between the flights + rail + orange panel
box(35.5, TOP, -0.1, 0.25, -2.5, 0.9, WHITE, collide=True)
ang = math.atan2(RISE, RUN)
L = NSTEP * RUN / math.cos(ang)
_n[0] += 1
items.append(f'<visual name="v{_n[0]}"><pose>{(35.5 + TOP) / 2:.3f} 0.2 {1.0 + RISE * NSTEP / 2:.3f} 0 {-ang:.4f} 0</pose>'
             f'<geometry><box><size>{L:.3f} 0.05 0.05</size></box></geometry>{mat((0.75, 0.75, 0.78), spec=0.8)}</visual>')
box(35.2, 35.5, -0.1, 0.25, 0, 1.1, (0.95, 0.62, 0.12))                           # orange newel panel
box(35.2, 35.5, -0.1, 0.25, 1.1, 1.15, (0.1, 0.1, 0.1))
# tall window + red column, fire-hose box, exit sign
bw(SE, -1.7, -0.5, 0, 0.03, -2.0, 5.8, (0.15, 0.15, 0.15))
bw(SE, -1.6, -0.6, 0.03, 0.035, -1.9, 5.7, (0.72, 0.85, 0.95), emissive=(0.35, 0.45, 0.5))
bw(SE, -0.45, -0.15, 0, 0.3, -2.5, HS, (0.80, 0.12, 0.12))
bw(SE, 0.9, 1.5, 0, 0.18, 3.3, 3.9, (0.85, 0.1, 0.1))
bw(SE, 0.95, 1.45, 0.18, 0.19, 3.35, 3.85, (0.95, 0.85, 0.85), transparency=0.5)
plate(SN, 35.0, 35.45, 1.95, 2.1, 'corridor_exit.png')
lights.append((37.5, 0, HS - 0.5, (1.0, 1.0, 0.97), 0.9, 12.0))
lights.append((38.0, -1.2, -0.5, (1.0, 1.0, 0.97), 0.5, 6.0))
box(34.5, 35.7, 1.0, 1.1, HS - 0.05, HS, (1, 1, 1), emissive=(1, 1, 1))

# =================================================================== LOBBY (junction, floor 2)
LS = Wall('x', -4, +1)
LN = Wall('x', 4, -1)
LE = Wall('y', 0, -1)
LW = Wall('y', -10, +1)
wall(LS, -10.25, 0.25)
wall(LN, -10.25, -6.4)
wall(LN, -3.6, 0.25)
wall(LE, -4, -1.8)
wall(LE, 1.8, 4)
wall(LW, -4, -1.4)
wall(LW, 1.4, 4)
# lift
bw(LS, -8.2, -6.6, 0, 0.06, 0, 2.3, (0.55, 0.56, 0.58))
bw(LS, -8.1, -7.42, 0.06, 0.08, 0, 2.2, (0.72, 0.73, 0.75), spec=0.9)
bw(LS, -7.38, -6.7, 0.06, 0.08, 0, 2.2, (0.72, 0.73, 0.75), spec=0.9)
plate(LS, -7.75, -7.05, 2.4, 2.75, 'corridor_lift.png', d=0.02)
# fire exit door
door(LS, -3.6, -2.6, 'dark')
plate(LS, -3.45, -2.75, 2.35, 2.6, 'corridor_exit.png', emissive=(0.2, 0.6, 0.3))
# fire-action sign on the pillar by corridor A, CCTV notice, posters
plate(LE, -3.6, -2.4, 1.2, 2.1, 'corridor_fire_action.png', d=0.01)
plate(LE, 2.4, 3.2, 1.5, 1.75, 'corridor_cctv.png')
plate(LN, -9.5, -8.3, 1.2, 2.6, 'corridor_vision.png', d=0.01)
window(LN, -8.0, -7.0, 1.15, 2.1)
plate(LN, -1.6, -0.6, 1.2, 2.6, 'corridor_quality.png', d=0.01)
plant(-3.3, 3.6, big=True)
plant(-0.4, -3.6)
box(-6.0, -4.5, -1.2, 0.2, 0, 0.012, (0.18, 0.18, 0.18))                            # door mats
box(-9.4, -8.0, -0.8, 0.8, 0, 0.012, (0.18, 0.18, 0.18))
# hanging "2" direction sign (two faces)
for y in (-0.6, 0.6):
    cyl(-5.0 + y * 1.4, 0.0, 2.75, 3.0, 0.006, (0.3, 0.3, 0.3))
box(-6.2, -3.8, -0.03, 0.03, 2.35, 2.75, (0.1, 0.1, 0.1))
box(-6.2, -3.8, -0.045, -0.033, 2.35, 2.75, (1, 1, 1), tex='corridor_dir_a.png')
box(-6.2, -3.8, 0.033, 0.045, 2.35, 2.75, (1, 1, 1), yaw=math.pi, tex='corridor_dir_b.png')
# banners over the three corridor mouths
banner_beam_y(4.0, -6.4, -3.6, 'corridor_q_purple.png', face=-1, z0=2.45)
banner_beam_x(-10.0, -1.4, 1.4, 'corridor_q_blue.png', face=+1, z0=2.45)
banner_beam_x(0.0, -1.8, 1.8, 'corridor_q_green.png', face=-1, z0=2.55)
for x, y in ((-7.5, -2), (-2.5, 2), (-7.5, 2), (-2.5, -2)):
    tube_light(x, y)
fan(-2.0, -2.3)

# =================================================================== CORRIDOR C (west: water cooler)
CN = Wall('x', 1.4, -1)
CS = Wall('x', -1.4, +1)
wall(CN, -38.25, -10)
wall(CS, -38.25, -12.6)
wall(CS, -10.9, -10)
wall(Wall('y', -38, +1), -1.4, 1.4)
# water-cooler alcove (south)
CA = Wall('x', -2.0, +1)
wall(CA, -12.6, -10.9)
wall(Wall('y', -12.6, +1), -2.25, -1.4)
wall(Wall('y', -10.9, -1), -2.25, -1.4)
box(-12.55, -10.95, -2.0, -1.45, 0, 0.9, (0.2, 0.18, 0.18), collide=True)          # stone counter
box(-12.55, -10.95, -2.0, -1.95, 0.9, 2.2, (0.12, 0.11, 0.11))
plate(Wall('y', -12.6, +1), -1.95, -1.6, 1.7, 2.0, 'corridor_water.png')
plate(CS, -13.15, -12.85, 1.9, 2.2, 'corridor_water.png')
plate(CS, -13.9, -13.3, 1.5, 1.72, 'corridor_exit.png')
chair(-13.6, -1.1)
door(CS, -15.3, -14.4, 'wood_glass')
cyl(-16.3, -1.3, 0.8, 1.35, 0.08, (0.8, 0.1, 0.1))                                 # fire extinguisher
bw(CS, -16.6, -16.0, 0, 0.02, 1.5, 2.1, (0.85, 0.2, 0.15))
plate(CS, -17.5, -17.0, 1.2, 1.9, 'corridor_conduct.png', d=0.01)
window(CS, -19.2, -18.2, 1.2, 2.1)
plant(-20.6, -1.15)
door(CS, -22.2, -21.3, 'dark')
window(CS, -24.5, -23.5, 1.2, 2.1)
plant(-26.0, -1.15)
door(CS, -28.6, -27.7, 'dark')
window(CS, -31.0, -30.0, 1.2, 2.1)
door(CS, -34.0, -33.1, 'wood_glass')
# north side
plate(CN, -11.6, -10.6, 1.6, 1.85, 'corridor_cctv.png')
plate(CN, -13.1, -12.4, 1.3, 2.4, 'corridor_vision.png', d=0.01)
window(CN, -14.8, -13.5, 1.0, 2.2, glass=(0.55, 0.6, 0.62))
plate(CN, -16.2, -15.2, 1.2, 2.7, 'corridor_quality.png', d=0.01)
window(CN, -20.0, -19.0, 1.2, 2.1)
door(CN, -23.0, -22.1, 'dark')
window(CN, -26.0, -25.0, 1.2, 2.1)
door(CN, -29.6, -28.7, 'dark')
window(CN, -33.0, -32.0, 1.2, 2.1)
# far end wall: door + windows + plants
CW = Wall('y', -38, +1)
door(CW, -0.45, 0.45, 'wood_glass')
window(CW, -1.2, -0.65, 1.0, 2.6, glass=(0.95, 0.85, 0.65))
window(CW, 0.65, 1.2, 1.0, 2.6, glass=(0.95, 0.85, 0.65))
plant(-37.6, -1.0)
plant(-37.6, 1.0)
banner_beam_x(-16.5, -1.4, 1.4, 'corridor_q_red.png', face=+1, z0=2.5)
banner_beam_x(-27.0, -1.4, 1.4, 'corridor_q_orange.png', face=+1, z0=2.6)
cable_tray(-36, -11, -0.25, 0.25, 2.8)
for x in (-13, -19, -25, -31, -36):
    tube_light(x, 0.6)
for x in (-14.5, -22, -30):
    fan(x, -0.3)

# =================================================================== CORRIDOR D (north: Tinkerers' Lab)
DW = Wall('y', -6.4, +1)
DE = Wall('y', -3.6, -1)
wall(DW, 4, 30.25)
wall(DE, 4, 30.25)
wall(Wall('x', 30, -1), -6.65, -3.35)
plate(DW, 4.4, 5.3, 1.2, 2.6, 'corridor_vision.png', d=0.01)
plate(DW, 5.7, 6.5, 1.1, 2.5, 'corridor_conduct.png', d=0.01)
window(DW, 6.9, 8.0, 1.25, 2.15)
plate(DW, 8.3, 8.75, 1.4, 2.0, 'corridor_tinkerers.png', d=0.02)
door(DW, 9.0, 9.95, 'transom')
plate(DW, 10.2, 10.55, 1.5, 1.95, 'corridor_cork.png', d=0.02)
window(DW, 11.0, 12.1, 1.25, 2.6)
plate(DW, 12.5, 12.85, 1.1, 1.9, 'corridor_quality.png', d=0.01)
plate(DW, 13.1, 13.45, 1.1, 1.9, 'corridor_vision.png', d=0.01)
window(DW, 13.9, 14.7, 1.25, 2.6)
door(DW, 17.0, 17.9, 'dark')
window(DW, 19.0, 20.0, 1.2, 2.1)
door(DW, 23.0, 23.9, 'dark')
window(DW, 26.0, 27.0, 1.2, 2.1)
door(DE, 6.2, 7.1, 'wood_glass')
window(DE, 8.5, 9.5, 1.2, 2.1)
door(DE, 12.0, 12.9, 'dark')
window(DE, 15.5, 16.5, 1.2, 2.1)
door(DE, 19.5, 20.4, 'dark')
plant(-3.95, 21.5)
plant(-6.05, 25.0)
window(DE, 24.0, 25.0, 1.2, 2.1)
cyl(-3.75, 27.8, 0.8, 1.35, 0.08, (0.8, 0.1, 0.1))                                 # extinguisher
bw(DE, 27.6, 28.0, 0, 0.02, 1.5, 2.0, (0.85, 0.2, 0.15))
DN = Wall('x', 30, -1)
door(DN, -5.45, -4.55, 'wood_glass')
window(DN, -6.2, -5.7, 1.2, 2.1, glass=(0.95, 0.85, 0.65))
window(DN, -4.3, -3.8, 1.2, 2.1, glass=(0.95, 0.85, 0.65))
banner_beam_y(18.0, -6.4, -3.6, 'corridor_q_orange.png', face=-1, z0=2.6)
cable_tray(-4.1, -3.8, 5, 29, 2.85)
for y in (7, 13, 19, 25):
    tube_light(-5.0, y, along='y')
for y in (10, 22):
    fan(-5.0, y)

# =================================================================== write SDF
light_sdf = []
for i, (x, y, z, c, inten, rng_) in enumerate(lights):
    light_sdf.append(f"""    <light type="point" name="light{i}">
      <pose>{x:.2f} {y:.2f} {z:.2f} 0 0 0</pose>
      <diffuse>{c[0]} {c[1]} {c[2]} 1</diffuse><specular>0.1 0.1 0.1 1</specular>
      <intensity>{inten}</intensity>
      <attenuation><range>{rng_}</range><constant>0.4</constant><linear>0.1</linear><quadratic>0.02</quadratic></attenuation>
      <cast_shadows>false</cast_shadows>
    </light>""")

sdf = f"""<?xml version="1.0"?>
<!-- Generated by tools/make_corridor_world.py - edit that script, not this file. -->
<sdf version="1.9">
  <world name="college_corridor">
    <physics name="2ms" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.72 0.72 0.72 1</ambient>
      <background>0.80 0.86 0.92 1</background>
      <shadows>false</shadows>
    </scene>
    <light type="directional" name="fill">
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse><specular>0.05 0.05 0.05 1</specular>
      <direction>0.3 0.4 -1</direction>
      <cast_shadows>false</cast_shadows>
    </light>
{chr(10).join(light_sdf)}
    <model name="safety_ground">
      <static>true</static>
      <pose>0 0 -3 0 0 0</pose>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry></collision>
      </link>
    </model>
    <model name="college_floor2">
      <static>true</static>
      <link name="link">
        {(chr(10) + '        ').join(items)}
      </link>
    </model>
  </world>
</sdf>
"""
with open(os.path.join(PKG, 'worlds', 'college_corridor.sdf'), 'w') as fh:
    fh.write(sdf)
print('elements:', len(items), 'lights:', len(lights))

# =================================================================== 2D map
RES = 0.05
mx0, my0, mx1, my1 = -39.5, -5.0, 42.5, 31.5
W_, H_ = int(round((mx1 - mx0) / RES)), int(round((my1 - my0) / RES))
gx = mx0 + (np.arange(W_) + 0.5) * RES
gy = my0 + (np.arange(H_) + 0.5) * RES
GX, GY = np.meshgrid(gx, gy)
img = np.full((H_, W_), 205, np.uint8)
for x0, x1, y0, y1 in free:
    img[(GX >= x0) & (GX <= x1) & (GY >= y0) & (GY <= y1)] = 254
for x0, x1, y0, y1 in occ:
    img[(GX >= x0 - 1e-6) & (GX <= x1 + 1e-6) & (GY >= y0 - 1e-6) & (GY <= y1 + 1e-6)] = 0
Image.fromarray(img[::-1]).save(os.path.join(PKG, 'maps', 'college_corridor.pgm'))
with open(os.path.join(PKG, 'maps', 'college_corridor.yaml'), 'w') as fh:
    fh.write(f"""image: college_corridor.pgm
mode: trinary
resolution: {RES}
origin: [{mx0}, {my0}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
print('map', W_, 'x', H_)
