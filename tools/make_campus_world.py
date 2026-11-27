#!/usr/bin/env python3
"""Generate the university-campus world (modelled on the Rajpura campus, layout approximate).

Writes:
  worlds/chitkara_campus.sdf        Gazebo world
  textures/campus_*.png             building facades, signs, mural
  maps/chitkara_campus.pgm/.yaml    2D map of what the 2.1 m roof lidar sees (for AMCL)
  maps/chitkara_campus_roads.pgm/.yaml   roads-only map (for the Nav2 costmaps: keeps the SUV on the roads)

    python3 tools/make_campus_world.py

Frame: x points from the main gate up the boulevard to the Turing Block, y to the left.
The SUV spawns at (0, 0) facing +x, just inside the main gate.

Roads (8 m wide):  boulevard y=0 (gate -> Turing Block), front road x=10, back road x=130,
                   north road y=100, south road y=-100  (a loop the SUV can drive round),
                   with an open turning circle at each of the six junctions so the SUV can turn round.
Inside the loop:   woods + Galileo block (north half), big lawn with garden (south half).
Around the loop:   Turing, Babbage, Tesla blocks and parking (east), Picasso, Le Corbusier, Bloom
                   (north), Square One, Edison, Newton (south). Highway outside the gate (west).
Block names are real; their positions are an assumption.
"""
import math
import os

import numpy as np
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
TEX = os.path.join(PKG, 'textures')
os.makedirs(TEX, exist_ok=True)
BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
LIDAR_Z = 2.1
FH = 3.6               # floor height
PPM = 16               # texture pixels per metre

items, lights = [], []
obst_rect, obst_circ = [], []    # lidar-visible obstacles (map)
road_rect, car_rect = [], []     # drivable areas / parked cars (nav map)
_n = [0]
rng = np.random.default_rng(42)


# ------------------------------------------------------------------ SDF helpers
def mat(c, emissive=None, tex=None, spec=0.08):
    s = (f'<material><ambient>{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1</ambient>'
         f'<diffuse>{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} 1</diffuse><specular>{spec} {spec} {spec} 1</specular>')
    if emissive:
        s += f'<emissive>{emissive[0]} {emissive[1]} {emissive[2]} 1</emissive>'
    if tex:
        s += (f'<pbr><metal><albedo_map>model://jeep_ww/textures/{tex}</albedo_map>'
              '<roughness>0.8</roughness><metalness>0.0</metalness></metal></pbr>')
    return s + '</material>'


def box(x0, x1, y0, y1, z0, z1, col, collide=False, yaw=0.0, **kw):
    _n[0] += 1
    x0, x1 = sorted((x0, x1)); y0, y1 = sorted((y0, y1))
    pose = f'{(x0 + x1) / 2:.3f} {(y0 + y1) / 2:.3f} {(z0 + z1) / 2:.3f} 0 0 {yaw:.4f}'
    g = f'<geometry><box><size>{x1 - x0:.3f} {y1 - y0:.3f} {z1 - z0:.3f}</size></box></geometry>'
    s = f'<visual name="v{_n[0]}"><pose>{pose}</pose>{g}{mat(col, **kw)}</visual>'
    if collide:
        s += f'<collision name="c{_n[0]}"><pose>{pose}</pose>{g}</collision>'
        if z0 <= LIDAR_Z <= z1:
            obst_rect.append((x0, x1, y0, y1))
    items.append(s)


def obox(cx, cy, cz, sx, sy, sz, yaw, col, **kw):
    """Box with its own size, rotated by yaw (visual only)."""
    _n[0] += 1
    items.append(f'<visual name="v{_n[0]}"><pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 {yaw:.4f}</pose>'
                 f'<geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>{mat(col, **kw)}</visual>')


def cyl(x, y, z0, z1, r, col, collide=False, **kw):
    _n[0] += 1
    pose = f'{x:.3f} {y:.3f} {(z0 + z1) / 2:.3f} 0 0 0'
    g = f'<geometry><cylinder><radius>{r}</radius><length>{z1 - z0:.3f}</length></cylinder></geometry>'
    s = f'<visual name="v{_n[0]}"><pose>{pose}</pose>{g}{mat(col, **kw)}</visual>'
    if collide:
        s += f'<collision name="c{_n[0]}"><pose>{pose}</pose>{g}</collision>'
        if z0 <= LIDAR_Z <= z1:
            obst_circ.append((x, y, r))
    items.append(s)


def sphere(x, y, z, r, col, sx=1.0, sz=1.0):
    _n[0] += 1
    items.append(f'<visual name="v{_n[0]}"><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>'
                 f'<geometry><ellipsoid><radii>{r * sx:.3f} {r * sx:.3f} {r * sz:.3f}</radii></ellipsoid></geometry>'
                 f'{mat(col)}</visual>')


def cone(x, y, z0, h, r, col):
    """Cypress-style column: stacked tapering cylinders."""
    for k in range(4):
        rr = r * (1.0 - k * 0.22)
        cyl(x, y, z0 + k * h / 4, z0 + (k + 1) * h / 4 + 0.3, rr, col)


# ------------------------------------------------------------------ textures
def font(px):
    return ImageFont.truetype(BOLD, px)


def facade(name, width, floors, style='punched', wall=(232, 222, 200), label=None, label_col=(200, 30, 40),
           entrance=False, seed=0):
    W, Hm = width, floors * FH + 1.2
    img = Image.new('RGB', (max(8, int(W * PPM)), int(Hm * PPM)), wall)
    d = ImageDraw.Draw(img)
    r = np.random.default_rng(seed)
    glass = (70, 95, 115)
    frame = (245, 245, 240)

    def px(x, z):   # metres (x from left, z from ground) -> pixels
        return int(x * PPM), int((Hm - z) * PPM)

    d.rectangle((*px(0, Hm), *px(W, Hm - 1.2)), fill=tuple(int(c * 0.9) for c in wall))      # parapet
    for f in range(floors):
        z = f * FH
        d.line((*px(0, z + FH - 0.15), *px(W, z + FH - 0.15)), fill=tuple(int(c * 0.85) for c in wall), width=3)
        if style == 'ribbon':
            d.rectangle((*px(0.8, z + 2.6), *px(W - 0.8, z + 0.9)), fill=glass)
            for x in np.arange(0.8, W - 0.8, 1.6):
                d.line((*px(x, z + 2.6), *px(x, z + 0.9)), fill=frame, width=2)
        else:
            bay = 3.2
            n = max(1, int((W - 1) / bay))
            off = (W - n * bay) / 2
            for k in range(n):
                x0 = off + k * bay + 0.6
                lit = r.random() < 0.15
                d.rectangle((*px(x0 - 0.08, z + 2.58), *px(x0 + 2.08, z + 0.82)), fill=frame)
                d.rectangle((*px(x0, z + 2.5), *px(x0 + 2.0, z + 0.9)),
                            fill=(150, 170, 185) if lit else glass)
                d.line((*px(x0 + 1.0, z + 2.5), *px(x0 + 1.0, z + 0.9)), fill=frame, width=2)
    if entrance:
        d.rectangle((*px(W / 2 - 4, 3.2), *px(W / 2 + 4, 0)), fill=(60, 80, 95))
        for x in np.arange(W / 2 - 4, W / 2 + 4.01, 1.0):
            d.line((*px(x, 3.2), *px(x, 0)), fill=frame, width=2)
        d.rectangle((*px(W / 2 - 5, 3.6), *px(W / 2 + 5, 3.2)), fill=(90, 90, 92))
    if style == 'turing':
        # maroon centre section and a colourful abstract mural strip on the right
        cx0, cx1 = W * 0.38, W * 0.62
        d.rectangle((*px(cx0, Hm - 1.2), *px(cx1, 3.6)), fill=(140, 62, 58))
        for f in range(1, floors):
            z = f * FH
            d.rectangle((*px(cx0 + 0.8, z + 2.5), *px(cx1 - 0.8, z + 0.9)), fill=(55, 60, 70))
        mx0, mx1 = W * 0.70, W * 0.80
        cols = [(230, 60, 60), (250, 190, 40), (60, 160, 220), (90, 190, 90), (170, 80, 190), (250, 120, 40)]
        for k in range(60):
            x = r.uniform(mx0, mx1); z = r.uniform(0.5, Hm - 2.5); s = r.uniform(0.6, 2.2)
            c = cols[k % len(cols)]
            if k % 2:
                d.ellipse((*px(x - s / 2, z + s / 2), *px(x + s / 2, z - s / 2)), fill=c)
            else:
                d.rectangle((*px(x - s / 2, z + s / 3), *px(x + s / 2, z - s / 3)), fill=c)
    if label:
        f_px = int(0.85 * PPM)
        tw = d.textlength(label, font=font(f_px))
        d.text(((img.width - tw) / 2, int(0.15 * PPM)), label, fill=label_col, font=font(f_px))
    img.save(os.path.join(TEX, name))
    return Hm


def sign_tex(name, text, size, bg, fg, px_):
    img = Image.new('RGB', size, bg)
    d = ImageDraw.Draw(img)
    tw = d.textlength(text, font=font(px_))
    d.text(((size[0] - tw) / 2, (size[1] - px_ * 1.2) / 2), text, fill=fg, font=font(px_))
    img.save(os.path.join(TEX, name))


def plate(cx, cy, z0, w, h, normal, tex, t=0.03):
    """Textured panel centred on (cx, cy), facing `normal` (one of 'x+','x-','y+','y-')."""
    nx, ny = {'x+': (1, 0), 'x-': (-1, 0), 'y+': (0, 1), 'y-': (0, -1)}[normal]
    yaw = math.atan2(nx, -ny)
    obox(cx, cy, z0 + h / 2, w, t, h, yaw, (1, 1, 1), tex=tex)


# ------------------------------------------------------------------ buildings
def building(key, x0, x1, y0, y1, floors, front, label=None, style='punched', wall=(232, 222, 200), seed=0):
    Hb = floors * FH
    box(x0, x1, y0, y1, 0, Hb + 0.2, tuple(c / 255 * 0.95 for c in wall), collide=True)
    faces = {'x-': (x0 - 0.02, (y0 + y1) / 2, y1 - y0), 'x+': (x1 + 0.02, (y0 + y1) / 2, y1 - y0),
             'y-': ((x0 + x1) / 2, y0 - 0.02, x1 - x0), 'y+': ((x0 + x1) / 2, y1 + 0.02, x1 - x0)}
    for nrm, (cx, cy, w) in faces.items():
        is_front = nrm == front
        tex = f'campus_{key}_{nrm[0]}{"f" if is_front else ""}.png'
        Hm = facade(tex, w, floors, style if (is_front or style == 'ribbon') else 'punched', wall,
                    label if is_front else None, entrance=is_front, seed=seed + len(nrm) + int(is_front))
        plate(cx, cy, 0, w, Hm, nrm, tex)
    box(x0 + 0.3, x1 - 0.3, y0 + 0.3, y1 - 0.3, Hb + 0.2, Hb + 0.4, (0.55, 0.55, 0.55))       # roof
    for k in range(2):                                                                          # water tanks
        tx = x0 + (x1 - x0) * (0.25 + 0.5 * k); ty = (y0 + y1) / 2
        cyl(tx, ty, Hb + 0.4, Hb + 2.2, 0.9, (0.12, 0.12, 0.12))
    box((x0 + x1) / 2 - 3, (x0 + x1) / 2 + 3, y0 + 2, y0 + 7, Hb + 0.4, Hb + 3.4, tuple(c / 255 * 0.9 for c in wall))
    return Hb


# ------------------------------------------------------------------ trees, lights, cars
def tree(x, y, h=None, kind='round'):
    if near_junction(x, y):
        return
    h = h or rng.uniform(7, 12)
    r = rng.uniform(0.22, 0.38)
    cyl(x, y, 0, 3.4, r, (0.36, 0.26, 0.18), collide=True)
    cyl(x, y, 3.4, h * 0.6, r * 0.7, (0.36, 0.26, 0.18))
    g = (rng.uniform(0.15, 0.25), rng.uniform(0.38, 0.5), rng.uniform(0.12, 0.2))
    if kind == 'round':
        cr = h * rng.uniform(0.28, 0.36)
        sphere(x, y, h - cr * 0.8, cr, g, sz=0.85)
        sphere(x + cr * 0.4, y - cr * 0.3, h - cr * 1.1, cr * 0.7, tuple(c * 0.9 for c in g))
    else:   # tall poplar / eucalyptus look; canopy kept above 3 m so the 2.1 m lidar sees only trunks
        zc = h * 0.66
        sphere(x, y, zc, h * 0.16, g, sx=1.0, sz=(zc - 3.0) / (h * 0.16))


def cypress(x, y):
    cyl(x, y, 0, 3.0, 0.65, (0.12, 0.32, 0.14), collide=True)
    sphere(x, y, 4.6, 0.9, (0.12, 0.32, 0.14), sz=2.8)


def street_light(x, y, side):
    cyl(x, y, 0, 7.5, 0.09, (0.55, 0.57, 0.6), collide=True)
    box(x - 0.05, x + 0.05, min(y, y - side * 1.6), max(y, y - side * 1.6), 7.4, 7.5, (0.55, 0.57, 0.6))
    box(x - 0.25, x + 0.25, y - side * 1.6 - 0.2, y - side * 1.6 + 0.2, 7.3, 7.4, (1, 1, 0.9), emissive=(0.4, 0.4, 0.35))


CAR_COLS = [(0.85, 0.85, 0.87), (0.1, 0.1, 0.12), (0.7, 0.1, 0.1), (0.2, 0.3, 0.6), (0.55, 0.56, 0.58), (0.9, 0.9, 0.9)]


def car(x, y, yaw):
    c = CAR_COLS[rng.integers(0, len(CAR_COLS))]
    obox(x, y, 0.55, 4.2, 1.75, 0.75, yaw, c, spec=0.5)
    obox(x - 0.3 * math.cos(yaw), y - 0.3 * math.sin(yaw), 1.2, 2.3, 1.55, 0.6, yaw, (0.2, 0.25, 0.3), spec=0.6)
    for sx in (-1.3, 1.3):
        for sy in (-0.8, 0.8):
            px_ = x + sx * math.cos(yaw) - sy * math.sin(yaw)
            py_ = y + sx * math.sin(yaw) + sy * math.cos(yaw)
            cyl(px_, py_, 0.0, 0.6, 0.3, (0.05, 0.05, 0.05))
    hx = abs(2.1 * math.cos(yaw)) + abs(0.9 * math.sin(yaw)); hy = abs(2.1 * math.sin(yaw)) + abs(0.9 * math.cos(yaw))
    car_rect.append((x - hx, x + hx, y - hy, y + hy))


# ------------------------------------------------------------------ ground, roads, paths
GRASS = (0.36, 0.55, 0.22)
ASPHALT = (0.30, 0.30, 0.31)
PATH = (0.78, 0.72, 0.62)


def road(x0, x1, y0, y1, centre_line=True):
    box(x0, x1, y0, y1, 0.002, 0.012, ASPHALT, spec=0.15)
    road_rect.append((min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1)))
    if not centre_line:
        return
    if (x1 - x0) > (y1 - y0):
        yc = (y0 + y1) / 2
        for x in np.arange(x0 + 2, x1 - 2, 6):
            box(x, x + 3, yc - 0.07, yc + 0.07, 0.012, 0.014, (0.95, 0.95, 0.9))
        for e in (y0 + 0.25, y1 - 0.25):
            box(x0, x1, e - 0.06, e + 0.06, 0.012, 0.014, (0.92, 0.92, 0.88))
    else:
        xc = (x0 + x1) / 2
        for y in np.arange(y0 + 2, y1 - 2, 6):
            box(xc - 0.07, xc + 0.07, y, y + 3, 0.012, 0.014, (0.95, 0.95, 0.9))
        for e in (x0 + 0.25, x1 - 0.25):
            box(e - 0.06, e + 0.06, y0, y1, 0.012, 0.014, (0.92, 0.92, 0.88))


JUNCTIONS = [(10, -100), (10, 0), (10, 100), (130, -100), (130, 0), (130, 100)]
RB_R = 14.0
road_circ = []


def roundabout(cx, cy):
    """Open paved turning circle with a painted ring: the car-like SUV can go straight through
    or turn round inside it (no island, so a 5 m SUV with a 5.6 m turning radius fits)."""
    cyl(cx, cy, 0.003, 0.0125, RB_R, ASPHALT, spec=0.15)
    cyl(cx, cy, 0.0125, 0.0135, 4.3, (0.95, 0.95, 0.9))
    cyl(cx, cy, 0.0135, 0.0145, 4.0, ASPHALT, spec=0.15)
    cyl(cx, cy, 0.0145, 0.0155, 1.2, (0.95, 0.8, 0.2))
    road_circ.append((cx, cy, RB_R))


def near_junction(x, y, margin=1.5):
    return any((x - a) ** 2 + (y - b) ** 2 < (RB_R + margin) ** 2 for a, b in JUNCTIONS)


def footpath(x0, x1, y0, y1):
    box(x0, x1, y0, y1, 0.002, 0.010, PATH)


def hedge(x0, x1, y0, y1, h=1.3, col=(0.16, 0.42, 0.16)):
    box(x0, x1, y0, y1, 0, h, col, collide=True)


# =================================================================== textures for signs
sign_tex('campus_gate_sign.png', 'MAIN GATE', (1600, 160), (250, 250, 250), (30, 30, 35), 110)
sign_tex('campus_security.png', 'SECURITY', (500, 140), (200, 30, 40), (255, 255, 255), 90)
sign_tex('campus_parking.png', 'P  PARKING', (600, 200), (110, 60, 160), (255, 255, 255), 110)

# =================================================================== OUTSIDE: highway and fields
box(-140, -10, -160, 160, 0.0005, 0.001, (0.45, 0.55, 0.25))                     # fields
for k, yy in enumerate(np.arange(-160, 160, 20)):
    box(-130, -45, yy, yy + 20, 0.001, 0.0015, (0.55, 0.6, 0.28) if k % 2 else (0.42, 0.56, 0.24))
box(-38, -21, -160, 160, 0.002, 0.012, ASPHALT, spec=0.15)                       # highway (two carriageways)
box(-30, -29, -160, 160, 0.0, 0.5, (0.6, 0.6, 0.6))                              # median kerb
hedge(-29.9, -29.1, -160, 160, 1.0)
for yy in np.arange(-158, 160, 8):
    for xx in (-34.5, -24.5):
        box(xx - 0.08, xx + 0.08, yy, yy + 4, 0.012, 0.014, (0.95, 0.95, 0.9))
for yy in np.arange(-150, 150, 11):
    tree(-48 + rng.uniform(-2, 2), yy, kind='tall')
road(-21, -10, -6, 6, centre_line=False)                                         # gate apron

# =================================================================== MAIN GATE (x = -10)
GX0, GX1 = -14.5, -7.0
box(GX0, GX1, -24, 24, 5.2, 5.9, (0.97, 0.97, 0.96))                             # canopy slab
plate(GX0 - 0.03, 12.0, 5.25, 16, 0.6, 'x-', 'campus_gate_sign.png')
plate(GX0 - 0.03, -12.0, 5.25, 16, 0.6, 'x-', 'campus_gate_sign.png')
plate(GX1 + 0.03, 12.0, 5.25, 16, 0.6, 'x+', 'campus_gate_sign.png')
plate(GX1 + 0.03, -12.0, 5.25, 16, 0.6, 'x+', 'campus_gate_sign.png')
for py in (-23.4, -14.0, -6.8, 6.8, 14.0, 23.4):
    box(-11.4, -10.2, py - 0.6, py + 0.6, 0, 5.2, (0.96, 0.95, 0.93), collide=True)
box(-13.5, -8.0, -1.0, 1.0, 0, 3.0, (0.93, 0.92, 0.9), collide=True)             # security booth (central island)
box(-13.55, -7.95, -1.05, 1.05, 3.0, 5.2, (0.96, 0.95, 0.93))
plate(-7.92, 0, 3.4, 1.6, 0.45, 'x+', 'campus_security.png')
plate(-13.58, 0, 3.4, 1.6, 0.45, 'x-', 'campus_security.png')
box(-12, -8, -1.4, 1.4, 0, 0.35, (0.4, 0.62, 0.3))                                # planter
for side in (-1, 1):                                                              # closed side gates + walls
    y0, y1 = sorted((side * 7.4, side * 13.4))
    box(-10.9, -10.7, y0, y1, 0.1, 2.3, (0.25, 0.25, 0.27), collide=True)
    for yy in np.arange(y0, y1, 0.25):
        box(-10.95, -10.65, yy, yy + 0.05, 0.1, 2.3, (0.35, 0.35, 0.37))
    y0, y1 = sorted((side * 14.6, side * 22.8))
    box(-11.2, -10.4, y0, y1, 0, 2.6, (0.72, 0.6, 0.48), collide=True)
# boundary wall (2.4 m) round the campus
WC = (0.88, 0.82, 0.7)
for side in (-1, 1):
    y0, y1 = sorted((side * 24.0, side * 150))
    box(-10.6, -10.0, y0, y1, 0, 2.4, WC, collide=True)
box(-10.6, 198, 149.4, 150, 0, 2.4, WC, collide=True)
box(-10.6, 198, -150, -149.4, 0, 2.4, WC, collide=True)
box(197.4, 198, -150, 150, 0, 2.4, WC, collide=True)

# =================================================================== ROADS
road(-10, 126, -4, 4)                    # boulevard
road(6, 14, -96, 96)                     # front road
road(126, 134, -96, 96)                  # back road
road(14, 126, 96, 104)                   # north road
road(14, 126, -104, -96)                 # south road
for cx, cy in JUNCTIONS:
    roundabout(cx, cy)
road(134, 148, -20, 20, centre_line=False)    # drop-off in front of the Turing Block
# parking lot (east of the back road)
road(134, 174, -64, -30, centre_line=False)
for yy in np.arange(-62, -32, 2.8):
    for xx in (148.5, 167.5):
        box(xx - 2.6, xx + 2.6, yy - 0.05, yy + 0.05, 0.0125, 0.014, (0.95, 0.95, 0.9))
for yy in np.arange(-60.6, -32, 2.8):
    if rng.random() < 0.75:
        car(148.5, yy, 0.0)
    if rng.random() < 0.75:
        car(167.5, yy, math.pi)
plate(138.3, -30.5, 2.5, 2.0, 0.66, 'x-', 'campus_parking.png')
cyl(138.3, -30.5, 0, 2.5, 0.06, (0.4, 0.4, 0.4), collide=True)

# footpaths along the roads
footpath(-10, 126, 4, 5.2); footpath(-10, 126, -5.2, -4)
footpath(134, 140, -96, 96)

# =================================================================== BOULEVARD details
hedge(24, 118, 5.4, 6.6, 1.4)                                  # clipped hedge (north side)
hedge(24, 118, -6.4, -5.6, 0.8)                                # low hedge (south side)
for x in np.arange(24, 116, 4.0):                              # flower strip
    box(x, x + 3.6, -7.4, -6.5, 0, 0.35, [(0.9, 0.2, 0.3), (0.95, 0.75, 0.2), (0.95, 0.5, 0.15)][int(x) % 3])
for x in np.arange(62, 116, 5.0):
    cypress(x, 8.2)
for x in np.arange(20, 60, 8.0):              # avenue trees: give the roof lidar something on both sides
    tree(x, 9.5, kind='round')
for x in np.arange(22, 122, 10.0):
    tree(x, -10.5, kind='round')
for x in np.arange(26, 110, 18):
    street_light(x, 4.7, +1)
    street_light(x + 9, -4.7, -1)

for yy in np.arange(-3.5, 3.6, 1.0):                            # zebra crossing at x = 94
    box(92.5, 95.5, yy - 0.25, yy + 0.25, 0.012, 0.0145, (0.97, 0.97, 0.95))

# =================================================================== WOODS (north half of the loop) + Galileo block
building('galileo', 62, 98, 66, 88, 5, 'y-', 'GALILEO BLOCK', seed=11)
placed = []
for _ in range(20000):
    if len(placed) >= 130:
        break
    x, y = rng.uniform(20, 121), rng.uniform(12, 90)
    if 58 < x < 102 and 62 < y < 92:
        continue
    if all((x - a) ** 2 + (y - b) ** 2 > 25 for a, b in placed):
        placed.append((x, y))
        tree(x, y, kind='tall' if rng.random() < 0.45 else 'round')
footpath(58, 102, 58, 62)

# =================================================================== LAWN (south half of the loop)
box(16, 124, -94, -8, 0.003, 0.006, (0.40, 0.62, 0.25))       # mown lawn, a bit brighter
tree(70, -48, h=13, kind='round')                              # big tree in the middle
for (x, y) in [(30, -30), (44, -78), (96, -82), (110, -40), (24, -62), (86, -22), (58, -86)]:
    tree(x, y, kind='round')
# garden near the Turing end: flower beds round a small fountain
cyl(108, -22, 0, 0.6, 3.0, (0.85, 0.85, 0.85), collide=False)
cyl(108, -22, 0.6, 0.65, 2.7, (0.35, 0.55, 0.85), spec=0.8)
cyl(108, -22, 0.6, 2.6, 0.25, (0.85, 0.85, 0.85), collide=True)
for k in range(8):
    a = 2 * math.pi * k / 8
    box(108 + 5.5 * math.cos(a) - 1, 108 + 5.5 * math.cos(a) + 1, -22 + 5.5 * math.sin(a) - 1,
        -22 + 5.5 * math.sin(a) + 1, 0, 0.4, [(0.9, 0.2, 0.3), (0.95, 0.8, 0.2), (0.6, 0.3, 0.8)][k % 3])
footpath(20, 122, -51, -49)            # lawn walkways
footpath(69, 71, -92, -10)
for x, y in ((40, -52.2), (90, -52.2), (72.2, -30), (72.2, -70)):   # benches
    box(x - 1, x + 1, y - 0.3, y + 0.3, 0.4, 0.5, (0.5, 0.35, 0.2))
    box(x - 1, x + 1, y - 0.3, y - 0.25, 0.5, 0.9, (0.5, 0.35, 0.2))

# =================================================================== BUILDINGS round the loop
BEIGE = (232, 222, 200); OFFW = (240, 236, 228); SAND = (226, 210, 180); GREYW = (225, 225, 222)
building('turing', 150, 180, -26, 26, 7, 'x-', 'TURING BLOCK', style='turing', wall=OFFW, seed=1)
for k, yy in enumerate(np.arange(-5.4, 5.5, 1.8)):              # red fins on the Turing front
    box(148.6, 149.9, yy - 0.3, yy + 0.3, 3.6, 7 * FH, (0.78, 0.12, 0.14), collide=True)
box(145.5, 150, -8, 8, 3.4, 3.8, (0.85, 0.85, 0.85))           # entrance canopy
for yy in (-7.5, 7.5):
    cyl(146, yy, 0, 3.4, 0.25, (0.85, 0.85, 0.85), collide=True)
building('babbage', 150, 180, 34, 70, 6, 'x-', 'BABBAGE BLOCK', wall=BEIGE, seed=2)
building('fleming', 150, 180, 78, 108, 5, 'x-', 'FLEMING BLOCK', style='ribbon', wall=GREYW, seed=3)
building('tesla', 150, 180, -104, -72, 5, 'x-', 'TESLA BLOCK', wall=SAND, seed=4)
building('picasso', 22, 56, 118, 142, 8, 'y-', 'PICASSO BLOCK', wall=OFFW, seed=5)
building('corbusier', 64, 100, 118, 140, 5, 'y-', 'LE CORBUSIER BLOCK', style='ribbon', wall=BEIGE, seed=6)
building('bloom', 108, 138, 118, 138, 4, 'y-', 'BLOOM BLOCK', wall=SAND, seed=7)
building('squareone', 22, 62, -140, -118, 3, 'y+', 'SQUARE ONE', style='ribbon', wall=(235, 225, 205), seed=8)
building('edison', 70, 102, -142, -118, 5, 'y+', 'EDISON BLOCK', wall=BEIGE, seed=9)
building('newton', 110, 138, -140, -118, 5, 'y+', 'NEWTON BLOCK', wall=GREYW, seed=10)
# Square One food-court umbrellas
for xx in np.arange(30, 60, 6):
    cyl(xx, -114, 0, 2.4, 0.05, (0.4, 0.4, 0.4), collide=True)
    cyl(xx, -114, 2.4, 2.6, 1.4, (0.85, 0.2, 0.2) if int(xx) % 4 else (0.95, 0.75, 0.2))
# trees in the gaps between blocks and along the outer roads
for x in np.arange(18, 130, 12):
    if rng.random() < 0.8:
        tree(x + rng.uniform(-2, 2), 108.5, kind='round')
    if rng.random() < 0.8:
        tree(x + rng.uniform(-2, 2), -108.5, kind='round')
for y in np.arange(-90, 92, 10):
    tree(2.5 + rng.uniform(-0.5, 0.5), y, kind='tall') if abs(y) > 12 else None
for y in (-112, -40, 74, 112, 140):
    tree(188 + rng.uniform(-3, 3), y, kind='round')

# =================================================================== SDF
for x, y in ((40, -60), (110, 60), (150, 0), (40, 120), (160, -90), (-20, 0)):
    lights.append((x, y))
sdf = f"""<?xml version="1.0"?>
<!-- Generated by tools/make_campus_world.py - edit that script, not this file. -->
<sdf version="1.9">
  <world name="chitkara_campus">
    <physics name="4ms" type="ignored">
      <max_step_size>0.004</max_step_size>
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
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.70 0.82 0.95 1</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>0.45 0.35 -0.82</direction>
    </light>
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>600 600</size></plane></geometry>{mat(GRASS)}</visual>
      </link>
    </model>
    <model name="campus">
      <static>true</static>
      <link name="link">
        {(chr(10) + '        ').join(items)}
      </link>
    </model>
  </world>
</sdf>
"""
with open(os.path.join(PKG, 'worlds', 'chitkara_campus.sdf'), 'w') as fh:
    fh.write(sdf)
print('elements:', len(items))

# =================================================================== TRAFFIC (moving people and vehicles)
# Gazebo actors follow looping scripted trajectories. They have no collision, but the SUV's
# front bumper lidar (and cameras) see them, so they work as moving obstacles.
def fmt_wp(t, x, y, z, yaw):
    return f'<waypoint><time>{t:.2f}</time><pose>{x:.3f} {y:.3f} {z:.3f} 0 0 {yaw:.4f}</pose></waypoint>'


def fillet_loop(corners, r, step_deg=10):
    """Closed polygon (in driving order) with each corner rounded to radius r."""
    pts = []
    n = len(corners)
    for i in range(n):
        p0, p1, p2 = np.array(corners[i - 1], float), np.array(corners[i], float), np.array(corners[(i + 1) % n], float)
        d1 = (p1 - p0) / np.linalg.norm(p1 - p0)
        d2 = (p2 - p1) / np.linalg.norm(p2 - p1)
        cross = d1[0] * d2[1] - d1[1] * d2[0]
        theta = math.acos(max(-1.0, min(1.0, float(d1 @ d2))))
        t = r * math.tan(theta / 2)
        t1 = p1 - d1 * t
        nrm = np.array([-d1[1], d1[0]]) * (1 if cross > 0 else -1)
        c = t1 + nrm * r
        a0 = math.atan2(t1[1] - c[1], t1[0] - c[0])
        k = max(2, int(math.degrees(theta) / step_deg))
        for j in range(k + 1):
            a = a0 + (1 if cross > 0 else -1) * theta * j / k
            pts.append((c[0] + r * math.cos(a), c[1] + r * math.sin(a)))
    return pts


def loop_trajectory(pts, speed, z, start_frac=0.0):
    """Waypoints round a closed polyline at constant speed, starting start_frac of the way round."""
    pts = [np.array(p, float) for p in pts]
    seg = [np.linalg.norm(pts[(i + 1) % len(pts)] - pts[i]) for i in range(len(pts))]
    total = sum(seg)
    # rotate the loop so it starts start_frac of the way round
    acc, i0 = 0.0, 0
    for i, L in enumerate(seg):
        if acc + L >= start_frac * total:
            i0 = i
            break
        acc += L
    order = pts[i0:] + pts[:i0] + [pts[i0]]
    out, t = [], 0.0
    for i in range(len(order) - 1):
        a, b = order[i], order[i + 1]
        L = float(np.linalg.norm(b - a))
        if L < 1e-6:
            continue
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        out.append(fmt_wp(t + (0.05 if out else 0.0), a[0], a[1], z, yaw))
        t += L / speed
        out.append(fmt_wp(t, b[0], b[1], z, yaw))
    return out


def back_and_forth(a, b, speed, z, pause=2.0, reverse=False):
    a, b = (b, a) if reverse else (a, b)
    L = math.hypot(b[0] - a[0], b[1] - a[1])
    y1 = math.atan2(b[1] - a[1], b[0] - a[0])
    y2 = math.atan2(a[1] - b[1], a[0] - b[0])
    T = L / speed
    w = [(0, a, y1), (T, b, y1), (T + pause, b, y1), (T + pause + 1, b, y2), (2 * T + pause + 1, a, y2),
         (2 * T + 2 * pause + 1, a, y2), (2 * T + 2 * pause + 2, a, y1)]
    return [fmt_wp(t, p[0], p[1], z, yw) for t, p, yw in w]


def highway(x, y0, speed, yaw_dir):
    """Straight run along the highway, wrapping from one end to the other."""
    y_from, y_to = (-165, 165) if yaw_dir > 0 else (165, -165)
    yaw = math.pi / 2 if yaw_dir > 0 else -math.pi / 2
    t1 = abs(y_to - y0) / speed
    t2 = t1 + 0.05
    t3 = t2 + abs(y0 - y_from) / speed
    return [fmt_wp(0, x, y0, 0, yaw), fmt_wp(t1, x, y_to, 0, yaw), fmt_wp(t2, x, y_from, 0, yaw),
            fmt_wp(t3, x, y0, 0, yaw)]


actors = []


def person(name, waypoints):
    actors.append(f"""    <actor name="{name}">
      <skin><filename>model://jeep_ww/meshes/walk.dae</filename><scale>1.0</scale></skin>
      <animation name="walking"><filename>model://jeep_ww/meshes/walk.dae</filename><scale>1.000000</scale>
        <interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="walking">{''.join(waypoints)}</trajectory>
      </script>
    </actor>""")


def vehicle(name, mesh, waypoints):
    actors.append(f"""    <actor name="{name}">
      <skin><filename>model://jeep_ww/meshes/{mesh}</filename><scale>1.0</scale></skin>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="drive">{''.join(waypoints)}</trajectory>
      </script>
    </actor>""")


PZ, WALK = 1.0, 1.25
person('walker_blvd_n1', back_and_forth((26, 4.35), (110, 4.35), WALK, PZ))
person('walker_blvd_n2', back_and_forth((26, 4.35), (110, 4.35), WALK * 0.9, PZ, reverse=True))
person('walker_blvd_s1', back_and_forth((30, -4.35), (112, -4.35), WALK, PZ, reverse=True))
person('walker_crossing1', back_and_forth((93.6, -4.8), (93.6, 4.8), 1.2, PZ, pause=15.0))
person('walker_crossing2', back_and_forth((94.4, -4.8), (94.4, 4.8), 1.1, PZ, pause=18.0, reverse=True))
person('walker_lawn1', back_and_forth((24, -50), (116, -50), WALK, PZ))
person('walker_lawn2', back_and_forth((70, -90), (70, -56), 1.1, PZ, reverse=True))
person('walker_turing_crossing', back_and_forth((123, 14), (140, 14), 1.2, PZ, pause=14.0))
person('walker_north', back_and_forth((24, 110.5), (100, 110.5), WALK, PZ))
person('walker_square_one', back_and_forth((24, -111), (64, -111), 1.1, PZ, reverse=True))
person('guard_gate', back_and_forth((-8, -5.5), (-2, -5.5), 0.9, PZ, pause=4.0))

# campus traffic keeps to the left-hand lane (2.6 m left of the road centre line)
LOOP_N = fillet_loop([(12.6, 2.6), (127.4, 2.6), (127.4, 97.4), (12.6, 97.4)], 6.0)       # anticlockwise
LOOP_S = fillet_loop([(12.6, -97.4), (127.4, -97.4), (127.4, -2.6), (12.6, -2.6)], 6.0)  # anticlockwise
LOOP_O = fillet_loop([(132.6, -102.6), (7.4, -102.6), (7.4, 102.6), (132.6, 102.6)], 6.0)  # clockwise
vehicle('campus_bus', 'traffic_bus.dae', loop_trajectory(LOOP_N, 4.5, 0.01, 0.10))
vehicle('car_campus_red', 'traffic_car_red.dae', loop_trajectory(LOOP_N, 6.0, 0.01, 0.60))
vehicle('car_campus_white', 'traffic_car_white.dae', loop_trajectory(LOOP_S, 6.0, 0.01, 0.30))
vehicle('car_campus_blue', 'traffic_car_blue.dae', loop_trajectory(LOOP_S, 5.0, 0.01, 0.85))
vehicle('car_campus_black', 'traffic_car_black.dae', loop_trajectory(LOOP_O, 6.5, 0.01, 0.45))
# highway (keep left: northbound on the west carriageway)
for k, (x, y0, v, d, col) in enumerate([(-35.5, -120, 14, 1, 'silver'), (-35.5, 60, 14, 1, 'white'),
                                         (-32.5, -20, 11, 1, 'red'), (-32.5, 130, 11, 1, 'blue'),
                                         (-26.0, 100, 13, -1, 'black'), (-26.0, -70, 13, -1, 'silver'),
                                         (-23.5, 20, 10, -1, 'white'), (-23.5, -140, 10, -1, 'red')]):
    vehicle(f'car_highway_{k}', f'traffic_car_{col}.dae', highway(x, y0, v, d))

traffic_sdf = sdf.replace('<world name="chitkara_campus">', '<world name="chitkara_campus">', 1).replace(
    '  </world>', '\n'.join(actors) + '\n  </world>')
traffic_sdf = traffic_sdf.replace('<!-- Generated by tools/make_campus_world.py - edit that script, not this file. -->',
                                  '<!-- Generated by tools/make_campus_world.py - campus world WITH moving people and traffic. -->')
with open(os.path.join(PKG, 'worlds', 'chitkara_campus_traffic.sdf'), 'w') as fh:
    fh.write(traffic_sdf)
print('actors:', len(actors))

# =================================================================== maps (0.1 m)
RES = 0.1
mx0, my0, mx1, my1 = -42.0, -156.0, 202.0, 156.0
W_, H_ = int(round((mx1 - mx0) / RES)), int(round((my1 - my0) / RES))
gx = mx0 + (np.arange(W_) + 0.5) * RES
gy = my0 + (np.arange(H_) + 0.5) * RES
GX, GY = np.meshgrid(gx, gy)


def rect_mask(rs):
    m = np.zeros((H_, W_), bool)
    for x0, x1, y0, y1 in rs:
        i0, i1 = max(0, int((x0 - mx0) / RES)), min(W_, int(math.ceil((x1 - mx0) / RES)))
        j0, j1 = max(0, int((y0 - my0) / RES)), min(H_, int(math.ceil((y1 - my0) / RES)))
        m[j0:j1, i0:i1] = True
    return m


occ = rect_mask(obst_rect)
for x, y, r in obst_circ:
    i0, i1 = int((x - r - mx0) / RES), int((x + r - mx0) / RES) + 1
    j0, j1 = int((y - r - my0) / RES), int((y + r - my0) / RES) + 1
    sub = (GX[j0:j1, i0:i1] - x) ** 2 + (GY[j0:j1, i0:i1] - y) ** 2 <= (r + RES / 2) ** 2
    occ[j0:j1, i0:i1] |= sub


def save_map(name, arr):
    Image.fromarray(arr[::-1]).save(os.path.join(PKG, 'maps', name + '.pgm'))
    with open(os.path.join(PKG, 'maps', name + '.yaml'), 'w') as fh:
        fh.write(f"""image: {name}.pgm
mode: trinary
resolution: {RES}
origin: [{mx0}, {my0}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")


lid = np.full((H_, W_), 254, np.uint8)
lid[occ] = 0
save_map('chitkara_campus', lid)

def circ_mask(cs):
    m = np.zeros((H_, W_), bool)
    for x, y, r in cs:
        i0, i1 = int((x - r - mx0) / RES), int((x + r - mx0) / RES) + 1
        j0, j1 = int((y - r - my0) / RES), int((y + r - my0) / RES) + 1
        m[j0:j1, i0:i1] |= (GX[j0:j1, i0:i1] - x) ** 2 + (GY[j0:j1, i0:i1] - y) ** 2 <= r ** 2
    return m


drive = (rect_mask(road_rect) | circ_mask(road_circ)) & ~rect_mask(car_rect) & ~occ
nav = np.full((H_, W_), 0, np.uint8)
nav[drive] = 254
nav[GX < -10.6] = 0          # stay inside the campus
save_map('chitkara_campus_roads', nav)
print('maps', W_, 'x', H_)
