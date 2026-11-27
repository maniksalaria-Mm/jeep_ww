#!/usr/bin/env python3
"""Generate the mountain-road world for jeep_ww.

Writes:
  worlds/mountain_road.sdf          the Gazebo world
  meshes/terrain_*.stl, trees_*.stl visual-only mountain scenery
  maps/mountain_road.pgm/.yaml      ground-truth 2D map (lidar height) for Nav2

Re-run after changing the layout:
    pip install numpy trimesh pillow
    python3 tools/make_mountain_world.py

Layout (world frame = map frame = odom frame; the SUV spawns at 0,0 facing +x):
  start meadow -> ramp 1 (east) -> bend -> ramp 2 (north) -> bend -> ramp 3 (west)
  -> bend -> ramp 4 (north) -> summit plateau (16.8 m up, 40 x 34 m: room to turn around).
Each ramp is 40 m long at a 6 deg grade (10.5 %), flanked by 6 m rock walls so
the 2D lidar always has something to see and the SUV cannot drive off.
"""
import math
import os

import numpy as np
import trimesh
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, '..')
GRADE = math.radians(6.0)
RISE = 40.0 * math.tan(GRADE)          # 4.2 m per ramp
WALL_H = 6.0                           # wall height above the road
WALL_T = 1.0                           # wall thickness
BURY = 3.0                             # how far walls/fill go below the road

S = [  # name, kind, x-range, y-range, heights, ramp direction, openings
    dict(n='meadow', k='flat', x=(-7, 15), y=(-11, 11), h=0.0, open={'E': (-4, 4)}),
    dict(n='ramp1', k='ramp', d='E', x=(15, 55), y=(-4, 4), h0=0.0, h1=RISE),
    dict(n='bend1', k='flat', x=(55, 71), y=(-8, 8), h=RISE, open={'W': (-4, 4), 'N': (59, 67)}),
    dict(n='ramp2', k='ramp', d='N', x=(59, 67), y=(8, 48), h0=RISE, h1=2 * RISE),
    dict(n='bend2', k='flat', x=(55, 71), y=(48, 64), h=2 * RISE, open={'S': (59, 67), 'W': (52, 60)}),
    dict(n='ramp3', k='ramp', d='W', x=(15, 55), y=(52, 60), h0=2 * RISE, h1=3 * RISE),
    dict(n='bend3', k='flat', x=(-1, 15), y=(48, 64), h=3 * RISE, open={'E': (52, 60), 'N': (3, 11)}),
    dict(n='ramp4', k='ramp', d='N', x=(3, 11), y=(64, 104), h0=3 * RISE, h1=4 * RISE),
    dict(n='summit', k='flat', x=(-13, 27), y=(104, 138), h=4 * RISE, open={'S': (3, 11)}),
]
SUMMIT_H = 4 * RISE

ROAD = (0.42, 0.36, 0.29)
ROCK = (0.46, 0.42, 0.38)
BOULDER = (0.38, 0.36, 0.34)
GRASS = (0.33, 0.45, 0.22)

items = []          # (name, pose(x,y,z,r,p,y), size(x,y,z), colour, footprint-for-map or None)
footprints = []     # oriented rectangles occupied at lidar height: (cx, cy, yaw, sx, sy)


def rot(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def add_box(name, anchor, local_off, rpy, size, colour, occupied=True):
    """Place a box whose local point `local_off` (relative to the box centre) sits at `anchor`."""
    R = rot(*rpy)
    c = np.array(anchor, float) - R @ np.array(local_off, float)
    items.append((name, (*c, *rpy), size, colour))
    if occupied:
        footprints.append((c[0], c[1], rpy[2], size[0], size[1], R))


def ramp_frame(s):
    """Return (rpy, start point, unit direction in plan, length in plan, width, cross-axis)."""
    x0, x1 = s['x']
    y0, y1 = s['y']
    if s['d'] == 'E':
        return (0, -GRADE, 0), (x0, (y0 + y1) / 2), (1, 0), x1 - x0, y1 - y0
    if s['d'] == 'W':
        return (0, GRADE, 0), (x1, (y0 + y1) / 2), (-1, 0), x1 - x0, y1 - y0
    if s['d'] == 'N':
        return (GRADE, 0, 0), ((x0 + x1) / 2, y0), (0, 1), y1 - y0, x1 - x0
    raise ValueError


def road_height(s, x, y):
    if s['k'] == 'flat':
        return s['h']
    _, (sx, sy), (ux, uy), L, _ = ramp_frame(s)
    t = np.clip(((x - sx) * ux + (y - sy) * uy) / L, 0, 1)
    return s['h0'] + t * (s['h1'] - s['h0'])


# ---------------------------------------------------------------- road + walls
for s in S:
    if s['k'] == 'flat':
        (x0, x1), (y0, y1), h = s['x'], s['y'], s['h']
        if h > 0:
            add_box(s['n'] + '_deck', ((x0 + x1) / 2, (y0 + y1) / 2, h), (0, 0, (h + 2) / 2), (0, 0, 0),
                    (x1 - x0, y1 - y0, h + 2), ROAD, occupied=False)
        sides = {'N': ('x', (x0 - WALL_T, x1 + WALL_T), y1 + WALL_T / 2),
                 'S': ('x', (x0 - WALL_T, x1 + WALL_T), y0 - WALL_T / 2),
                 'E': ('y', (y0 - WALL_T, y1 + WALL_T), x1 + WALL_T / 2),
                 'W': ('y', (y0 - WALL_T, y1 + WALL_T), x0 - WALL_T / 2)}
        for side, (axis, (a, b), c) in sides.items():
            segs = [(a, b)]
            if side in s.get('open', {}):
                oa, ob = s['open'][side]
                segs = [(a, oa), (ob, b)]
            for i, (a0, b0) in enumerate(segs):
                if b0 - a0 < 0.05:
                    continue
                m = (a0 + b0) / 2
                hz = WALL_H + h + 2
                if axis == 'x':
                    add_box(f"{s['n']}_wall_{side}{i}", (m, c, h + WALL_H), (0, 0, hz / 2), (0, 0, 0),
                            (b0 - a0, WALL_T, hz), ROCK)
                else:
                    add_box(f"{s['n']}_wall_{side}{i}", (c, m, h + WALL_H), (0, 0, hz / 2), (0, 0, 0),
                            (WALL_T, b0 - a0, hz), ROCK)
    else:
        rpy, (sx, sy), (ux, uy), L, W = ramp_frame(s)
        slope_len = L / math.cos(GRADE)
        ext = 1.0                                      # bury 1 m into the level below
        mid = (sx + ux * L / 2, sy + uy * L / 2, (s['h0'] + s['h1']) / 2)
        T = s['h1'] + BURY
        if s['d'] in 'EW':
            size = (slope_len + ext, W, T)
            # shift the box back along the slope by ext/2 so only the low end is extended
            off_along = ext / 2
        else:
            size = (W, slope_len + ext, T)
            off_along = ext / 2
        # local axes: for E/W ramps the slope axis is local x, for N ramps local y
        if s['d'] == 'E':
            loc = (off_along, 0, T / 2)
        elif s['d'] == 'W':
            loc = (-off_along, 0, T / 2)
        else:
            loc = (0, off_along, T / 2)
        add_box(s['n'] + '_deck', mid, loc, rpy, size, ROAD, occupied=False)

        # side walls + buttresses (rock outcrops every 8 m, alternating sides)
        hz = WALL_H + BURY
        for side in (-1, 1):
            if s['d'] in 'EW':
                wl = (0, side * (W / 2 + WALL_T / 2), -(WALL_H - hz / 2))
                wsize = (slope_len + 1.0, WALL_T, hz)
            else:
                wl = (side * (W / 2 + WALL_T / 2), 0, -(WALL_H - hz / 2))
                wsize = (WALL_T, slope_len + 1.0, hz)
            add_box(f"{s['n']}_wall_{'L' if side > 0 else 'R'}", mid, wl, rpy, wsize, ROCK)
        k = 0
        for along in np.arange(-L / 2 + 6, L / 2 - 4, 8.0):
            side = 1 if k % 2 == 0 else -1
            k += 1
            px, py = mid[0] + ux * along, mid[1] + uy * along
            pz = road_height(s, px, py)
            depth = 0.7
            if s['d'] in 'EW':
                bl = (0, side * (W / 2 - depth / 2), -(WALL_H - hz / 2))
                bsize = (1.4, depth, hz)
            else:
                bl = (side * (W / 2 - depth / 2), 0, -(WALL_H - hz / 2))
                bsize = (depth, 1.4, hz)
            add_box(f"{s['n']}_outcrop{k}", (px, py, pz), bl, rpy, bsize, ROCK)

# ---------------------------------------------------------------- boulders / landmarks
rng = np.random.default_rng(7)
boulders = [  # x, y, size, on-segment
    (-4.5, -8.5, 2.2, 0), (-4.0, 8.0, 2.6, 0), (11.5, -8.5, 1.8, 0), (12.0, 8.5, 2.0, 0),
    (68.5, -5.5, 2.6, 2), (68.5, 5.0, 1.8, 2),
    (68.5, 61.0, 2.6, 4), (57.5, 61.5, 1.8, 4),
    (1.5, 50.5, 2.6, 6), (12.5, 61.5, 1.8, 6),
    (-10.5, 135.5, 2.4, 8), (24.5, 135.5, 2.4, 8), (-10.5, 106.5, 2.0, 8), (24.5, 106.5, 1.8, 8),
]
for i, (x, y, sz, si) in enumerate(boulders):
    h = S[si]['h']
    yaw = rng.uniform(0, math.pi)
    tall = rng.uniform(3.0, 4.2)        # taller than the roof lidar (2.1 m) so the SUV can see it
    add_box(f'boulder{i}', (x, y, h - 0.3), (0, 0, -tall / 2), (rng.uniform(-.08, .08), rng.uniform(-.08, .08), yaw),
            (sz, sz * rng.uniform(0.7, 1.0), tall), BOULDER)

# summit cairn / flag pole
add_box('summit_cairn', (7.0, 136.2, SUMMIT_H), (0, 0, -1.4), (0, 0, 0.3), (1.6, 1.6, 2.8), BOULDER)
add_box('summit_pole', (7.0, 136.2, SUMMIT_H + 2.8), (0, 0, -1.6), (0, 0, 0), (0.12, 0.12, 3.2), (0.2, 0.2, 0.2),
        occupied=False)
add_box('summit_flag', (7.55, 136.2, SUMMIT_H + 5.6), (0, 0, 0), (0, 0, 0), (1.0, 0.04, 0.6), (0.85, 0.1, 0.1),
        occupied=False)

# ---------------------------------------------------------------- scenery meshes (visual only)
BAND = 3.2      # keep terrain below the road within this distance of any road edge


def road_clamp(x, y, rise=0.9):
    """Upper limit for the scenery: below the road inside the corridor, then rising
    at `rise` m per m away from it (a cut valley instead of a vertical cliff)."""
    z = np.full(x.shape, np.inf)
    for s in S:
        (x0, x1), (y0, y1) = s['x'], s['y']
        px, py = np.clip(x, x0, x1), np.clip(y, y0, y1)
        dist = np.hypot(x - px, y - py)
        lim = road_height(s, px, py) - 0.4 + rise * np.maximum(dist - BAND, 0)
        z = np.minimum(z, lim)
    return z


def terrain(x, y):
    z = np.zeros_like(x)
    for cx, cy, h, r in [(35, 30, 40, 22), (-45, 150, 58, 40), (80, 165, 64, 45), (130, 60, 50, 35),
                         (-55, 55, 34, 28), (40, 95, 28, 20), (110, 115, 42, 28), (-30, -45, 18, 26),
                         (60, -45, 22, 24)]:
        z += h * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * r ** 2))
    g = np.random.default_rng(3)
    for _ in range(14):
        k = g.uniform(0.02, 0.15)
        a = g.uniform(0, 2 * np.pi)
        ph = g.uniform(0, 2 * np.pi)
        z += (0.7 / (k * 12)) * np.sin(k * (x * np.cos(a) + y * np.sin(a)) + ph)
    # gentle rise everywhere away from the start meadow so the road visibly climbs into the hills
    z += np.clip((np.hypot(x - 4, y) - 18) * 0.12, 0, 12)
    return z


STEP = 1.25
xs = np.arange(-80, 150 + STEP, STEP)
ys = np.arange(-70, 200 + STEP, STEP)
X, Y = np.meshgrid(xs, ys)
Z = terrain(X, Y)
Z = np.minimum(Z, road_clamp(X, Y))
Z = np.maximum(Z, -1.0)
ny, nx = Z.shape
V = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])
idx = np.arange(nx * ny).reshape(ny, nx)
a, b, c, d = idx[:-1, :-1].ravel(), idx[:-1, 1:].ravel(), idx[1:, :-1].ravel(), idx[1:, 1:].ravel()
F = np.vstack([np.column_stack([a, b, d]), np.column_stack([a, d, c])])
tri_z = V[F][:, :, 2]
keep = tri_z.max(axis=1) > -0.05           # drop triangles hidden under the meadow/ground
F = F[keep]
mesh = trimesh.Trimesh(V, F, process=False)
fn = mesh.face_normals
fz = V[F][:, :, 2].mean(axis=1)
noise = np.random.default_rng(11).normal(0, 2.5, len(F))
steep = fn[:, 2] < 0.80
snow = ((fz + noise > 40) & ~steep) | (fz + noise > 50)
rock = ~snow & (steep | (fz + noise > 26))
grass = ~snow & ~rock
for name, sel in (('terrain_grass.stl', grass), ('terrain_rock.stl', rock), ('terrain_snow.stl', snow)):
    m = trimesh.Trimesh(V, F[sel], process=False)
    m.remove_unreferenced_vertices()
    m.export(os.path.join(PKG, 'meshes', name))
    print(f'{name:20s} {len(m.faces)} faces')

# pine trees on gentle grassy slopes away from the road
g = np.random.default_rng(5)
trunks, crowns = [], []
cand = np.column_stack([g.uniform(-75, 145, 6000), g.uniform(-65, 195, 6000)])
cz = np.minimum(terrain(cand[:, 0], cand[:, 1]), road_clamp(cand[:, 0], cand[:, 1]))
far = road_clamp(cand[:, 0], cand[:, 1], rise=1.0) > cz + 4.0   # clearly outside the cut
ok = far & (cz > 0.5) & (cz < 26)
for (x, y), z in list(zip(cand[ok], cz[ok]))[:420]:
    hgt = g.uniform(5, 10)
    tr = trimesh.creation.cylinder(radius=0.18, height=hgt * 0.35, sections=6)
    tr.apply_translation([x, y, z + hgt * 0.175 - 0.2])
    trunks.append(tr)
    for frac, rad in ((0.25, 0.30), (0.50, 0.24)):
        cone = trimesh.creation.cone(radius=hgt * rad, height=hgt * 0.6, sections=7)
        cone.apply_translation([x, y, z + hgt * frac])
        crowns.append(cone)
trimesh.util.concatenate(trunks).export(os.path.join(PKG, 'meshes', 'trees_trunk.stl'))
trimesh.util.concatenate(crowns).export(os.path.join(PKG, 'meshes', 'trees_crown.stl'))
print('trees', len(trunks))


# ---------------------------------------------------------------- SDF
def mat(c, spec=0.05):
    return (f'<material><ambient>{c[0]} {c[1]} {c[2]} 1</ambient><diffuse>{c[0]} {c[1]} {c[2]} 1</diffuse>'
            f'<specular>{spec} {spec} {spec} 1</specular></material>')


def f(v):
    return ' '.join(f'{x:.4f}' for x in v)


link = []
for name, pose, size, col in items:
    link.append(f"""        <collision name="{name}_c"><pose>{f(pose)}</pose><geometry><box><size>{f(size)}</size></box></geometry></collision>
        <visual name="{name}_v"><pose>{f(pose)}</pose><geometry><box><size>{f(size)}</size></box></geometry>{mat(col)}</visual>""")

scenery = []
for name, col in (('terrain_grass', GRASS), ('terrain_rock', (0.45, 0.41, 0.37)), ('terrain_snow', (0.93, 0.94, 0.97)),
                  ('trees_trunk', (0.30, 0.20, 0.12)), ('trees_crown', (0.12, 0.30, 0.14))):
    scenery.append(f"""        <visual name="{name}"><geometry><mesh><uri>model://jeep_ww/meshes/{name}.stl</uri></mesh></geometry>{mat(col)}</visual>""")

sdf = f"""<?xml version="1.0"?>
<!-- Generated by tools/make_mountain_world.py - edit that script, not this file. -->
<sdf version="1.9">
  <world name="mountain_road">
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
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.62 0.76 0.92 1</background>
      <sky></sky>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.45 0.35 -0.82</direction>
    </light>

    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="c"><geometry><plane><normal>0 0 1</normal><size>800 800</size></plane></geometry></collision>
        <visual name="v"><geometry><plane><normal>0 0 1</normal><size>800 800</size></plane></geometry>{mat(GRASS)}</visual>
      </link>
    </model>

    <!-- Road decks, rock walls, outcrops and boulders (all box collisions) -->
    <model name="mountain_road">
      <static>true</static>
      <link name="link">
{chr(10).join(link)}
      </link>
    </model>

    <!-- Mountains, snow and pine trees (visual only, kept outside the road corridor) -->
    <model name="scenery">
      <static>true</static>
      <link name="link">
{chr(10).join(scenery)}
      </link>
    </model>
  </world>
</sdf>
"""
with open(os.path.join(PKG, 'worlds', 'mountain_road.sdf'), 'w') as fh:
    fh.write(sdf)
print('world items', len(items))

# ---------------------------------------------------------------- ground-truth map
RES = 0.05
mx0, my0, mx1, my1 = -18.0, -16.0, 80.0, 143.0
W_, H_ = int((mx1 - mx0) / RES), int((my1 - my0) / RES)
gx = mx0 + (np.arange(W_) + 0.5) * RES
gy = my0 + (np.arange(H_) + 0.5) * RES
GX, GY = np.meshgrid(gx, gy)
img = np.full((H_, W_), 205, np.uint8)                     # unknown
for s in S:
    (x0, x1), (y0, y1) = s['x'], s['y']
    img[(GX >= x0) & (GX <= x1) & (GY >= y0) & (GY <= y1)] = 254
for cx_, cy_, yaw, sx, sy, R in footprints:
    # project the box footprint in plan (ignore the small grade tilt)
    hx = abs(R[0, 0]) * sx / 2 + abs(R[0, 1]) * sy / 2
    hy = abs(R[1, 0]) * sx / 2 + abs(R[1, 1]) * sy / 2
    c_, s_ = math.cos(yaw), math.sin(yaw)
    lx = (GX - cx_) * c_ + (GY - cy_) * s_
    ly = -(GX - cx_) * s_ + (GY - cy_) * c_
    if abs(yaw) > 1e-6:
        sel = (np.abs(lx) <= sx / 2) & (np.abs(ly) <= sy / 2)
    else:
        sel = (np.abs(GX - cx_) <= hx) & (np.abs(GY - cy_) <= hy)
    img[sel] = 0
os.makedirs(os.path.join(PKG, 'maps'), exist_ok=True)
Image.fromarray(img[::-1]).save(os.path.join(PKG, 'maps', 'mountain_road.pgm'))
with open(os.path.join(PKG, 'maps', 'mountain_road.yaml'), 'w') as fh:
    fh.write(f"""image: mountain_road.pgm
mode: trinary
resolution: {RES}
origin: [{mx0}, {my0}, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
""")
print('map', W_, 'x', H_)
