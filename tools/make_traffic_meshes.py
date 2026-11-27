#!/usr/bin/env python3
"""Make the moving-vehicle meshes for the campus traffic (COLLADA, used as Gazebo actor skins).

    pip install trimesh pycollada numpy
    python3 tools/make_traffic_meshes.py

Writes meshes/traffic_car_<colour>.dae and meshes/traffic_bus.dae (vehicle faces +x, wheels on z=0).
The walking person (meshes/walk.dae) is not generated here: it comes from Gazebo Classic's media
files (Apache License 2.0, Open Source Robotics Foundation) - see meshes/walk.dae.NOTICE.
"""
import math
import os

import numpy as np
import trimesh

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'meshes')


def coloured(m, rgb):
    m.visual = trimesh.visual.ColorVisuals(m, face_colors=np.tile(np.array(list(rgb) + [255], np.uint8),
                                                                    (len(m.faces), 1)))
    return m


def box(ex, at, rgb):
    b = trimesh.creation.box(extents=ex)
    b.apply_translation(at)
    return coloured(b, rgb)


def wheel(x, y, r, w=0.24):
    c = trimesh.creation.cylinder(radius=r, height=w, sections=20)
    c.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
    c.apply_translation([x, y, r])
    return coloured(c, (20, 20, 20))


def save(name, parts):
    data = trimesh.exchange.dae.export_collada(parts).decode()
    data = data.replace('<up_axis>Y_UP</up_axis>', '<up_axis>Z_UP</up_axis>')
    with open(os.path.join(OUT, name), 'w') as fh:
        fh.write(data)
    print(name)


def car(rgb):
    glass = (45, 55, 70)
    p = [box([4.3, 1.78, 0.72], [0, 0, 0.62], rgb),
         box([2.3, 1.6, 0.58], [-0.25, 0, 1.27], glass),
         box([2.1, 1.5, 0.06], [-0.25, 0, 1.58], rgb)]
    p += [wheel(x, y, 0.33) for x in (-1.35, 1.35) for y in (-0.8, 0.8)]
    for y in (-0.6, 0.6):
        p.append(box([0.04, 0.35, 0.14], [2.16, y, 0.75], (250, 250, 225)))
        p.append(box([0.04, 0.35, 0.14], [-2.16, y, 0.80], (200, 20, 20)))
    return p


for name, rgb in (('red', (200, 30, 30)), ('white', (235, 235, 238)), ('black', (25, 25, 28)),
                  ('blue', (40, 70, 160)), ('silver', (160, 162, 168))):
    save(f'traffic_car_{name}.dae', car(rgb))

# campus bus: 10 m, white and red with a window band
bus = [box([10.0, 2.5, 2.4], [0, 0, 1.75], (240, 240, 240)),
       box([10.02, 2.52, 0.35], [0, 0, 0.75], (190, 30, 40)),
       box([8.6, 2.54, 0.9], [-0.4, 0, 2.15], (40, 50, 65)),
       box([0.05, 2.2, 1.0], [5.01, 0, 2.1], (40, 50, 65)),
       box([10.0, 2.4, 0.12], [0, 0, 3.0], (225, 225, 228))]
bus += [wheel(x, y, 0.5, 0.3) for x in (-3.2, 3.4) for y in (-1.05, 1.05)]
for y in (-0.9, 0.9):
    bus.append(box([0.04, 0.4, 0.18], [5.02, y, 0.9], (250, 250, 225)))
save('traffic_bus.dae', bus)
