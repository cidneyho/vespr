#!/usr/bin/env python3
"""Generate the perch_branch model: a bark-textured mesh and matching collisions.

The tree layout below is the single source of truth. Running this script writes
  models/perch_branch/meshes/branch.obj   visual: tapered, slightly lumpy tubes with UVs
  models/perch_branch/model.sdf           collisions: short cylinders following the same tubes
so what the camera sees and what the drone can touch always agree.

Usage: python3 make_branch_mesh.py   (then restart the sim)
"""
import math
from pathlib import Path

import numpy as np

MODEL_DIR = Path(__file__).resolve().parent.parent / 'models' / 'perch_branch'
TEXTURES = 'model://perch_branch/materials/textures/bark_brown_02'


def elev(deg):
    """Unit direction along +x, tilted up by deg."""
    a = math.radians(deg)
    return (math.cos(a), 0.0, math.sin(a))


# Model frame: trunk at the origin, z up, the limb grows along +x.
# pieces: (direction, length) straight runs joined end to end.
# Radius tapers linearly from r0 at the start to r1 at the end.
# A twig starts on its parent at arc length `at`.
# uv_stretch: how many circumferences of length one texture tile covers.
#   Above 1 stretches the bark lengthwise, so thin limbs don't show a short repeat.
LIMB_PIECES = 10
BRANCHES = [
    dict(name='trunk', start=(0, 0, 0), pieces=[((0, 0, 1), 3.0)],
         r0=0.13, r1=0.11, sides=32, ds=0.05, collision_step=1.0, uv_stretch=1),
    # Gentle curve: 1.9 m in 10 runs, 12 deg up at the trunk easing to 2 deg at the tip
    dict(name='limb', start=(0.05, 0, 1.6),
         pieces=[(elev(12 - 10 * i / (LIMB_PIECES - 1)), 1.9 / LIMB_PIECES)
                 for i in range(LIMB_PIECES)],
         r0=0.032, r1=0.018, sides=16, ds=0.02, collision_step=0.25, uv_stretch=3),
    dict(name='twig_a', parent='limb', at=0.7, pieces=[((0.5, 0.7, 0.5), 0.4)],
         r0=0.009, r1=0.003, sides=8, ds=0.02, collision_step=0.4, uv_stretch=3),
    dict(name='twig_b', parent='limb', at=1.75, pieces=[((0.5, -0.7, 0.4), 0.35)],
         r0=0.009, r1=0.003, sides=8, ds=0.02, collision_step=0.35, uv_stretch=3),
]

# Clear stretch of the limb between twig_a and twig_b, as arc length from the trunk
PERCH_ZONE = (1.0, 1.7)

# Bark lumps as a fraction of radius. Small, so collisions stay within ~1 mm.
LUMP = 0.03


class Branch:
    def __init__(self, spec, start):
        self.spec = spec
        self.start = np.array(start, float)
        self.dirs = [np.array(d, float) / np.linalg.norm(d) for d, _ in spec['pieces']]
        self.lens = [length for _, length in spec['pieces']]
        self.length = sum(self.lens)

    def radius(self, s):
        return self.spec['r0'] + (self.spec['r1'] - self.spec['r0']) * s / self.length

    def point(self, s):
        p = self.start.copy()
        for d, length in zip(self.dirs, self.lens):
            step = min(s, length)
            p += d * step
            s -= step
            if s <= 0:
                break
        return p

    def tangent(self, s):
        """Piece direction, averaged exactly at a joint."""
        edge = 0.0
        for i, (d, length) in enumerate(zip(self.dirs, self.lens)):
            edge += length
            if abs(s - edge) < 1e-9 and i + 1 < len(self.dirs):
                t = d + self.dirs[i + 1]
                return t / np.linalg.norm(t)
            if s < edge:
                return d
        return self.dirs[-1]

    def chunks(self):
        """(start_s, end_s) runs for collision cylinders, never crossing a joint."""
        out, s0 = [], 0.0
        for length in self.lens:
            n = max(1, math.ceil(length / self.spec['collision_step'] - 1e-9))
            for i in range(n):
                out.append((s0 + length * i / n, s0 + length * (i + 1) / n))
            s0 += length
        return out


def build_branches():
    built = {}
    for spec in BRANCHES:
        if 'parent' in spec:
            start = built[spec['parent']].point(spec['at'])
        else:
            start = spec['start']
        built[spec['name']] = Branch(spec, start)
    return list(built.values())


def tube(branch, phase):
    """Rings of vertices along the branch, plus a cap at the far end."""
    sides, ds = branch.spec['sides'], branch.spec['ds']
    n_rings = max(2, math.ceil(branch.length / ds) + 1)
    s_values = np.linspace(0, branch.length, n_rings)
    # Joints are exact ring positions so the bend is sharp, not cut across
    joints = np.cumsum(branch.lens)[:-1]
    s_values = np.unique(np.concatenate([s_values, joints]))

    tile_len = branch.spec['uv_stretch'] * 2 * math.pi * (branch.spec['r0'] + branch.spec['r1']) / 2

    t0 = branch.tangent(0)
    seed = np.array([0, 0, 1.0]) if abs(t0[2]) < 0.9 else np.array([1.0, 0, 0])
    normal = np.cross(t0, np.cross(seed, t0))
    normal /= np.linalg.norm(normal)

    verts, uvs, norms, rings = [], [], [], []
    for s in s_values:
        t = branch.tangent(s)
        # Parallel transport keeps the texture seam from twisting
        normal = normal - np.dot(normal, t) * t
        normal /= np.linalg.norm(normal)
        binormal = np.cross(t, normal)
        centre = branch.point(s)
        r = branch.radius(s)
        ring = []
        for k in range(sides + 1):  # last vertex repeats the first, for the UV seam
            th = 2 * math.pi * k / sides
            radial = math.cos(th) * normal + math.sin(th) * binormal
            lump = 1 + LUMP * (math.sin(3 * th + 7 * s + phase)
                               + 0.6 * math.sin(5 * th - 11 * s + 2 * phase)) / 1.6
            ring.append(len(verts))
            verts.append(centre + radial * r * lump)
            uvs.append((k / sides, s / tile_len))
            norms.append(radial)
        rings.append(ring)

    faces = []
    for a, b in zip(rings, rings[1:]):
        for k in range(sides):
            faces.append((a[k], a[k + 1], b[k + 1]))
            faces.append((a[k], b[k + 1], b[k]))

    # End cap
    tip = len(verts)
    verts.append(branch.point(branch.length))
    uvs.append((0.5, branch.length / tile_len))
    norms.append(branch.tangent(branch.length))
    last = rings[-1]
    for k in range(sides):
        faces.append((last[k], last[k + 1], tip))

    return verts, uvs, norms, faces


def write_obj(branches, path):
    lines = ['# Generated by scripts/make_branch_mesh.py. Do not edit by hand.',
             'mtllib branch.mtl', 'usemtl bark']
    offset = 0
    for i, branch in enumerate(branches):
        verts, uvs, norms, faces = tube(branch, phase=1.7 * i)
        lines.append(f'o {branch.spec["name"]}')
        lines += [f'v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}' for v in verts]
        lines += [f'vt {u:.5f} {v:.5f}' for u, v in uvs]
        lines += [f'vn {n[0]:.5f} {n[1]:.5f} {n[2]:.5f}' for n in norms]
        for face in faces:
            idx = [f + offset + 1 for f in face]
            lines.append('f ' + ' '.join(f'{j}/{j}/{j}' for j in idx))
        offset += len(verts)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('\n'.join(lines) + '\n')
    # Only for other mesh viewers such as Blender. Gazebo uses the SDF material
    path.with_suffix('.mtl').write_text(
        '# Generated by scripts/make_branch_mesh.py. Do not edit by hand.\n'
        'newmtl bark\nKd 1 1 1\n'
        'map_Kd ../materials/textures/bark_brown_02_diff_1k.jpg\n')
    return offset


def collision_xml(branch):
    out = []
    for i, (s0, s1) in enumerate(branch.chunks()):
        a, b = branch.point(s0), branch.point(s1)
        centre, d = (a + b) / 2, (b - a) / np.linalg.norm(b - a)
        # Cylinders run along z; tilt by pitch, then turn by yaw
        pitch, yaw = math.acos(np.clip(d[2], -1, 1)), math.atan2(d[1], d[0])
        r = branch.radius((s0 + s1) / 2)
        out.append(f'''      <collision name="{branch.spec['name']}_{i}">
        <pose>{centre[0]:.4f} {centre[1]:.4f} {centre[2]:.4f} 0 {pitch:.5f} {yaw:.5f}</pose>
        <geometry>
          <cylinder><radius>{r:.4f}</radius><length>{s1 - s0:.4f}</length></cylinder>
        </geometry>
      </collision>''')
    return '\n'.join(out)


def write_sdf(branches, path):
    limb = next(b for b in branches if b.spec['name'] == 'limb')
    p0, p1 = (limb.point(s) for s in PERCH_ZONE)
    r0, r1 = (limb.radius(s) for s in PERCH_ZONE)
    # perch_point: middle of the perch zone on the limb's centreline, x along the limb
    s_perch = sum(PERCH_ZONE) / 2
    perch, (tx, ty, tz) = limb.point(s_perch), limb.tangent(s_perch)
    perch_pitch = -math.asin(tz)  # SDF pitch is positive nose down
    perch_yaw = math.atan2(ty, tx)
    collisions = '\n'.join(collision_xml(b) for b in branches)
    path.write_text(f'''<?xml version="1.0"?>
<!--
  Generated by scripts/make_branch_mesh.py. Edit the layout there and rerun it.

  Tree trunk with one limb and two twigs. Model frame: trunk at the origin, limb along +x.
  Trunk: {2 * BRANCHES[0]['r0'] * 100:.0f} cm at the base, 3 m tall.
  Limb leaves the trunk at 1.6 m and curves gently, 12 deg up easing to 2 deg,
    out to x {limb.point(limb.length)[0]:.2f}. Diameter tapers from {2 * limb.spec['r0'] * 100:.1f} cm to {2 * limb.spec['r1'] * 100:.1f} cm.
  Perch zone: limb from x {p0[0]:.2f} to {p1[0]:.2f}, diameter {2 * r0 * 100:.1f} to {2 * r1 * 100:.1f} cm,
    top at z {p0[2] + r0:.2f} to {p1[2] + r1:.2f}.
  perch_point: where to perch, on the limb's centreline at the middle of the perch zone
    (x {perch[0]:.3f}, z {perch[2]:.3f}; the top is {limb.radius(s_perch) * 100:.1f} cm above). x along the limb, z up.
  Static. The visual is meshes/branch.obj with Poly Haven's bark_brown_02 (CC0).
-->
<sdf version="1.9">
  <model name="perch_branch">
    <static>true</static>
    <link name="tree">
{collisions}
      <visual name="visual">
        <geometry>
          <mesh><uri>model://perch_branch/meshes/branch.obj</uri></mesh>
        </geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
          <pbr>
            <metal>
              <albedo_map>{TEXTURES}_diff_1k.jpg</albedo_map>
              <normal_map>{TEXTURES}_nor_gl_1k.jpg</normal_map>
              <roughness_map>{TEXTURES}_rough_1k.jpg</roughness_map>
              <metalness>0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
    <!-- No collision or visual: a marker that Gazebo reports the pose of (perch_sim mocap_sim) -->
    <link name="perch_point">
      <pose>{perch[0]:.4f} {perch[1]:.4f} {perch[2]:.4f} 0 {perch_pitch:.5f} {perch_yaw:.5f}</pose>
    </link>
  </model>
</sdf>
''')


def main():
    branches = build_branches()
    n_verts = write_obj(branches, MODEL_DIR / 'meshes' / 'branch.obj')
    write_sdf(branches, MODEL_DIR / 'model.sdf')
    n_coll = sum(len(b.chunks()) for b in branches)
    print(f'Wrote {MODEL_DIR}/meshes/branch.obj ({n_verts} vertices)')
    print(f'Wrote {MODEL_DIR}/model.sdf ({n_coll} collision cylinders)')


if __name__ == '__main__':
    main()
