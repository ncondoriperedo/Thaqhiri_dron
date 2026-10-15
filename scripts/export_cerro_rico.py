"""Export the supplied Blender mine without modifying the source .blend.

Usage: blender -b 02_Cerro_Rico_Messy.blend --python scripts/export_cerro_rico.py
"""

import hashlib
import json
from pathlib import Path

import bpy
import numpy as np
from mathutils import Vector
from mathutils.bvhtree import BVHTree


ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'src/thaiquiri_webots/webots_world/protos/cerro_rico'
DEST.mkdir(parents=True, exist_ok=True)
(DEST / 'textures').mkdir(exist_ok=True)
scene = bpy.context.scene
assert scene.unit_settings.scale_length == 1.0, 'Expected metre-scale source'
collision = next(o for o in scene.objects if o.name.startswith('SIM COLLISION'))
spawn = next(o for o in scene.objects if o.name.startswith('SPAWN'))
source_objects = list(scene.objects)

for image in bpy.data.images:
    if image.packed_file:
        image.filepath_raw = str(DEST / 'textures' / (image.name + '.png'))
        image.file_format = 'PNG'
        image.save()

# Retain practical lights, but omit the Blender inspection/camera torch.
lights = [{'name': o.name, 'position': list(o.matrix_world.translation),
           'color': list(o.data.color), 'watts': o.data.energy}
          for o in source_objects if o.type == 'LIGHT' and not o.name.startswith('Inspection')]
vertices = [collision.matrix_world @ v.co for v in collision.data.vertices]
faces = [list(p.vertices) for p in collision.data.polygons]
tree = BVHTree.FromPolygons(vertices, faces)
# Deterministic, area-weighted collision-surface reference for map scoring.
v = np.asarray(vertices, dtype=np.float32)
triangles = np.asarray([[face[0], face[i], face[i+1]]
                        for face in faces for i in range(1, len(face)-1)])
tri = v[triangles]
area = np.linalg.norm(np.cross(tri[:,1]-tri[:,0], tri[:,2]-tri[:,0]), axis=1)
rng = np.random.default_rng(42)
chosen = tri[rng.choice(len(tri), 300000, p=area/area.sum())]
a = np.sqrt(rng.random((len(chosen), 1)))
b = rng.random((len(chosen), 1))
samples = (1-a)*chosen[:,0] + a*(1-b)*chosen[:,1] + a*b*chosen[:,2]
np.savez_compressed(DEST / 'surface_reference.npz', points=np.vstack([v, samples]).astype(np.float32))
probes = []
for y in range(2, 19):
    for x in (-0.5, 0.0, 0.5, 1.0):
        origin = Vector((x, y, 1.2))
        down = tree.ray_cast(origin, Vector((0, 0, -1)), 8)[0]
        up = tree.ray_cast(origin, Vector((0, 0, 1)), 8)[0]
        probes.append({'x': x, 'y': y, 'floor': down.z if down else None,
                       'ceiling': up.z if up else None})

bpy.ops.object.select_all(action='DESELECT')
collision.hide_set(False)
collision.select_set(True)
bpy.context.view_layer.objects.active = collision
bpy.ops.wm.obj_export(filepath=str(DEST / 'collision.obj'), export_selected_objects=True,
                      forward_axis='Y', up_axis='Z', export_materials=False,
                      export_triangulated_mesh=True, export_normals=False, export_uv=False)

bpy.ops.object.select_all(action='DESELECT')
visual = [o for o in source_objects if o.type in ('MESH', 'CURVE', 'FONT')
          and not o.hide_render and o != collision]
for obj in visual:
    obj.hide_set(False)
    obj.select_set(True)
bpy.context.view_layer.objects.active = visual[0]
bpy.ops.object.convert(target='MESH')
# Reduce thousands of separate CAD draw objects while retaining material/UV slots.
bpy.ops.object.join()
bpy.ops.wm.obj_export(filepath=str(DEST / 'visual.obj'), export_selected_objects=True,
                      forward_axis='Y', up_axis='Z', export_materials=True,
                      export_triangulated_mesh=True, path_mode='RELATIVE')
# Blender's OBJ exporter gives packed images invalid relative paths. We saved
# their pixels above; make the MTL self-contained and verify every reference.
mtl = DEST / 'visual.mtl'
lines = []
for line in mtl.read_text().splitlines():
    if line.startswith('map_Kd '):
        texture = Path(line.split(maxsplit=1)[1]).name
        assert (DEST / 'textures' / texture).is_file(), texture
        line = 'map_Kd textures/' + texture
    lines.append(line)
mtl.write_text('\n'.join(lines) + '\n')

metadata = {
    'source': Path(bpy.data.filepath).name,
    'source_sha256': hashlib.sha256(Path(bpy.data.filepath).read_bytes()).hexdigest(),
    'blender_version': bpy.app.version_string,
    'metres_per_unit': 1.0, 'axes': 'Blender XYZ retained; Webots ENU Z-up',
    'visual_objects': len(visual), 'collision_object': collision.name,
    'collision_vertices': len(vertices), 'collision_faces': len(faces),
    'bounds': [[min(v[i] for v in vertices), max(v[i] for v in vertices)] for i in range(3)],
    'spawn_marker': list(spawn.matrix_world.translation), 'floor_probes': probes,
    'lighting_note': 'Practical light positions/colors retained; watts mapped to relative Webots intensity, not photometrically calibrated.',
    'lights': lights,
}
(DEST / 'manifest.json').write_text(json.dumps(metadata, indent=2) + '\n')

light_nodes = []
for light in lights:
    xyz = ' '.join(f'{v:.6g}' for v in light['position'])
    color = ' '.join(f'{v:.6g}' for v in light['color'])
    intensity = min(3.0, max(0.3, light['watts'] / 100))
    light_nodes.append(f'''    PointLight {{
      on IS lightsOn
      location {xyz}
      color {color}
      intensity {intensity:.6g}
      attenuation 0 0 0.25
      radius 12
      castShadows FALSE
    }}''')
proto = '''#VRML_SIM R2025a utf8
# Generated by scripts/export_cerro_rico.py. Coordinates are metres, Z-up.
PROTO CerroRico [
  field SFVec3f translation 0 0 0
  field SFBool lightsOn TRUE
] {
  Solid {
    translation IS translation
    name "Cerro Rico mine"
    children [
      CadShape { url [ "cerro_rico/visual.obj" ] }
LIGHTS
    ]
    # A static concave Mesh preserves navigable tunnel interiors.
    boundingObject Mesh { url [ "cerro_rico/collision.obj" ] }
  }
}
'''.replace('LIGHTS', '\n'.join(light_nodes))
(DEST.parent / 'CerroRico.proto').write_text(proto)
print(json.dumps({k: v for k, v in metadata.items() if k not in ('lights', 'floor_probes')}, indent=2))
print('Entrance floor/ceiling probes:', probes[:24])
