"""
Rig and decimate the coloured shuttle FBX for CARLA 0.9.15 / Unreal 4.26 (step 3).

Run (Blender 3.6 headless, CPU only):
  blender -b --factory-startup --python 03_rig_shuttle.py -- [--src FBX] [--skeleton FBX]
      [--out DIR]

Steps: import the colour v2 FBX (already +X forward, +Z up, metres, ground z=0) -> bake world
transforms, drop the empties -> decimate (wheels hard, big body parts 0.5, decal/light parts
untouched) -> group the 8 wheel objects into 4 corners -> vertex groups named after CARLA's
bones -> join into ONE mesh 'nift_shuttle' -> import CARLA's VehicleSkeleton (bones
Vehicle_Base, Wheel_Front_Left/Right, Wheel_Rear_Left/Right) and translate each wheel bone
(head AND tail, so orientation, roll and length stay verbatim) to the measured wheel centre
-> bind -> export
  SK_nift_shuttle.fbx         skeletal mesh (X forward, Z up; CARLA guide settings)
  SK_nift_shuttle_defaultaxes.fbx  same, Blender default axes (fallback if Unreal shows
                              it rotated)
  SMC_nift_shuttle.fbx        physics asset: base mesh 'nift_shuttle' + UCX_nift_shuttle_01
                              (convex hull, no wheels)
  SM_sc_nift_shuttle.fbx      raycast mesh: body ~8k tris + 16-sided wheel cylinders
  SM_nift_shuttle_lights.fbx  light mesh for CARLA's light system: the lens strips' outer
                              faces, one copy per light function, UVs in that function's
                              cell of CARLA's light atlas (see LIGHT_CELLS)
  nift_shuttle_rigged.blend, textures/, report.md, shots/*.png

The input FBX comes from the shuttle's CAD model, which is not part of this repository.
"""
import collections
import json
import math
import os
import shutil
import sys
import time

import bmesh
import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
argv = sys.argv[sys.argv.index('--') + 1:] if '--' in sys.argv else []


def arg(name, default):
    return argv[argv.index(name) + 1] if name in argv else default


SRC = arg('--src', os.path.join(HERE, 'nift_shuttle_coloured.fbx'))
SKEL = arg(
    '--skeleton',
    os.path.join(
        os.environ.get('CARLA_BUILD_DIR', os.path.expanduser('~/carla_build')),
        'tools/skeleton/VehicleSkeletonBinary.fbx',
    ),
)
OUT = arg('--out', os.path.join(HERE, 'output'))
NAME = 'nift_shuttle'
RATIO_RIM, RATIO_TYRE, RATIO_BODY, BODY_MIN_TRIS = 0.06, 0.15, 0.5, 600
STEER_DEG, MASS_KG, COG_HEIGHT_M = 15.0, 500.0, 0.658   # owner: nift_common/vehicle.yaml (copy: Blender has no ROS); report only  # noqa: E501
os.makedirs(os.path.join(OUT, 'shots'), exist_ok=True)
T0 = time.time()
def log(*a): print('[rig %5.0fs]' % (time.time() - T0), *a, flush=True)


def tris(me): return sum(len(p.vertices) - 2 for p in me.polygons)


def wbbox(objs):
    bpy.context.view_layer.update()        # bound_box is cached until the depsgraph re-evaluates
    pts = [o.matrix_world @ Vector(c) for o in objs for c in o.bound_box]
    return Vector(map(min, *pts)), Vector(map(max, *pts))


def centre(o):
    mn, mx = wbbox([o])
    return (mn + mx) / 2


def select_only(objs, active=None):
    bpy.ops.object.select_all(action='DESELECT')
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = active or objs[0]


# ---- 1. import, bake transforms, drop empties
bpy.ops.wm.read_homefile(use_empty=True)
bpy.ops.import_scene.fbx(filepath=SRC)
meshes = [o for o in bpy.data.objects if o.type == 'MESH']
before = {
    'objects': len(meshes),
    'tris': sum(tris(o.data) for o in meshes),
    'verts': sum(len(o.data.vertices) for o in meshes),
}
seen = set()
for o in meshes:
    if o.data.name in seen:
        o.data = o.data.copy()
    seen.add(o.data.name)
for o in meshes:
    mw = o.matrix_world.copy()
    o.data.transform(mw)
    o.parent = None
    o.matrix_world = Matrix.Identity(4)
for o in [o for o in bpy.data.objects if o.type != 'MESH']:
    bpy.data.objects.remove(o)
for m in bpy.data.materials:                       # viewport colours for the Workbench screenshots
    if m.use_nodes:
        bsdf = next((n for n in m.node_tree.nodes if n.type == 'BSDF_PRINCIPLED'), None)
        if bsdf:
            c = list(bsdf.inputs['Base Color'].default_value)
            if m.name.startswith('Glass'):
                c[3] = 0.35
                m.blend_method = 'BLEND'
            m.diffuse_color = c
            m.roughness = bsdf.inputs['Roughness'].default_value
            m.metallic = bsdf.inputs['Metallic'].default_value
mn, mx = wbbox(meshes)
log('imported', before, 'bbox', [round(v, 3) for v in mn], [round(v, 3) for v in mx])

# ---- 2. wheels: group the 8 objects into corners
wheel_objs = [o for o in meshes if o.name.split('.')[0].startswith(('Tire', 'Wheel'))]
corners = collections.defaultdict(list)
for o in wheel_objs:
    c = centre(o)
    key = ('Front' if c.x > 0 else 'Rear') + '_' + ('Left' if c.y > 0 else 'Right')
    corners['Wheel_' + key].append(o)
assert sorted(corners) == [
    'Wheel_Front_Left',
    'Wheel_Front_Right',
    'Wheel_Rear_Left',
    'Wheel_Rear_Right',
], corners
wheel_centres = {}
for bone, objs in corners.items():
    a, b = wbbox(objs)
    c = (a + b) / 2
    size = b - a
    wheel_centres[bone] = {
        'centre': [round(v, 4) for v in c],
        'diameter': round(size.z, 4),
        'width': round(size.y, 4),
        'objects': [o.name for o in objs],
    }
log('wheels', json.dumps(wheel_centres))
body_objs = [o for o in meshes if o not in wheel_objs]

# ---- 3. decimate


def decimate(o, ratio):
    select_only([o])
    m = o.modifiers.new('dec', 'DECIMATE')
    m.ratio = ratio
    m.use_collapse_triangulate = True
    bpy.ops.object.modifier_apply(modifier=m.name)


def has_mat(o, prefix):
    return any(s.material and s.material.name.startswith(prefix) for s in o.material_slots)


kept_untouched = []
for o in wheel_objs:
    decimate(o, RATIO_RIM if has_mat(o, 'Wheel_Rim') else RATIO_TYRE)
for o in body_objs:
    if has_mat(o, 'Decal') or has_mat(o, 'Light') or tris(o.data) < BODY_MIN_TRIS:
        kept_untouched.append(o.name)
        continue
    decimate(o, RATIO_BODY)
after_dec = {
    'tris': sum(tris(o.data) for o in meshes),
    'wheel_tris': sum(tris(o.data) for o in wheel_objs),
    'body_tris': sum(tris(o.data) for o in body_objs),
}
log('decimated', after_dec, 'untouched', len(kept_untouched))

# ---- 3b. light mesh for CARLA's light system
# CARLA's BaseVehiclePawn sets the light parameters (Position, Low Beam, Brake, Left Blinker,
# ...) on the material of every component tagged "emissive"; M_VehicleLights_Master then picks a
# face's light function from where its UVs fall in an 8-cell atlas
# (Content/Carla/Static/GenericMaterials/Vehicles/Lights/LightColorsCoord: brake / position top
# left, left / right blinker top right, fog / reverse bottom left, high / low beam bottom right;
# the position and fog cells are split red (back) / white (front)). Each lens strip's outer face
# is copied once per function it serves, pushed a few mm outward (translucent, so no depth fight
# with the lens) and all its UVs collapsed to the cell centre.
LIGHT_CELLS = {
    'brake': (0.167, 0.167),
    'position_back': (0.417, 0.25),
    'position_front': (0.25, 0.417),
    'left_blink': (0.833, 0.167),
    'right_blink': (0.667, 0.333),
    'high_beam': (0.667, 0.667),
    'low_beam': (0.833, 0.833),
}
LIGHT_FUNCS = {
    'Light_Tail_Red': ['position_back', 'brake'],
    'Light_Front_White': ['position_front', 'low_beam', 'high_beam'],
    # blink -> left / right by side (+Y = left)
    'Light_Indicator_Amber': ['blink'],
}
LIGHT_OFFSET_M, LIGHT_STEP_M = 0.004, 0.003
# the short lower strips at the four corners end at z ~0.50
LOWER_STRIP_TOP_M = 0.55
lights_bm = bmesh.new()
light_uv = lights_bm.loops.layers.uv.new('UVMap')
light_report = collections.Counter()
for o in body_objs:
    mats = [s.material.name if s.material else '' for s in o.material_slots]
    kind = next((m for m in mats if m in LIGHT_FUNCS), None)
    if not kind:
        continue
    lo, hi = wbbox([o])
    size = hi - lo
    c = (lo + hi) / 2
    # the strip's thin direction is the one it faces
    axis = 0 if size.x < size.y else 1
    sign = 1.0 if c[axis] > 0 else -1.0
    out = Vector((sign, 0, 0)) if axis == 0 else Vector((0, sign, 0))
    side_blink = 'left_blink' if c.y > 0 else 'right_blink'
    funcs = [side_blink] if LIGHT_FUNCS[kind] == ['blink'] else list(LIGHT_FUNCS[kind])
    if kind != 'Light_Indicator_Amber' and c.z < LOWER_STRIP_TOP_M:
        # the CAD's amber strips sit under the side overhang, out of sight: blink the lower corner
        # strips too
        funcs.append(side_blink)
    me = o.data
    faces = [p for p in me.polygons if mats[p.material_index] == kind and p.normal.dot(out) > 0.5]
    for k, func in enumerate(funcs):
        d = out * (LIGHT_OFFSET_M + k * LIGHT_STEP_M)
        u, v = LIGHT_CELLS[func]
        for p in faces:
            vs = [lights_bm.verts.new(me.vertices[i].co + d) for i in p.vertices]
            f = lights_bm.faces.new(vs)
            for loop in f.loops:
                loop[light_uv].uv = (u, 1.0 - v)     # Blender v up, Unreal v down
            light_report[func] += 1
lights = None
if light_report:
    me = bpy.data.meshes.new(NAME + '_lights')
    lights_bm.normal_update()
    lights_bm.to_mesh(me)
    lights_bm.free()
    lights = bpy.data.objects.new(NAME + '_lights', me)
    bpy.context.scene.collection.objects.link(lights)
    lights.data.materials.append(bpy.data.materials.new('M_Lights_NiftShuttle'))
log('light mesh faces per function', dict(light_report))

# ---- 4. vertex groups + join into one mesh
for o in meshes:                       # same UV layer set everywhere so the join keeps DecalUV
    names = [l.name for l in o.data.uv_layers]  # noqa: E741
    if 'DecalUV' not in names:
        o.data.uv_layers.new(name='DecalUV')


def weight_all(o, group):
    vg = o.vertex_groups.new(name=group)
    vg.add(list(range(len(o.data.vertices))), 1.0, 'REPLACE')


for bone, objs in corners.items():
    for o in objs:
        weight_all(o, bone)
for o in body_objs:
    weight_all(o, 'Vehicle_Base')
active = next(
    o
    for o in body_objs
    if [l.name for l in o.data.uv_layers] == ['Texture UVs', 'DecalUV']  # noqa: E741
)
select_only(meshes, active)
bpy.ops.object.join()
body = bpy.context.view_layer.objects.active
body.name = NAME
body.data.name = NAME
log(
    'joined',
    NAME,
    'tris',
    tris(body.data),
    'slots',
    len(body.material_slots),
    'vgroups',
    [g.name for g in body.vertex_groups],
    'uv',
    [l.name for l in body.data.uv_layers],  # noqa: E741
)

# ---- 5. CARLA skeleton: translate wheel bones, bind
bpy.ops.import_scene.fbx(filepath=SKEL)
arm = next(o for o in bpy.data.objects if o.type == 'ARMATURE')
# exactly "Armature": Unreal then drops the armature node instead of making it a root bone
arm.name = 'Armature'
inv = arm.matrix_world.inverted()
bpy.ops.object.select_all(action='DESELECT')
arm.select_set(True)
bpy.context.view_layer.objects.active = arm
bpy.ops.object.mode_set(mode='EDIT')
bone_report = {}
for eb in arm.data.edit_bones:
    if eb.name in wheel_centres:
        target = inv @ Vector(wheel_centres[eb.name]['centre'])
        d = target - eb.head
        eb.head += d
        eb.tail += d
    bone_report[eb.name] = {
        'parent': eb.parent.name if eb.parent else None,
        'head_world': [round(v, 4) for v in (arm.matrix_world @ eb.head)],
        'axis_world': [
            round(v, 3) for v in (arm.matrix_world.to_3x3() @ (eb.tail - eb.head)).normalized()
        ],
    }
bpy.ops.object.mode_set(mode='OBJECT')
assert [b.name for b in arm.data.bones] == [
    'Vehicle_Base',
    'Wheel_Front_Left',
    'Wheel_Front_Right',
    'Wheel_Rear_Right',
    'Wheel_Rear_Left',
], [b.name for b in arm.data.bones]
body.parent = arm
body.matrix_parent_inverse = arm.matrix_world.inverted()
mod = body.modifiers.new('Armature', 'ARMATURE')
mod.object = arm
log('bones', json.dumps(bone_report))

# ---- 6. collision (SMC) + raycast (SM_sc) meshes


def new_obj(name, bm):
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    o = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(o)
    return o


body_only_verts = [
    body.data.vertices[i].co.copy()
    for i in range(len(body.data.vertices))
    if any(
        g.group == body.vertex_groups['Vehicle_Base'].index for g in body.data.vertices[i].groups
    )
]
bm = bmesh.new()
for co in body_only_verts[::3]:
    bm.verts.new(co)       # every 3rd vertex is plenty for a hull
bm.verts.ensure_lookup_table()
res = bmesh.ops.convex_hull(bm, input=bm.verts)
bmesh.ops.delete(
    bm,
    geom=[
        v
        for v in bm.verts
        if v not in set(g for g in res['geom'] if isinstance(g, bmesh.types.BMVert))
    ],
    context='VERTS',
)
hull = new_obj('UCX_%s_01' % NAME, bm)
decimate(hull, 0.3) if tris(hull.data) > 120 else None
smc_base = hull.copy()
smc_base.data = hull.data.copy()
smc_base.name = NAME + '_smc_base'
smc_base.data.name = NAME + '_smc_base'
bpy.context.scene.collection.objects.link(smc_base)
log('UCX hull tris', tris(hull.data))
# raycast: body copy decimated to ~8k + 16-sided wheel cylinders
sc = body.copy()
sc.data = body.data.copy()
sc.name = 'SM_sc_%s' % NAME
sc.data.name = sc.name
sc.parent = None
sc.matrix_world = Matrix.Identity(4)
sc.modifiers.clear()
sc.vertex_groups.clear()
bpy.context.scene.collection.objects.link(sc)
# drop the wheel faces from the raycast body (they become cylinders)
bm = bmesh.new()
bm.from_mesh(sc.data)
# faces whose verts were wheels: we lost groups on the copy, so cut by position instead (inside a
# wheel's cylinder box)
boxes = [
    (Vector(w['centre']), w['diameter'] / 2 + 0.01, w['width'] / 2 + 0.01)
    for w in wheel_centres.values()
]


def in_wheel(co):
    return any(
        abs(co.y - c.y) <= hw and (co.x - c.x) ** 2 + (co.z - c.z) ** 2 <= r * r
        for c, r, hw in boxes
    )


bmesh.ops.delete(
    bm, geom=[f for f in bm.faces if all(in_wheel(v.co) for v in f.verts)], context='FACES'
)
for w in wheel_centres.values():
    c = Vector(w['centre'])
    r = w['diameter'] / 2
    hw = w['width'] / 2
    ret = bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        segments=16,
        radius1=r,
        radius2=r,
        depth=2 * hw,
        matrix=Matrix.Translation(c) @ Matrix.Rotation(math.radians(90), 4, 'X'),
    )
bm.to_mesh(sc.data)
bm.free()
for s in list(sc.material_slots):
    pass
decimate(sc, min(1.0, 8000.0 / max(1, tris(sc.data))))
log('SM_sc tris', tris(sc.data))

# ---- 7. exports
fbx_common = dict(
    use_selection=True,
    add_leaf_bones=False,
    bake_anim=False,
    mesh_smooth_type='FACE',
    use_mesh_modifiers=True,
    path_mode='RELATIVE',
    embed_textures=False,
    apply_unit_scale=True,
    apply_scale_options='FBX_SCALE_NONE',
    use_armature_deform_only=True,
)
select_only([arm, body], body)
bpy.ops.export_scene.fbx(
    filepath=os.path.join(OUT, 'SK_%s.fbx' % NAME),
    object_types={'ARMATURE', 'MESH'},
    axis_forward='X',
    axis_up='Z',
    **fbx_common,
)
bpy.ops.export_scene.fbx(
    filepath=os.path.join(OUT, 'SK_%s_defaultaxes.fbx' % NAME),
    object_types={'ARMATURE', 'MESH'},
    **fbx_common,
)
smc_base.name = NAME + '__tmp'
body.name = NAME + '__body'
smc_base.name = NAME      # base mesh must carry the vehicle name
select_only([smc_base, hull], smc_base)
bpy.ops.export_scene.fbx(
    filepath=os.path.join(OUT, 'SMC_%s.fbx' % NAME),
    object_types={'MESH'},
    axis_forward='X',
    axis_up='Z',
    **fbx_common,
)
smc_base.name = NAME + '_smc_base'
body.name = NAME
select_only([sc], sc)
bpy.ops.export_scene.fbx(
    filepath=os.path.join(OUT, 'SM_sc_%s.fbx' % NAME),
    object_types={'MESH'},
    axis_forward='X',
    axis_up='Z',
    **fbx_common,
)
if lights:
    select_only([lights], lights)
    bpy.ops.export_scene.fbx(
        filepath=os.path.join(OUT, 'SM_%s_lights.fbx' % NAME),
        object_types={'MESH'},
        axis_forward='X',
        axis_up='Z',
        **fbx_common,
    )
tex_src = os.path.join(os.path.dirname(SRC), 'textures')
if os.path.isdir(tex_src):
    shutil.copytree(tex_src, os.path.join(OUT, 'textures'), dirs_exist_ok=True)
for img in bpy.data.images:
    if img.filepath:
        img.filepath = '//textures/' + os.path.basename(img.filepath)
log('exported')

# ---- 8. screenshots (Workbench, else Cycles CPU)
for o in (hull, smc_base, sc) + ((lights,) if lights else ()):
    o.hide_render = True
bone_vis = []
for eb in arm.data.bones:
    h = arm.matrix_world @ eb.head_local
    t = arm.matrix_world @ eb.tail_local
    bm = bmesh.new()
    d = t - h
    bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        segments=8,
        radius1=0.03,
        radius2=0.005,
        depth=d.length,
        matrix=Matrix.Translation((h + t) / 2) @ d.to_track_quat('Z', 'Y').to_matrix().to_4x4(),
    )
    bmesh.ops.create_icosphere(bm, subdivisions=1, radius=0.06, matrix=Matrix.Translation(h))
    o = new_obj('bonevis_' + eb.name, bm)
    bone_vis.append(o)
    m = bpy.data.materials.new('bonevis')
    m.diffuse_color = (1, 0.1, 0.1, 1)
    o.data.materials.append(m)
scene = bpy.context.scene
cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam'))
scene.collection.objects.link(cam)
scene.camera = cam
cam.data.type = 'ORTHO'
scene.render.resolution_x, scene.render.resolution_y = 1600, 1000
scene.render.image_settings.file_format = 'PNG'
mid = Vector((0, 0, 0.95))
views = {
    'front': ((8, 0, 0.95), 3.6),
    'rear': ((-8, 0, 0.95), 3.6),
    'left': ((0, 8, 0.95), 3.6),
    'right': ((0, -8, 0.95), 3.6),
    'top': ((0, 0, 9), 3.6),
    'threequarter': ((6, -5, 4), 4.2),
}


def shoot(tag, engine):
    scene.render.engine = engine
    if engine == 'BLENDER_WORKBENCH':
        sh = scene.display.shading
        sh.light = 'STUDIO'
        sh.color_type = 'MATERIAL'
        sh.show_cavity = True
    else:
        scene.cycles.samples = 16
        scene.cycles.device = 'CPU'
    for name, (pos, scale) in views.items():
        cam.location = Vector(pos)
        cam.data.ortho_scale = scale
        cam.rotation_euler = (mid - cam.location).to_track_quat('-Z', 'Y').to_euler()
        scene.render.filepath = os.path.join(OUT, 'shots', '%s_%s.png' % (tag, name))
        bpy.ops.render.render(write_still=True)


try:
    for o in bone_vis:
        o.hide_render = True
    shoot('mesh', 'BLENDER_WORKBENCH')
    for o in bone_vis:
        o.hide_render = False
    body.show_wire = True
    body.display_type = 'WIRE'
    scene.display.shading.show_xray = True
    scene.display.shading.xray_alpha = 0.3
    shoot('bones', 'BLENDER_WORKBENCH')
    body.display_type = 'TEXTURED'
    scene.display.shading.show_xray = False
    shots_engine = 'workbench'
except Exception as e:
    log('workbench failed:', e)
    shots_engine = 'cycles'
    for o in bone_vis:
        o.hide_render = True
    shoot('mesh', 'CYCLES')
for o in bone_vis:
    bpy.data.objects.remove(o)
bpy.data.objects.remove(cam)

# ---- 9. save + report
bpy.ops.wm.save_as_mainfile(filepath=os.path.join(OUT, '%s_rigged.blend' % NAME))
fl, fr = Vector(wheel_centres['Wheel_Front_Left']['centre']), Vector(
    wheel_centres['Wheel_Front_Right']['centre']
)
rl, rr = Vector(wheel_centres['Wheel_Rear_Left']['centre']), Vector(
    wheel_centres['Wheel_Rear_Right']['centre']
)
geom = {
    'wheelbase': round(((fl + fr) / 2 - (rl + rr) / 2).length, 4),
    'track_front': round((fl - fr).length, 4),
    'track_rear': round((rl - rr).length, 4),
    'wheel_radius': round(wheel_centres['Wheel_Front_Left']['diameter'] / 2, 4),
    'wheel_width': wheel_centres['Wheel_Front_Left']['width'],
    'bbox_min': [round(v, 3) for v in mn],
    'bbox_max': [round(v, 3) for v in mx],
}
rep = {
    'source': SRC,
    'skeleton': SKEL,
    'before': before,
    'after_decimate': after_dec,
    'final_tris': tris(body.data),
    'final_verts': len(body.data.vertices),
    'material_slots': [s.material.name for s in body.material_slots],
    'vertex_groups': [g.name for g in body.vertex_groups],
    'uv_layers': [l.name for l in body.data.uv_layers],  # noqa: E741
    'wheels': wheel_centres,
    'bones': bone_report,
    'geometry': geom,
    'ucx_tris': tris(hull.data),
    'sm_sc_tris': tris(sc.data),
    'light_faces': dict(light_report),
    'untouched_objects': kept_untouched,
    'ratios': {
        'rim': RATIO_RIM,
        'tyre': RATIO_TYRE,
        'body': RATIO_BODY,
        'body_min_tris': BODY_MIN_TRIS,
    },
    'shots_engine': shots_engine,
    'seconds': round(time.time() - T0),
}
json.dump(rep, open(os.path.join(OUT, 'report.json'), 'w'), indent=1)
md = [
    '# Phase 3a — rigged shuttle (%s)' % time.strftime('%Y-%m-%d %H:%M'),
    '',
    'Source `%s`, skeleton `%s`, Blender %s, %d s.'
    % (os.path.relpath(SRC, HERE), os.path.basename(SKEL), bpy.app.version_string, rep['seconds']),
    '',
    '| | before | after |',
    '|---|---|---|',
    '| objects | %d | 1 (`nift_shuttle`) |' % before['objects'],
    '| triangles | %d | %d (wheels %d, body %d) |'
    % (before['tris'], rep['final_tris'], after_dec['wheel_tris'], after_dec['body_tris']),
    '| vertices | %d | %d |' % (before['verts'], rep['final_verts']),
    '',
    'Decimation: rims %.2f, tyres %.2f, body parts with >= %d tris %.1f; %d small / decal / light '
    'parts untouched.'
    % (RATIO_RIM, RATIO_TYRE, BODY_MIN_TRIS, RATIO_BODY, len(kept_untouched)),
    '',
    '## Geometry (from the wheel objects, metres)',
    '',
    '| wheelbase | track front | track rear | wheel radius | wheel width | bbox |',
    '|---|---|---|---|---|---|',
    '| %.4f | %.4f | %.4f | %.4f | %.4f | %s .. %s |'
    % (
        geom['wheelbase'],
        geom['track_front'],
        geom['track_rear'],
        geom['wheel_radius'],
        geom['wheel_width'],
        geom['bbox_min'],
        geom['bbox_max'],
    ),
    '',
    'Origin: ground under the mid-wheelbase point (x = 0 is also the body centre); +X forward, +Z '
    'up.',
    '',
    '## Bones (CARLA `VehicleSkeleton`, orientation / roll / length untouched, wheel bones '
    'translated)',
    '',
    '| bone | parent | head (world, m) | axis |',
    '|---|---|---|---|',
]
md += [
    '| %s | %s | %s | %s |' % (b, v['parent'], v['head_world'], v['axis_world'])
    for b, v in bone_report.items()
]
md += [
    '',
    'Vertex groups: '
    + ', '.join(rep['vertex_groups'])
    + ' (each wheel 100 %% to its bone, everything else to `Vehicle_Base`).',
    '',
    '## Material slots (%d, colour v2 names kept)' % len(rep['material_slots']),
    '',
    ', '.join('`%s`' % m for m in rep['material_slots']),
    '',
    'UV maps: `Texture UVs` (0), `DecalUV` (1: the four decal slots sample this one; set the '
    'decal textures to Clamp in Unreal).',
    '',
    '## Files',
    '',
    "- `SK_nift_shuttle.fbx` — skeletal mesh, exported X forward / Z up (CARLA guide). "
    "`SK_nift_shuttle_defaultaxes.fbx` — same with Blender's default axes; import whichever "
    "stands upright facing +X in Unreal.",
    '- `SMC_nift_shuttle.fbx` — physics asset: base mesh `nift_shuttle` + `UCX_nift_shuttle_01` '
    '(convex hull of the body without wheels, %d tris).'
    % rep['ucx_tris'],
    '- `SM_sc_nift_shuttle.fbx` — raycast mesh (%d tris: body decimated + 16-sided wheel '
    'cylinders).'
    % rep['sm_sc_tris'],
    "- `SM_nift_shuttle_lights.fbx` — light mesh for CARLA's light system (faces per function: "
    "%s); goes on a static mesh component tagged `emissive` with an `M_VehicleLights_Master` "
    "instance."
    % rep['light_faces'],
    '- `nift_shuttle_rigged.blend`, `textures/` (decal PNGs, relative paths), `report.json`, '
    '`shots/` (Workbench: `mesh_*` and `bones_*` x-ray with the bones in red).',
    '',
    '## For phase 4',
    '',
    '- Wheel blueprints: radius %.1f cm, width %.1f cm; front steer %g deg, rear 0, handbrake '
    'rear.'
    % (geom['wheel_radius'] * 100, geom['wheel_width'] * 100, STEER_DEG),
    '- Mass %g kg, CoG height %g m (nift_common/vehicle.yaml). Lights: the `Light_*` slots of the '
    'body are the lenses switched off; the light mesh glows on top of them.'
    % (MASS_KG, COG_HEIGHT_M),
    '- Glass slots -> CARLA glass material instance; `Paint_Black` -> car-paint instance; decals '
    '-> unlit/translucent with the PNGs.',
]
open(os.path.join(OUT, 'report.md'), 'w').write("\n".join(md) + "\n")
log('DONE', json.dumps(geom), 'final tris', rep['final_tris'])
