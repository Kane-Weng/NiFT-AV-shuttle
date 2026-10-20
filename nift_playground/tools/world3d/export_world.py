"""
File: export_world.py

Description:
  The Blender step of the playground's 3D world (run by build_world.sh, headless). It reads the
  USD copy of the public digital twin (mcity/mcity-digital-twin, MIT), keeps the static world
  (terrain, buildings, props, signs, poles, lights), drops everything on the exclusion list,
  gives every surface one flat colour, strips the twin's brand name from every name and exports
  a raw .glb in our map frame. It also writes the height grid, the tree positions and an
  alignment check against our Lanelet2 map.

  blender -b --python export_world.py -- <McityMap_Main.usdc> <osm> <out_dir>

  Outputs in <out_dir>: world_raw.glb, height.json + height.i16 (raw grid), trees.json,
  export_report.json. Nothing here reads anything outside the USD folder and the .osm file.

  Frames. The USD stage is the .xodr transverse Mercator frame (x east, y north, centimetres,
  z up, absolute elevation). Our map frame is UTM 17N minus the map origin of
  nift_common/site.py. The two differ by one similarity transform (grid convergence and the UTM
  scale factor), fitted below to well under a millimetre. Heights are stored relative to z0, the
  median road height under the lanelet nodes, so the page's flat map sits near z = 0.
"""

import bpy
import json
import math
import os
import re
import sys
import time
from collections import Counter, defaultdict

import numpy as np
from mathutils import Matrix, Vector
from mathutils.bvhtree import BVHTree
from mathutils.kdtree import KDTree
from pxr import Sdf, Usd, UsdGeom, UsdShade

argv = sys.argv[sys.argv.index('--') + 1:]
USD, OSM, OUT = argv[0], argv[1], argv[2]
os.makedirs(OUT, exist_ok=True)
T0 = time.time()
report = {'excluded': [], 'warnings': []}


def log(*a):
    print(f'[world3d {time.time() - T0:6.1f}s]', *a, flush=True)


# ------------------------------------------------------------------ what goes in, what stays out

# The static world. Foliage (/Root/Foliage_Instanced) stays out: its source is not credited in the
# twin's README, so we draw our own trees at its positions instead (trees.json).
PRIM_MASK = ['/Root/Static', '/Root/Terrain', '/Root/TrafficPoles', '/Root/TrafficLights',
             '/Root/StreetLights', '/Root/TrafficLightCables', '/Root/Looks']

# Whole objects left out (matched on the clean USD-style path). Each carries a logo or a brand.
DROP_PATHS = [
    (r'/SM_ElectricalBox_s001_Mcity_\d+(/|$)', 'electrical boxes wrapped in U-M, Mcity and sponsor logos'),
    (r'/SM_McityMailBox_s001(/|$)', 'mailbox painted with the U-M Block M'),
    (r'/SM_McityRock_s001(/|$)', 'painted rock with the Mcity logo'),
    (r'/ShuttleStop_000_Sign(/|$)', 'shuttle stop sign with the Mcity logo'),
]
# Surfaces left out by material (each USD section is its own mesh with one material).
DROP_MATERIALS = [
    (r'^MI_McityRoundabout_Logo', 'U-M Block M disc on the roundabout island'),
    (r'^MI_StreetLight_s002_Banner', 'street light banners "Welcome to Mcity, University of Michigan"'),
    (r'^MI_McityTunnel_Signage', 'planet M and MDOT logos on the tunnel'),
    (r'^MI_McityGPSBlocker_Hops', 'plant cards on the GPS blocker (foliage of unclear source)'),
    (r'^M_Sky', 'sky dome'),
]
# Textures that must never decide a colour either (Unreal Starter Content masks).
STARTER_CONTENT = re.compile(r'T_MacroVariation|T_Perlin_Noise_M|StarterContent', re.I)

# The twin's brand name and other marks never appear in an exported name.
BRAND = re.compile(r'm[\s_-]?city|umich|mdot|planet[\s_-]?m|block[\s_-]?m\b', re.I)


def scrub(name):
    s = re.sub(r'(?i)m[\s_-]?city[_-]?', '', name)
    s = re.sub(r'(?i)umich|mdot|planet[\s_-]?m', '', s)
    return s or 'node'


# Flat colours set by hand where a texture average reads badly from above (linear RGB).
TERRAIN_COLOURS = [
    (r'/Road_Road_MainRoads/', (0.105, 0.105, 0.11)),
    (r'/Road_Gutter/', (0.30, 0.30, 0.29)),
    (r'/Road_Curb/', (0.42, 0.42, 0.40)),
    (r'/Road_Crosswalk_TrafficIslandCrosswalk/', (0.40, 0.40, 0.38)),
    (r'/SM_Sidewalks/', (0.42, 0.41, 0.38)),
    (r'/SM_Ground/', (0.12, 0.20, 0.06)),
    (r'/SM_McityRoundabout_Concrete/', (0.40, 0.39, 0.36)),
    (r'/SM_McityRoundabout_Logo/', (0.40, 0.39, 0.36)),
]
MARKING_WHITE = (0.75, 0.75, 0.72)
MARKING_YELLOW = (0.70, 0.45, 0.04)

# Trees: the twin's prototypes come with scale factors that make some species far too tall
# (elderberry shrubs at 28 m, oaks and ashes up to 45 m). We keep each instance's size relative
# to its species, but centre every species on a plausible height for a young planted site.
# family, typical height [m], crown width / height
SPECIES = {
    'Blue_Berry_Elder': ('shrub', 3.0, 1.0),
    'Colorado_Spruce': ('conifer', 9.0, 0.45),
    'Eastern_Hemlock': ('conifer', 6.0, 0.5),
    'Elm_Sapling': ('broadleaf', 4.5, 0.6),
    'Forsythia': ('shrub', 2.2, 1.3),
    'Grass_Trimmed_A': ('grass', 0.25, 0.0),
    'Grass_Trimmed_B': ('grass', 0.25, 0.0),
    'Hawthorn': ('broadleaf', 5.0, 0.9),
    'Holly': ('shrub', 2.0, 0.9),
    'Meadowlark': ('shrub', 1.8, 1.1),
    'Red_Ash': ('broadleaf', 12.0, 0.6),
    'Red_Maple': ('broadleaf', 10.0, 0.7),
    'Scarlet_Oak': ('broadleaf', 13.0, 0.75),
    'Shumard_Oak': ('broadleaf', 13.0, 0.75),
    'White_Ash': ('broadleaf', 12.0, 0.65),
    'White_Pine': ('conifer', 6.0, 0.5),
    'Yellow_Pine': ('conifer', 14.0, 0.4),
    'Yew': ('shrub', 1.5, 1.3),
}

# ------------------------------------------------------------------ map frame (pure numpy)

WGS_A = 6378137.0
WGS_F = 1 / 298.257223563
_n = WGS_F / (2 - WGS_F)
_e = math.sqrt(WGS_F * (2 - WGS_F))
_A = WGS_A / (1 + _n) * (1 + _n ** 2 / 4 + _n ** 4 / 64)
_al = [_n / 2 - 2 * _n ** 2 / 3 + 5 * _n ** 3 / 16, 13 * _n ** 2 / 48 - 3 * _n ** 3 / 5, 61 * _n ** 3 / 240]
TWIN_LAT0, TWIN_LON0 = 42.3005934157, -83.69928318881136   # the .xodr geoReference (+proj=tmerc)
MAP_LAT0, MAP_LON0 = 42.3005, -83.6987                     # nift_common/site.py


def kruger(lat, lon, lon0, k0):
    p = np.radians(lat)
    lam = np.radians(lon - lon0)
    t = np.sinh(np.arctanh(np.sin(p)) - _e * np.arctanh(_e * np.sin(p)))
    xi = np.arctan2(t, np.cos(lam))
    eta = np.arctanh(np.sin(lam) / np.sqrt(1 + t * t))
    E = eta + sum(_al[j] * np.cos(2 * (j + 1) * xi) * np.sinh(2 * (j + 1) * eta) for j in range(3))
    N = xi + sum(_al[j] * np.sin(2 * (j + 1) * xi) * np.cosh(2 * (j + 1) * eta) for j in range(3))
    return k0 * _A * E, k0 * _A * N


def twin_xy(lat, lon):
    x, y = kruger(lat, lon, TWIN_LON0, 1.0)
    _, y0 = kruger(np.array(TWIN_LAT0), np.array(TWIN_LON0), TWIN_LON0, 1.0)
    return x, y - y0


def map_xy(lat, lon):
    x, y = kruger(lat, lon, -81.0, 0.9996)  # UTM 17N
    x0, y0 = kruger(np.array(MAP_LAT0), np.array(MAP_LON0), -81.0, 0.9996)
    return x - x0, y - y0


def fit_similarity():
    g = np.mgrid[42.2975:42.3045:80j, -83.7020:-83.6940:80j].reshape(2, -1)
    T = np.c_[twin_xy(*g)]
    M = np.c_[map_xy(*g)]
    mt, mm = T.mean(0), M.mean(0)
    U, s, Vt = np.linalg.svd((M - mm).T @ (T - mt))
    R = U @ Vt
    sc = s.sum() / ((T - mt) ** 2).sum()
    t = mm - sc * R @ mt
    res = np.linalg.norm((sc * (R @ T.T)).T + t - M, axis=1)
    return sc, math.atan2(R[1, 0], R[0, 0]), t, float(res.max())


SC, ROT, TR, FIT_RES = fit_similarity()
log(f'twin -> map: scale {SC:.9f} rotation {math.degrees(ROT):.6f} deg translation {TR.round(4)} residual {FIT_RES * 1000:.3f} mm')


def twin_to_map(x, y):
    c, s = math.cos(ROT), math.sin(ROT)
    return SC * (c * x - s * y) + TR[0], SC * (s * x + c * y) + TR[1]


osm_txt = open(OSM).read()
ll = np.array([(float(a), float(b)) for a, b in re.findall(r"lat=['\"]([-\d.]+)['\"] lon=['\"]([-\d.]+)['\"]", osm_txt)])
NODES = np.c_[map_xy(ll[:, 0], ll[:, 1])]
log('lanelet map nodes', len(NODES))

# ------------------------------------------------------------------ USD side: materials, trees

stage = Usd.Stage.Open(USD)
MPU = UsdGeom.GetStageMetersPerUnit(stage)

BC_KEYS = ('inputs:BaseColor_Map', 'inputs:diffuse_texture', 'inputs:BaseColorMap', 'inputs:T1BaseColorMap')
TINT_KEYS = ('inputs:AlbedoTint', 'inputs:BaseColorTint', 'inputs:T1BaseColorTint', 'inputs:diffuse_tint')
usd_mats = {}
for p in stage.Traverse():
    if not p.IsA(UsdShade.Shader) or str(p.GetPath()).startswith('/Root/Foliage'):
        continue
    name = p.GetParent().GetName()
    if name in usd_mats:
        continue
    info = {'bc': None, 'tint': (1.0, 1.0, 1.0), 'glass': False, 'emissive': None, 'const': None,
            'mdl': str(p.GetAttribute('info:mdl:sourceAsset').Get().path) if p.GetAttribute('info:mdl:sourceAsset') else ''}
    for k in BC_KEYS:
        a = p.GetAttribute(k)
        v = a.Get() if a else None
        if isinstance(v, Sdf.AssetPath) and v.resolvedPath:
            info['bc'] = v.resolvedPath
            break
    for k in TINT_KEYS:
        a = p.GetAttribute(k)
        v = a.Get() if a else None
        if v is not None:
            info['tint'] = tuple(float(x) for x in list(v)[:3])
            break
    a = p.GetAttribute('inputs:OpacityMap')
    if a and a.Get() is not None:
        info['glass'] = True
    if 'FENCE' in info['mdl']:
        info['glass'] = True
    a = p.GetAttribute('inputs:emissive_color')
    if a and a.Get() is not None:
        info['emissive'] = tuple(float(x) for x in a.Get())
    a = p.GetAttribute('inputs:diffuse_color_constant')
    if a and a.Get() is not None:
        info['const'] = tuple(float(x) for x in a.Get())
    usd_mats[name] = info
log('USD materials', len(usd_mats))
for name, info in usd_mats.items():
    if info['bc'] and STARTER_CONTENT.search(info['bc']):
        report['warnings'].append(f'{name}: base colour from Starter Content, ignored')
        info['bc'] = None

# Trees: positions, species, yaw and a corrected size
veg = stage.GetPrimAtPath('/Root/Foliage_Instanced/Vegetation')
xc = UsdGeom.XformCache()
bbc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render'])
proto_h, proto_w, raw = {}, {}, defaultdict(list)
for c in veg.GetChildren():
    sp = re.sub(r'_\d+$', '', c.GetName())
    if sp not in proto_h:
        r = bbc.ComputeUntransformedBound(c).ComputeAlignedRange()
        proto_h[sp] = (r.GetMax()[2] - r.GetMin()[2]) * MPU
        proto_w[sp] = max(r.GetMax()[0] - r.GetMin()[0], r.GetMax()[1] - r.GetMin()[1]) * MPU
    m = xc.GetLocalToWorldTransform(c)
    t = m.ExtractTranslation()
    sx, sz = m.GetRow3(0).GetLength(), m.GetRow3(2).GetLength()
    xdir = m.TransformDir((1, 0, 0))
    raw[sp].append((t[0] * MPU, t[1] * MPU, math.atan2(xdir[1], xdir[0]), proto_h[sp] * sz, proto_w[sp] * sx))
trees = {'families': {}, 'species': {}}
tree_rows = []
for sp, rows in sorted(raw.items()):
    if sp not in SPECIES:
        report['warnings'].append(f'unknown tree species {sp}, drawn as broadleaf')
    fam, typ, ratio = SPECIES.get(sp, ('broadleaf', 8.0, 0.7))
    med_h = float(np.median([r[3] for r in rows]))
    hs = []
    for x, y, yaw, h, w in rows:
        mx, my = twin_to_map(x, y)
        rel = (h / med_h) ** 0.5 if med_h > 0 else 1.0
        hh = min(max(typ * rel, 0.6 * typ), 1.5 * typ)
        if fam == 'grass':
            ww = min(max(w, 1.0), 8.0)
            hh = typ
        elif fam == 'shrub':
            ww = min(max(w, 0.8 * hh), ratio * 1.6 * hh)
        else:
            ww = ratio * hh
        hs.append(hh)
        tree_rows.append((fam, sp, round(mx, 2), round(my, 2), round(yaw + ROT, 3), round(hh, 2), round(ww, 2)))
    trees['species'][sp] = {'family': fam, 'count': len(rows), 'twin_median_height_m': round(med_h, 1),
                            'height_m': [round(min(hs), 1), round(float(np.median(hs)), 1), round(max(hs), 1)]}
for fam in ('conifer', 'broadleaf', 'shrub', 'grass'):
    sel = [r for r in tree_rows if r[0] == fam]
    names = sorted({r[1] for r in sel})
    # x, y, yaw, height, width, species index: flat arrays keep the file small
    trees['families'][fam] = {'species': names, 'rows': [[r[2], r[3], r[4], r[5], r[6], names.index(r[1])] for r in sel]}
log('trees', len(tree_rows), {k: len(v['rows']) for k, v in trees['families'].items()})
del stage

# ------------------------------------------------------------------ Blender: import

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.usd_import(filepath=USD, prim_path_mask=','.join(PRIM_MASK), import_lights=False, import_cameras=False,
                      import_materials=True, import_usd_preview=False, support_scene_instancing=False,
                      apply_unit_conversion_scale=True, import_visible_only=True)
meshes = [o for o in bpy.data.objects if o.type == 'MESH']
log('imported mesh objects', len(meshes), 'materials', len(bpy.data.materials))


def clean_path(o):
    parts = []
    while o is not None:
        parts.append(re.sub(r'\.\d{3}$', '', o.name))
        o = o.parent
    return '/' + '/'.join(reversed(parts))


def base_mat(m):
    return re.sub(r'\.\d{3}$', '', m.name) if m else ''


def tris(o):
    return sum(len(p.vertices) - 2 for p in o.data.polygons)


# ------------------------------------------------------------------ exclusions

excluded = Counter()
excluded_tris = Counter()
for o in list(meshes):
    path = clean_path(o)
    mat = base_mat(o.material_slots[0].material) if o.material_slots else ''
    reason = next((r for pat, r in DROP_PATHS if re.search(pat, path)), None)
    reason = reason or next((r for pat, r in DROP_MATERIALS if re.search(pat, mat)), None)
    if reason:
        excluded[reason] += 1
        excluded_tris[reason] += tris(o)
        bpy.data.objects.remove(o, do_unlink=True)
meshes = [o for o in bpy.data.objects if o.type == 'MESH']
for reason, n in excluded.items():
    report['excluded'].append({'what': reason, 'meshes': n, 'triangles': excluded_tris[reason]})
    log('excluded', n, 'meshes', excluded_tris[reason], 'tris:', reason)
for pat, reason in DROP_PATHS + DROP_MATERIALS:
    if reason not in excluded:
        report['warnings'].append(f'exclusion rule matched nothing: {reason}')

# Traffic light cables are dense tubes: keep their line, cut most of their triangles
cable_tris_before = sum(tris(o) for o in meshes if '/Root/TrafficLightCables/' in clean_path(o))
for o in meshes:
    if '/Root/TrafficLightCables/' in clean_path(o):
        mod = o.modifiers.new('decimate', 'DECIMATE')
        mod.ratio = 0.12
log('cables', cable_tris_before, 'tris before decimation')

# ------------------------------------------------------------------ flat colours


def srgb_to_linear(c):
    return np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)


avg_cache = {}


def texture_average(path):
    if path in avg_cache:
        return avg_cache[path]
    img = bpy.data.images.load(path)
    img.scale(32, 32)
    px = np.empty(32 * 32 * 4, np.float32)
    img.pixels.foreach_get(px)
    px = px.reshape(-1, 4)
    rgb = px[:, :3] if img.is_float else srgb_to_linear(px[:, :3])
    a = px[:, 3]
    w = a if a.sum() > 1.0 else np.ones_like(a)
    c = tuple(float(v) for v in (rgb * w[:, None]).sum(0) / w.sum())
    bpy.data.images.remove(img)
    avg_cache[path] = c
    return c


def colour_for(o):
    path = clean_path(o)
    mat = base_mat(o.material_slots[0].material) if o.material_slots and o.material_slots[0].material else ''
    alpha, emissive = 1.0, None
    if '/Road_Marking/' in path:
        return (MARKING_YELLOW if 'Yellow' in mat else MARKING_WHITE), 1.0, None
    for pat, c in TERRAIN_COLOURS:
        if re.search(pat, path):
            return c, 1.0, None
    info = usd_mats.get(mat)
    if info is None:
        report['warnings'].append(f'no USD material for {mat or "(none)"} at {path}')
        return (0.35, 0.35, 0.35), 1.0, None
    if info['emissive'] is not None:  # traffic and crossing signal lamps
        e = info['emissive']
        return tuple(0.25 * v for v in e), 1.0, tuple(0.6 * v for v in e)
    if info['bc']:
        c = texture_average(info['bc'])
    elif info['const']:
        c = info['const']
    else:
        c = (0.35, 0.35, 0.35)  # e.g. Unreal's default grid material: a neutral grey, no texture
    c = tuple(min(1.0, a * b) for a, b in zip(c, info['tint']))
    if info['glass']:
        alpha = 0.35
        c = tuple(0.6 * v + 0.4 * w for v, w in zip(c, (0.55, 0.62, 0.68)))
    return c, alpha, emissive


palette = {}


def palette_material(c, alpha, emissive):
    key = tuple(round(v, 3) for v in c) + (round(alpha, 2),) + (tuple(round(v, 2) for v in emissive) if emissive else ())
    if key in palette:
        return palette[key]
    m = bpy.data.materials.new(f'flat_{len(palette):03d}')
    m.use_nodes = True
    nt = m.node_tree
    bsdf = next(n for n in nt.nodes if n.type == 'BSDF_PRINCIPLED')
    bsdf.inputs['Base Color'].default_value = (*c, 1.0)
    bsdf.inputs['Roughness'].default_value = 0.85
    bsdf.inputs['Metallic'].default_value = 0.0
    if alpha < 1:
        bsdf.inputs['Alpha'].default_value = alpha
        m.surface_render_method = 'BLENDED'
    if emissive:
        bsdf.inputs['Emission Color'].default_value = (*emissive, 1.0)
        bsdf.inputs['Emission Strength'].default_value = 1.0
    palette[key] = m
    return m


for o in meshes:
    c, a, e = colour_for(o)
    m = palette_material(c, a, e)
    o.data.materials.clear()
    o.data.materials.append(m)
    for p in o.data.polygons:
        p.material_index = 0
    # No textures ship, so no UVs or vertex colours either
    for uv in list(o.data.uv_layers):
        o.data.uv_layers.remove(uv)
    for ca in list(o.data.color_attributes):
        o.data.color_attributes.remove(ca)
for m in list(bpy.data.materials):
    if m.users == 0:
        bpy.data.materials.remove(m)
for img in list(bpy.data.images):
    bpy.data.images.remove(img)
log('palette materials', len(palette))

# ------------------------------------------------------------------ names and the map-frame root

for coll in (bpy.data.objects, bpy.data.meshes, bpy.data.materials):
    for idb in coll:
        idb.name = scrub(idb.name)
leftover = [o.name for o in bpy.data.objects if BRAND.search(o.name)] + [m.name for m in bpy.data.meshes if BRAND.search(m.name)]
if leftover:
    raise SystemExit(f'brand names left after scrubbing: {leftover[:10]}')

# ------------------------------------------------------------------ height grid (in the twin frame first)

TERRAIN_CLASSES = ('/Root/Terrain/',)
ROAD_CLASSES = ('/Road_Road_MainRoads/', '/Road_Crosswalk_TrafficIslandCrosswalk/', '/Road_Gutter/', '/Road_Marking/')
dg = bpy.context.evaluated_depsgraph_get()


def world_tris(objs):
    verts, polys = [], []
    for o in objs:
        me = o.evaluated_get(dg).to_mesh()
        me.calc_loop_triangles()
        mw = o.matrix_world
        base = len(verts)
        verts.extend(mw @ v.co for v in me.vertices)
        polys.extend(tuple(base + i for i in t.vertices) for t in me.loop_triangles)
        o.evaluated_get(dg).to_mesh_clear()
    return verts, polys


terrain_objs = [o for o in meshes if any(k in clean_path(o) for k in TERRAIN_CLASSES)]
road_objs = [o for o in terrain_objs if any(k in clean_path(o) for k in ROAD_CLASSES)]
tv, tp = world_tris(terrain_objs)
rv, rp = world_tris(road_objs)
bvh = BVHTree.FromPolygons(tv, tp)
bvh_road = BVHTree.FromPolygons(rv, rp)
log('terrain triangles', len(tp), 'road triangles', len(rp))
zmax = max(v.z for v in tv) + 50.0


def ground_z(tx, ty, tree=bvh):
    hit = tree.ray_cast(Vector((tx, ty, zmax)), Vector((0, 0, -1)))
    return hit[0].z if hit[0] is not None else None


def map_to_twin(mx, my):
    x, y = (mx - TR[0]) / SC, (my - TR[1]) / SC
    c, s = math.cos(-ROT), math.sin(-ROT)
    return c * x - s * y, s * x + c * y


# z0: median road height under the lanelet nodes
node_z = []
on_road = 0
for mx, my in NODES:
    tx, ty = map_to_twin(mx, my)
    zr = ground_z(tx, ty, bvh_road)
    if zr is not None:
        on_road += 1
        node_z.append(zr)
Z0 = round(float(np.median(node_z)), 2)
log('z0', Z0, 'nodes on the road surface', on_road, '/', len(NODES))

# Grid over the terrain's extent, axis-aligned in the map frame, 1 m cells
CELL = 1.0
tvv = np.array([v[:] for v in tv])
corners = np.array([twin_to_map(x, y) for x, y in tvv[:, :2][::max(1, len(tvv) // 20000)]])
gx0 = math.floor(corners[:, 0].min()) - 2
gy0 = math.floor(corners[:, 1].min()) - 2
nx = int(math.ceil(corners[:, 0].max() - gx0)) + 3
ny = int(math.ceil(corners[:, 1].max() - gy0)) + 3
H = np.full((ny, nx), np.nan, np.float64)
for j in range(ny):
    my = gy0 + j * CELL
    for i in range(nx):
        tx, ty = map_to_twin(gx0 + i * CELL, my)
        z = ground_z(tx, ty)
        if z is not None:
            H[j, i] = z - Z0
holes = int(np.isnan(H).sum())
# Fill cells outside the terrain from their nearest neighbours, ring by ring
while np.isnan(H).any():
    P = np.pad(H, 1, constant_values=np.nan)
    nb = np.stack([P[1 + dy:ny + 1 + dy, 1 + dx:nx + 1 + dx] for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dx or dy])
    with np.errstate(all='ignore'):
        fill = np.nanmean(nb, axis=0)
    m = np.isnan(H) & ~np.isnan(fill)
    if not m.any():
        H[np.isnan(H)] = 0.0
        break
    H[m] = fill[m]
Hcm = np.clip(np.round(H * 100), -32768, 32767).astype('<i2')
Hcm.tofile(os.path.join(OUT, 'height.i16'))
height_meta = {'x0': float(gx0), 'y0': float(gy0), 'cell': CELL, 'nx': nx, 'ny': ny, 'scale': 0.01,
               'encoding': 'int16le, row-major from (x0, y0), +x along a row', 'z0_elevation_m': Z0}
json.dump(height_meta, open(os.path.join(OUT, 'height.json'), 'w'), indent=1)
log(f'height grid {nx} x {ny}, {holes} cells off the terrain filled, range {np.nanmin(H):.2f} .. {np.nanmax(H):.2f} m')

# Alignment check: lanelet nodes over the road surface, and how far the others are from it
mask_pts = []
road_v = np.array([v[:] for v in rv])
rmin, rmax = road_v.min(0), road_v.max(0)
step = 0.25
off_d = []
kd = None
ys = np.arange(rmin[1], rmax[1], step)
xs = np.arange(rmin[0], rmax[0], step)
off_nodes = []
for mx, my in NODES:
    tx, ty = map_to_twin(mx, my)
    if ground_z(tx, ty, bvh_road) is None:
        off_nodes.append((tx, ty))
# Distance of the off-road nodes to the road, searched on a 0.25 m lattice within 6 m
for tx, ty in off_nodes:
    best = 99.0
    for r in np.arange(step, 6.01, step):
        found = False
        for a in np.linspace(0, 2 * math.pi, max(8, int(2 * math.pi * r / step)), endpoint=False):
            if ground_z(tx + r * math.cos(a), ty + r * math.sin(a), bvh_road) is not None:
                best, found = r, True
                break
        if found:
            break
    off_d.append(best)
off_d = np.array(off_d) if off_d else np.zeros(1)
report['alignment'] = {
    'fit_residual_mm': round(FIT_RES * 1000, 3),
    'scale': SC, 'rotation_deg': math.degrees(ROT), 'translation_m': [float(TR[0]), float(TR[1])],
    'lanelet_nodes': int(len(NODES)), 'nodes_on_road_surface_pct': round(100 * on_road / len(NODES), 2),
    'off_road_nodes_distance_to_road_m': {'median': float(np.median(off_d)), 'p90': float(np.percentile(off_d, 90)),
                                          'max_searched_6m': float(off_d.max())},
    'z0_elevation_m': Z0,
    'road_height_rel_z0_m': [round(float(min(node_z)) - Z0, 2), round(float(max(node_z)) - Z0, 2)],
}
log('alignment', report['alignment'])

# ------------------------------------------------------------------ map-frame root and export

root = bpy.data.objects.new('world', None)
bpy.context.scene.collection.objects.link(root)
c, s = math.cos(ROT), math.sin(ROT)
root.matrix_world = Matrix(((SC * c, -SC * s, 0, TR[0]), (SC * s, SC * c, 0, TR[1]), (0, 0, SC, -Z0 * SC), (0, 0, 0, 1)))
for o in list(bpy.data.objects):
    if o is not root and o.parent is None:
        mw = o.matrix_world.copy()
        o.parent = root
        o.matrix_world = root.matrix_world @ mw
bpy.context.view_layer.update()

total_tris = sum(tris(o) for o in bpy.data.objects if o.type == 'MESH')
report['raw'] = {'mesh_objects': len([o for o in bpy.data.objects if o.type == 'MESH']), 'triangles_before_decimation': total_tris,
                 'palette_materials': len(palette)}
exp = bpy.ops.export_scene.gltf.get_rna_type().properties.keys()
kw = dict(filepath=os.path.join(OUT, 'world_raw.glb'), export_format='GLB', export_cameras=False, export_lights=False,
          export_apply=True, export_yup=True, export_texcoords=False, export_normals=True, export_extras=False)
for k, v in (('export_vertex_color', 'NONE'), ('export_attributes', False), ('export_image_format', 'NONE'),
             ('export_materials', 'EXPORT'), ('export_tangents', False), ('export_skins', False), ('export_animations', False)):
    if k in exp:
        kw[k] = v
bpy.ops.export_scene.gltf(**kw)
log('raw glb', round(os.path.getsize(kw['filepath']) / 1e6, 2), 'MB')

json.dump(trees, open(os.path.join(OUT, 'trees.json'), 'w'), separators=(',', ':'))
report['trees'] = trees['species']
report['blender'] = bpy.app.version_string
json.dump(report, open(os.path.join(OUT, 'export_report.json'), 'w'), indent=1)
log('done')
