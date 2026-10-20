"""
Assign CARLA materials to the shuttle's slots and its light mesh (step 4, materials).

The 18 slots of SK_NiftShuttle (colour v2 names). Headless:
UE4Editor-Cmd … -run=pythonscript -script=04_materials.py -NullRHI

- Paint, rubber, metal, rims: stock instances in Content/Carla/Static/Car/GeneralMaterials
  (some tinted copies).
- Glass: CARLA's car-glass materials are static-mesh only and the shuttle's glass is part
  of its skeletal mesh, so one translucent material of our own (M_Nift_Glass, one-sided:
  the CAD panes are closed ~6 mm slabs, so one surface per wall faces the camera) with two
  instances: smoked panes ~45 % see-through (measured on the garage photos) and the darker
  lower bands.
- Logos: one material per decal texture (UV channel 1 = Blender's DecalUV) that blends the
  logo over the same glass.
- Lenses: the body's Light_* slots are the lenses switched off; the light mesh
  (SM_NiftShuttle_Lights, the component tagged "emissive") gets an instance of CARLA's
  M_VehicleLights_Master, which CARLA switches per light function.
"""
import os

import unreal

eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
def log(*a): unreal.log_warning('P4M ' + ' '.join(str(x) for x in a))


G = '/Game/Carla/Static/Car/GeneralMaterials'
D = '/Game/Carla/Static/Car/4Wheeled/Nift_Shuttle'
MD = D + '/Materials'
LIGHTS_MASTER = '/Game/Carla/Static/GenericMaterials/Vehicles/M_VehicleLights_Master'
TEX = os.environ.get(
    'NIFT_TEXTURE_DIR',
    os.path.join(
        os.environ.get('CARLA_BUILD_DIR', os.path.expanduser('~/carla_build')),
        'tools/rig/textures',
    ),
)
# linear; the smoked panes read near-black in reflection, the cabin shows through
GLASS_TINT = (0.012, 0.012, 0.013)
GLASS = {
    'MI_Nift_Glass_Smoked': (GLASS_TINT, 0.55),
    'MI_Nift_Glass_SmokedDark': ((0.008, 0.008, 0.009), 0.75),
}
sk = eal.load_asset(D + '/SK_NiftShuttle')


def tinted(src, name, rgb, metallic=None, roughness=None):
    dst = MD + '/' + name
    if eal.does_asset_exist(dst):
        eal.delete_asset(dst)
    mi = eal.duplicate_asset(G + '/' + src, dst)
    mel.set_material_instance_vector_parameter_value(
        mi, 'Base Color', unreal.LinearColor(*rgb, 1.0)
    )
    if metallic is not None:
        mel.set_material_instance_scalar_parameter_value(mi, 'Metallic', metallic)
    if roughness is not None:
        mel.set_material_instance_scalar_parameter_value(mi, 'Clear Coat Top Roughness', roughness)
    mel.update_material_instance(mi)
    eal.save_loaded_asset(mi)
    return mi


def instance(parent, name, scalars=None, vectors=None):
    dst = MD + '/' + name
    if eal.does_asset_exist(dst):
        eal.delete_asset(dst)
    mi = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, MD, unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew()
    )
    mel.set_material_instance_parent(mi, parent)
    for k, v in (scalars or {}).items():
        mel.set_material_instance_scalar_parameter_value(mi, k, v)
    for k, v in (vectors or {}).items():
        mel.set_material_instance_vector_parameter_value(mi, k, unreal.LinearColor(*v, 1.0))
    mel.update_material_instance(mi)
    eal.save_loaded_asset(mi)
    return mi


def import_texture(png):
    name = 'T_Nift_' + os.path.splitext(os.path.basename(png))[0]
    task = unreal.AssetImportTask()
    task.set_editor_property('filename', png)
    task.set_editor_property('destination_path', MD)
    task.set_editor_property('destination_name', name)
    task.set_editor_property('replace_existing', True)
    task.set_editor_property('automated', True)
    task.set_editor_property('save', True)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    paths = list(task.get_editor_property('imported_object_paths'))
    t = unreal.load_object(None, paths[0]) if paths else None
    if not t:
        raise RuntimeError('texture import failed ' + png)
    t.set_editor_property('address_x', unreal.TextureAddress.TA_CLAMP)
    t.set_editor_property('address_y', unreal.TextureAddress.TA_CLAMP)
    t.set_editor_property('srgb', True)
    # streamed, the logos stayed at a low mip on the vehicle (blurred text)
    t.set_editor_property('never_stream', True)
    eal.save_loaded_asset(t)
    return t


def new_translucent(name, two_sided):
    dst = MD + '/' + name
    if eal.does_asset_exist(dst):
        eal.delete_asset(dst)
    mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        name, MD, unreal.Material, unreal.MaterialFactoryNew()
    )
    mat.set_editor_property('blend_mode', unreal.BlendMode.BLEND_TRANSLUCENT)
    mat.set_editor_property('two_sided', two_sided)
    # otherwise the game shows the default material on the vehicle
    mat.set_editor_property('used_with_skeletal_mesh', True)
    try:   # per-pixel lit translucency: sharp sun and sky reflections on the glass
        mat.set_editor_property(
            'translucency_lighting_mode',
            unreal.TranslucencyLightingMode.TLM_SURFACE_PER_PIXEL_LIGHTING,
        )
    except Exception as e:
        log('translucency lighting mode not set:', e)
    return mat


def const(mat, value, x, y):
    c = mel.create_material_expression(mat, unreal.MaterialExpressionConstant, x, y)
    c.set_editor_property('r', value)
    return c


def param(mat, cls, name, default, x, y):
    p = mel.create_material_expression(mat, cls, x, y)
    p.set_editor_property('parameter_name', name)
    p.set_editor_property(
        'default_value',
        unreal.LinearColor(*default, 1.0) if isinstance(default, tuple) else default,
    )
    return p


def glass_material():
    mat = new_translucent('M_Nift_Glass', two_sided=False)
    tint = param(mat, unreal.MaterialExpressionVectorParameter, 'Tint', GLASS_TINT, -400, 0)
    opa = param(mat, unreal.MaterialExpressionScalarParameter, 'Opacity', 0.55, -400, 200)
    rough = param(mat, unreal.MaterialExpressionScalarParameter, 'Roughness', 0.05, -400, 300)
    spec = param(mat, unreal.MaterialExpressionScalarParameter, 'Specular', 0.5, -400, 400)
    mel.connect_material_property(tint, '', unreal.MaterialProperty.MP_BASE_COLOR)
    mel.connect_material_property(opa, '', unreal.MaterialProperty.MP_OPACITY)
    mel.connect_material_property(rough, '', unreal.MaterialProperty.MP_ROUGHNESS)
    mel.connect_material_property(spec, '', unreal.MaterialProperty.MP_SPECULAR)
    mel.recompile_material(mat)
    eal.save_loaded_asset(mat)
    return mat


def decal_material(name, tex):
    """
    Make the logo vinyl material on the smoked glass.

    Colour and opacity blend from the glass (alpha 0) to the logo (alpha 1).
    """
    # two-sided: the far side's logos show mirrored through the glass, as on the shuttle
    mat = new_translucent(name, two_sided=True)
    uv = mel.create_material_expression(mat, unreal.MaterialExpressionTextureCoordinate, -900, 0)
    uv.set_editor_property('coordinate_index', 1)
    ts = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -700, 0)
    ts.set_editor_property('texture', tex)
    mel.connect_material_expressions(uv, '', ts, 'UVs')
    tint = param(mat, unreal.MaterialExpressionVectorParameter, 'Tint', GLASS_TINT, -700, -250)
    base = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, -300, -100
    )
    mel.connect_material_expressions(tint, '', base, 'A')
    mel.connect_material_expressions(ts, 'RGB', base, 'B')
    mel.connect_material_expressions(ts, 'A', base, 'Alpha')
    mel.connect_material_property(base, '', unreal.MaterialProperty.MP_BASE_COLOR)
    opa = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, -300, 150
    )
    mel.connect_material_expressions(
        param(mat, unreal.MaterialExpressionScalarParameter, 'Opacity', 0.55, -700, 250),
        '',
        opa,
        'A',
    )
    mel.connect_material_expressions(const(mat, 0.97, -700, 350), '', opa, 'B')
    mel.connect_material_expressions(ts, 'A', opa, 'Alpha')
    mel.connect_material_property(opa, '', unreal.MaterialProperty.MP_OPACITY)
    rough = mel.create_material_expression(
        mat, unreal.MaterialExpressionLinearInterpolate, -300, 400
    )
    mel.connect_material_expressions(const(mat, 0.05, -700, 450), '', rough, 'A')
    mel.connect_material_expressions(const(mat, 0.45, -700, 520), '', rough, 'B')
    mel.connect_material_expressions(ts, 'A', rough, 'Alpha')
    mel.connect_material_property(rough, '', unreal.MaterialProperty.MP_ROUGHNESS)
    mel.connect_material_property(
        const(mat, 0.5, -300, 600), '', unreal.MaterialProperty.MP_SPECULAR
    )
    mel.recompile_material(mat)
    eal.save_loaded_asset(mat)
    return mat


m = {
    n: eal.load_asset(G + '/' + n)
    for n in [
        'Black',
        'Rubber_Inst',
        'BrushedAluminiumWhite_Inst',
        'plastiCbLACK',
        'White',
        'BrightRed',
        'Yellow',
    ]
}
for k, v in m.items():
    if v is None:
        raise RuntimeError('missing material ' + k)
wood = tinted('White', 'MI_Nift_Wood', (0.70, 0.46, 0.24), metallic=0.0, roughness=0.5)
seat = tinted('White', 'MI_Nift_SeatGrey', (0.27, 0.27, 0.29), metallic=0.0, roughness=0.8)
dark = tinted('Black', 'MI_Nift_PowderCoat', (0.03, 0.03, 0.032), metallic=0.0, roughness=0.7)
# lenses switched off (the light mesh glows on top of them when CARLA turns a light on)
lens_red = tinted(
    'BrightRed', 'MI_Nift_LensRed', (0.22, 0.004, 0.004), metallic=0.0, roughness=0.15
)
lens_white = tinted('White', 'MI_Nift_LensWhite', (0.45, 0.45, 0.46), metallic=0.0, roughness=0.15)
lens_amber = tinted('Yellow', 'MI_Nift_Amber', (0.40, 0.12, 0.0), metallic=0.0, roughness=0.15)
glass_master = glass_material()
glass = {
    name: instance(glass_master, name, {'Opacity': o}, {'Tint': t})
    for name, (t, o) in GLASS.items()
}
decals = {}
for slot in ['Decal_Front', 'Decal_Rear', 'Decal_Side_L', 'Decal_Side_R']:
    t = import_texture(os.path.join(TEX, slot + '.png'))
    decals[slot] = decal_material('M_Nift_' + slot, t)
    log('decal', slot, t.get_name(), '->', decals[slot].get_name())
mapping = {
    'Paint_Black': m['Black'],
    'Skirt_Black_Textured': m['plastiCbLACK'],
    'PowderCoat_DarkGrey': dark,
    'Glass_Smoked': glass['MI_Nift_Glass_Smoked'],
    'Glass_Smoked_Dark': glass['MI_Nift_Glass_SmokedDark'],
    'Light_Front_White': lens_white,
    'Light_Tail_Red': lens_red,
    'Light_Indicator_Amber': lens_amber,
    'Rubber_Tyre': m['Rubber_Inst'],
    'Rubber_Black': m['Rubber_Inst'],
    'Wheel_Rim_Yellow': m['Yellow'],  # rims: CAD colour, unseen on the photos
    'Metal_Satin': m['BrushedAluminiumWhite_Inst'],
    'Wood_Light': wood,
    'Seat_Grey': seat,
}
mapping.update(decals)
mats = list(sk.get_editor_property('materials'))
out = []
for sm in mats:
    name = str(sm.get_editor_property('material_slot_name'))
    mi = mapping.get(name)
    if mi is None:
        log('no mapping for slot', name)
    nm = unreal.SkeletalMaterial()
    nm.set_editor_property('material_slot_name', name)
    nm.set_editor_property('material_interface', mi)
    out.append(nm)
sk.set_editor_property('materials', out)
eal.save_loaded_asset(sk)
sk2 = eal.load_asset(D + '/SK_NiftShuttle')
log(
    'slots now',
    [
        (
            str(x.get_editor_property('material_slot_name')),
            (
                x.get_editor_property('material_interface').get_name()
                if x.get_editor_property('material_interface')
                else None
            ),
        )
        for x in sk2.get_editor_property('materials')
    ],
)

# light mesh: CARLA's vehicle-lights master, off = faint lens tint (Opacity as the stock Lincoln
# 2020)
lights_mi = instance(eal.load_asset(LIGHTS_MASTER), 'MI_Nift_VehicleLights', {'Opacity': 0.1})
lights_sm = eal.load_asset(D + '/SM_NiftShuttle_Lights')
if lights_sm is None:
    raise RuntimeError('SM_NiftShuttle_Lights missing: run 04_build_vehicle.py first')
lights_sm.set_material(0, lights_mi)
eal.save_loaded_asset(lights_sm)
log('light mesh material', lights_sm.get_material(0).get_name())
log('DONE')
