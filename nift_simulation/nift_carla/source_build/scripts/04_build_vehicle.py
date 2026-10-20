"""
Make vehicle.nift.shuttle in the CARLA editor (step 4, scripted part).

Built from the step-3 FBX files by duplicating the Nissan Micra's blueprints (same CARLA
skeleton). Headless:

  UE4Editor-Cmd CarlaUE4.uproject -run=pythonscript -script=04_build_vehicle.py
      -NullRHI -unattended -nopause -nosplash

Needs UNiftVehicleSetup (CarlaTools plugin, NiftVehicleSetup.cpp) for the physics asset,
the custom collision mesh and the light mesh component (tagged "emissive": CARLA's
BaseVehiclePawn drives the light parameters on it).
Idempotent: re-running re-imports and re-applies the settings.
"""
import os

import unreal

RIG = os.environ.get(
    'NIFT_RIG_DIR',
    os.path.join(
        os.environ.get('CARLA_BUILD_DIR', os.path.expanduser('~/carla_build')), 'tools/rig'
    ),
)  # the step-3 FBX files
SK_DIR = '/Game/Carla/Static/Car/4Wheeled/Nift_Shuttle'
BP_DIR = '/Game/Carla/Blueprints/Vehicles/NiftShuttle'
MICRA_SK = '/Game/Carla/Static/Car/4Wheeled/Nissan_Micra'
MICRA_BP = '/Game/Carla/Blueprints/Vehicles/NissanMicra'
NAME = 'NiftShuttle'
# Wheels measured from the mesh (step-3 report), not vehicle.yaml's 0.228 / 0.213 m: CARLA's wheels
# must fit the body
WHEEL_RADIUS_CM, WHEEL_WIDTH_CM = 22.75, 21.2
STEER_DEG, MASS_KG = 15.0, 500.0   # owner: nift_common/vehicle.yaml max_steering_deg, mass (copy: Unreal's Python has no ROS)  # noqa: E501
WHEELS = {
    'FLW': ('Wheel_Front_Left', True),
    'FRW': ('Wheel_Front_Right', True),
    'RLW': ('Wheel_Rear_Left', False),
    'RRW': ('Wheel_Rear_Right', False),
}
# Light components copied from the Lincoln 2020 (its tuned intensity / colour / cone / shadows) to
# the shuttle's lamp strips (x +-145.5 cm, y +-52 cm, lower strips z 34-50, upper z 53-116; Unreal
# y > 0 = right). CARLA's BaseVehiclePawn groups them by name and switches them with the light
# state; the blinkers also only flash with blinker components.
LIGHT_SOURCE_BP = '/Game/Carla/Blueprints/Vehicles/LincolnMKZ2020/BP_Lincoln2020'
LIGHT_COMPONENTS = [  # (name on the shuttle, Lincoln component, x, y, z cm)
    ('front-low_beam-l-1', 'front-low_beam-l-1', 148, -52, 45),
    ('front-low_beam-r-1', 'front-low_beam-r-1', 148, 52, 45),
    ('front-high_beam-l-1', 'front-high_beam-l-1', 148, -52, 50),
    ('front-high_beam-r-1', 'front-high_beam-r-1', 148, 52, 50),
    ('front-position-l-1', 'front-position-l-1', 148, -52, 85),
    ('front-position-r-1', 'front-position-r-1', 148, 52, 85),
    ('front-blinker-l-1', 'front-blinker-l-1', 148, -52, 42),
    ('front-blinker-r-1', 'front-blinker-r-1', 148, 52, 42),
    ('back-blinker-l-1', 'back-blinker-l-1', -148, -52, 42),
    ('back-blinker-r-1', 'back-blinker-r-1', -148, 52, 42),
    ('back-position-l', 'back-position-l', -148, -52, 85),
    ('back-position-r', 'back-position-l', -148, 52, 85),
    ('back-brake-l-1', 'back-brake-l-1', -148, -52, 60),
    ('back-brake-r-1', 'back-brake-l-1', -148, 52, 60),
    ('interior-1', 'interior-1', 0, 0, 175),
]
eal = unreal.EditorAssetLibrary
def log(*a): unreal.log_warning('P4 ' + ' '.join(str(x) for x in a))


class P4Error(Exception):
    pass


def fail(*a):
    unreal.log_error('P4 FAIL ' + ' '.join(str(x) for x in a))
    raise P4Error(' '.join(str(x) for x in a))


def import_fbx(fbx, dest_dir, name, skeletal, skeleton=None, physics=False):
    ui = unreal.FbxImportUI()
    ui.set_editor_property('import_mesh', True)
    ui.set_editor_property('import_materials', False)
    ui.set_editor_property('import_textures', False)
    ui.set_editor_property('import_animations', False)
    ui.set_editor_property('import_as_skeletal', skeletal)
    ui.set_editor_property(
        'mesh_type_to_import',
        (
            unreal.FBXImportType.FBXIT_SKELETAL_MESH
            if skeletal
            else unreal.FBXImportType.FBXIT_STATIC_MESH
        ),
    )
    if skeletal:
        ui.set_editor_property('skeleton', skeleton)
        ui.set_editor_property('create_physics_asset', physics)
        sd = ui.get_editor_property('skeletal_mesh_import_data')
        sd.set_editor_property(
            'normal_import_method', unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS
        )
        sd.set_editor_property('import_morph_targets', False)
        sd.set_editor_property('update_skeleton_reference_pose', False)
        sd.set_editor_property('use_t0_as_ref_pose', False)
        sd.set_editor_property('convert_scene', True)
    else:
        sd = ui.get_editor_property('static_mesh_import_data')
        sd.set_editor_property('combine_meshes', True)
        sd.set_editor_property('auto_generate_collision', False)
        sd.set_editor_property(
            'normal_import_method', unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS
        )
        sd.set_editor_property('convert_scene', True)
    task = unreal.AssetImportTask()
    task.set_editor_property('filename', fbx)
    task.set_editor_property('destination_path', dest_dir)
    task.set_editor_property('destination_name', name)
    task.set_editor_property('replace_existing', True)
    task.set_editor_property('automated', True)
    task.set_editor_property('save', True)
    task.set_editor_property('options', ui)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
    paths = list(task.get_editor_property('imported_object_paths'))
    log('imported', fbx, '->', paths)
    if not paths:
        fail('import produced nothing for', fbx)
    obj = unreal.load_object(None, paths[0])
    if obj is None:
        fail('could not load', paths[0])
    want = dest_dir + '/' + name
    if obj.get_name() != name:
        # Unreal appended the FBX node name (package SK_X, object SK_X_node). An in-place rename
        # fails, so go through a temporary package, drop the redirector, rename back; if anything
        # fails keep the suffixed asset.
        tmp = want + '_tmp'
        try:
            if eal.does_asset_exist(tmp):
                eal.delete_asset(tmp)
            if eal.rename_loaded_asset(obj, tmp):
                try:
                    eal.delete_asset(paths[0])
                except Exception as e:
                    log('redirector delete', e)
                o2 = eal.load_asset(tmp)
                if o2 and eal.rename_loaded_asset(o2, want):
                    o3 = eal.load_asset(want)
                    if o3:
                        obj = o3
                    else:
                        obj = eal.load_asset(tmp) or obj
                else:
                    obj = eal.load_asset(tmp) or obj
        except Exception as e:
            log('rename failed, keeping', obj.get_path_name(), e)
        log('asset now', obj.get_path_name())
    return obj


def main():
    # ---------------------------------------------------------------- 1. meshes
    micra_skel = eal.load_asset(MICRA_SK + '/SK_Nissan_Micra_Skeleton')
    sk = import_fbx(
        os.path.join(RIG, 'SK_nift_shuttle.fbx'),
        SK_DIR,
        'SK_' + NAME,
        True,
        skeleton=micra_skel,
        physics=True,
    )
    if not isinstance(sk, unreal.SkeletalMesh):
        fail('skeletal import gave', sk)
    smc = import_fbx(os.path.join(RIG, 'SMC_nift_shuttle.fbx'), SK_DIR, 'SMC_' + NAME, False)
    sm_sc = import_fbx(os.path.join(RIG, 'SM_sc_nift_shuttle.fbx'), SK_DIR, 'SM_sc_' + NAME, False)
    lights = import_fbx(
        os.path.join(RIG, 'SM_nift_shuttle_lights.fbx'), SK_DIR, 'SM_' + NAME + '_Lights', False
    )
    log('light mesh', lights.get_name(), 'bbox', lights.get_bounding_box())
    pa = sk.get_editor_property('physics_asset')
    if pa is None:
        fail('no physics asset created on import')
    log(
        'SK',
        sk.get_name(),
        'skeleton',
        sk.get_editor_property('skeleton').get_name(),
        'physics asset',
        pa.get_name(),
        'materials',
        [
            str(m.get_editor_property('material_slot_name'))
            for m in sk.get_editor_property('materials')
        ],
    )
    bounds = sk.get_bounds()
    log('SK bounds extent (cm)', bounds.box_extent, 'origin', bounds.origin)
    for b in ('Vehicle_Base', 'Wheel_Front_Left', 'Wheel_Rear_Right'):
        log('bone', b, unreal.NiftVehicleSetup.get_ref_bone_component_transform(sk, b))

    # ---------------------------------------------------------------- 2. physics asset + raycast
    # mesh (C++ helper)
    n = unreal.NiftVehicleSetup.setup_vehicle_physics_asset(
        pa, sk, smc, 'Vehicle_Base', [w[0] for w in WHEELS.values()], WHEEL_RADIUS_CM
    )
    log('physics asset bodies configured', n)
    if n != 5:
        fail('expected 5 bodies, got', n)
    eal.save_loaded_asset(pa)
    eal.save_loaded_asset(sk)

    # ---------------------------------------------------------------- 3. duplicate the Micra's
    # blueprints
    def dup(src, dst):
        if eal.does_asset_exist(dst):
            eal.delete_asset(dst)
        a = eal.duplicate_asset(src, dst)
        if not a:
            fail('duplicate failed', src, dst)
        return a
    animbp = dup(MICRA_SK + '/AnimBP_Nissan_Micra', SK_DIR + '/AnimBP_' + NAME)
    wheel_bps = {
        k: dup(MICRA_BP + '/BP_NissanMicra_' + k, BP_DIR + '/BP_' + NAME + '_' + k) for k in WHEELS
    }
    veh_bp = dup(MICRA_BP + '/BP_NissanMicra', BP_DIR + '/BP_' + NAME)

    def gen_class(bp):
        try:
            c = bp.get_editor_property('generated_class')
        except Exception:
            c = None
        return c or unreal.load_class(
            None, bp.get_path_name().split('.')[0] + '.' + bp.get_name() + '_C'
        )

    def cdo_of(bp): return unreal.get_default_object(gen_class(bp))
    # wheels
    for k, (bone, front) in WHEELS.items():
        c = cdo_of(wheel_bps[k])
        c.set_editor_property('shape_radius', WHEEL_RADIUS_CM)
        c.set_editor_property('shape_width', WHEEL_WIDTH_CM)
        c.set_editor_property('steer_angle', STEER_DEG if front else 0.0)
        c.set_editor_property('affected_by_handbrake', not front)
        unreal.NiftVehicleSetup.compile_blueprint(wheel_bps[k])
        eal.save_loaded_asset(wheel_bps[k])
        log(
            'wheel',
            k,
            'radius',
            c.get_editor_property('shape_radius'),
            'steer',
            c.get_editor_property('steer_angle'),
            'handbrake',
            c.get_editor_property('affected_by_handbrake'),
        )
    # vehicle
    c = cdo_of(veh_bp)
    mesh = c.get_editor_property('mesh')
    mesh.set_editor_property('skeletal_mesh', sk)
    mesh.set_editor_property('anim_class', gen_class(animbp))
    mv = c.get_editor_property('vehicle_movement')
    ws = list(mv.get_editor_property('wheel_setups'))
    for w in ws:
        bone = str(w.get_editor_property('bone_name'))
        k = next(k for k, v in WHEELS.items() if v[0] == bone)
        w.set_editor_property('wheel_class', gen_class(wheel_bps[k]))
    mv.set_editor_property('wheel_setups', ws)
    mv.set_editor_property('mass', MASS_KG)
    unreal.NiftVehicleSetup.set_blueprint_component_static_mesh(veh_bp, 'CustomCollision', sm_sc)
    if not unreal.NiftVehicleSetup.add_blueprint_static_mesh_component(
        veh_bp, 'SM_' + NAME + '_Lights', lights, ['emissive']
    ):
        fail('could not add the light mesh component')
    light_src = eal.load_asset(LIGHT_SOURCE_BP)
    for name, src, x, y, z in LIGHT_COMPONENTS:
        if not unreal.NiftVehicleSetup.copy_blueprint_component(
            veh_bp, light_src, src, name, unreal.Vector(x, y, z)
        ):
            fail('could not copy light component', src)
    unreal.NiftVehicleSetup.compile_blueprint(veh_bp)
    eal.save_loaded_asset(animbp)
    eal.save_loaded_asset(veh_bp)
    c = cdo_of(veh_bp)
    mv = c.get_editor_property('vehicle_movement')
    log(
        'vehicle BP mesh',
        c.get_editor_property('mesh').get_editor_property('skeletal_mesh').get_name(),
        'anim',
        c.get_editor_property('mesh').get_editor_property('anim_class').get_name(),
        'mass',
        mv.get_editor_property('mass'),
        'wheels',
        [
            (
                str(w.get_editor_property('bone_name')),
                w.get_editor_property('wheel_class').get_name(),
            )
            for w in mv.get_editor_property('wheel_setups')
        ],
    )
    for line in unreal.NiftVehicleSetup.describe_blueprint_components(veh_bp).splitlines():
        log('component', line)

    # ---------------------------------------------------------------- 4. vehicle factory entry
    vf = eal.load_asset('/Game/Carla/Blueprints/Vehicles/VehicleFactory')
    vcdo = cdo_of(vf)
    defs = list(vcdo.get_editor_property('vehicles'))
    defs = [
        d
        for d in defs
        if not (
            str(d.get_editor_property('make')) == 'nift'
            and str(d.get_editor_property('model')) == 'shuttle'
        )
    ]
    template = next(
        d for d in defs if d.get_editor_property('class_').get_name() == 'BP_NissanMicra_C'
    )
    d = unreal.VehicleParameters()
    d.assign(template)
    d.set_editor_property('make', 'nift')
    d.set_editor_property('model', 'shuttle')
    d.set_editor_property('class_', gen_class(veh_bp))
    d.set_editor_property('number_of_wheels', 4)
    d.set_editor_property('generation', 2)
    d.set_editor_property('has_lights', True)
    defs.append(d)
    vcdo.set_editor_property('vehicles', defs)
    unreal.NiftVehicleSetup.compile_blueprint(vf)
    eal.save_loaded_asset(vf)
    log(
        'factory now',
        len(vcdo.get_editor_property('vehicles')),
        'vehicles; last',
        vcdo.get_editor_property('vehicles')[-1],
    )
    log('DONE')


try:
    main()
except P4Error as e:
    unreal.log_error('P4 ABORTED: %s' % e)
except Exception as e:
    import traceback
    unreal.log_error("P4 EXCEPTION: %s\n%s" % (e, traceback.format_exc()))
