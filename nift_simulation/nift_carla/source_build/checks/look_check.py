"""
Look check for vehicle.nift.shuttle against any running server.

Camera sensors round a parked shuttle, day and night, with CARLA's light states switched
through the API (synchronous mode, restored afterwards). Writes <out>/<state>_<view>.png;
blinker states write every frame of 1.5 s to <out>/frames/ (they flash at about 1 Hz).

  python3 look_check.py --out DIR [--spawn N] [--views front34,rear34,...]
      [--states day_off,night_brake,...]

States: day|night _ off|position|low_beam|high_beam|brake|left_blink|right_blink.
Views: see VIEWS.
"""
import argparse
import math
import os
import queue
import sys

import carla

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
ap.add_argument('--spawn', type=int, default=3)
ap.add_argument('--blueprint', default='vehicle.nift.shuttle')
ap.add_argument('--views', default='front34,rear34,side,front,rear,close_side')
ap.add_argument(
    '--states',
    default=(
        'day_off,day_brake,night_off,night_position,night_low_beam,night_brake,'
        'night_left_blink,night_right_blink'
    ),
)
ap.add_argument('--width', type=int, default=1600)
ap.add_argument('--height', type=int, default=1000)
ap.add_argument('--host', default='localhost')
ap.add_argument('--warmup', type=int, default=25)
a = ap.parse_args()
os.makedirs(a.out, exist_ok=True)
VLS = carla.VehicleLightState
STATES = {
    'off': VLS.NONE,
    'position': VLS.Position,
    'low_beam': VLS.Position | VLS.LowBeam,
    'high_beam': VLS.Position | VLS.LowBeam | VLS.HighBeam,
    'brake': VLS.Position | VLS.Brake,
    'left_blink': VLS.LeftBlinker,
    'right_blink': VLS.RightBlinker,
}
# (distance m, angle from the vehicle's forward axis deg, height m, fov)
VIEWS = {
    'front34': (7.0, 35, 2.0, 60),
    'rear34': (7.0, 150, 2.0, 60),
    'side': (6.5, 90, 1.3, 60),
    'front': (6.0, 0, 1.2, 55),
    'rear': (6.0, 180, 1.2, 55),
    'close_side': (3.2, -70, 1.0, 75),
    'top': (0.01, 0, 9.0, 60),
    # level with the amber side strips
    'low_left': (2.2, -90, 0.46, 100),
    'low_right': (2.2, 90, 0.46, 100),
}
AIM_Z = {'low_left': 0.46, 'low_right': 0.46}

c = carla.Client(a.host, 2000)
c.set_timeout(120.0)
w = c.get_world()
orig = w.get_settings()
s = w.get_settings()
s.synchronous_mode = True
s.fixed_delta_seconds = 0.05
w.apply_settings(s)
actors = []
try:
    bp = w.get_blueprint_library().find(a.blueprint)
    print(
        'blueprint',
        bp.id,
        {x.id: str(x) for x in bp if x.id in ('has_lights', 'generation', 'base_type')},
    )
    sps = w.get_map().get_spawn_points()
    v = None
    for i in list(range(a.spawn, len(sps))) + list(range(a.spawn)):
        v = w.try_spawn_actor(bp, sps[i])
        if v is not None:
            print('spawned at spawn point', i, sps[i].location)
            break
    if v is None:
        sys.exit('spawn failed')
    actors.append(v)
    for _ in range(40):
        w.tick()                                  # settle on the suspension
    v.apply_control(carla.VehicleControl(hand_brake=True))
    cams = {}
    for name in a.views.split(','):
        d, ang, h, fov = VIEWS[name]
        cb = w.get_blueprint_library().find('sensor.camera.rgb')
        cb.set_attribute('image_size_x', str(a.width))
        cb.set_attribute('image_size_y', str(a.height))
        cb.set_attribute('fov', str(fov))
        x = d * math.cos(math.radians(ang))
        y = d * math.sin(math.radians(ang))
        if name == 'top':
            ctf = carla.Transform(carla.Location(0, 0, h), carla.Rotation(pitch=-89.9))
        else:
            ctf = carla.Transform(
                carla.Location(x, y, h),
                carla.Rotation(
                    pitch=-math.degrees(math.atan2(h - AIM_Z.get(name, 0.9), d)), yaw=ang + 180
                ),
            )
        cam = w.spawn_actor(cb, ctf, attach_to=v)
        q = queue.Queue()
        cam.listen(q.put)
        cams[name] = (cam, q)
        actors.append(cam)
    for st in a.states.split(','):
        when, light = st.split('_', 1)
        weather = (
            carla.WeatherParameters.ClearNoon
            if when == 'day'
            else carla.WeatherParameters(sun_altitude_angle=-25.0, cloudiness=10.0)
        )
        w.set_weather(weather)
        v.set_light_state(carla.VehicleLightState(STATES[light]))
        for _ in range(a.warmup):
            w.tick()                         # exposure adaptation
        # blinkers flash: keep every frame of 1.5 s
        if 'blink' in light:
            for k in range(30):
                fr = w.tick()
                for name, (cam, q) in cams.items():
                    img = q.get(timeout=30)
                    while img.frame < fr:
                        img = q.get(timeout=30)
                    img.save_to_disk(
                        os.path.join(a.out, 'frames', '%s_%s_f%02d.png' % (st, name, k))
                    )
        else:
            fr = w.tick()
            for name, (cam, q) in cams.items():
                img = q.get(timeout=30)
                while img.frame < fr:
                    img = q.get(timeout=30)
                img.save_to_disk(os.path.join(a.out, '%s_%s.png' % (st, name)))
        for name, (cam, q) in cams.items():
            while not q.empty():
                q.get_nowait()
        print('state', st, 'light state now', v.get_light_state(), flush=True)
finally:
    for x in actors:
        try:
            if x.type_id.startswith('sensor'):
                x.stop()
        except Exception:
            pass
    c.apply_batch([carla.command.DestroyActor(x) for x in actors])
    w.apply_settings(orig)
    w.set_weather(carla.WeatherParameters.ClearNoon)
    print('done')
