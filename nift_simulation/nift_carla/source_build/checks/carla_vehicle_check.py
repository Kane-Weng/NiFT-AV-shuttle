"""
Check a vehicle blueprint on a running server, without pygame.

Connect to the server (the editor's Play-in-Editor server works too), spawn a vehicle by
blueprint filter, hold throttle for a few seconds, report speed and the wheel geometry,
destroy it.

  python3 carla_vehicle_check.py [--filter micra] [--host localhost] [--port 2000] [--seconds 5]
"""
import argparse
import math
import sys
import time

import carla

ap = argparse.ArgumentParser()
ap.add_argument('--filter', default='micra')
ap.add_argument('--host', default='localhost')
ap.add_argument('--port', type=int, default=2000)
ap.add_argument('--seconds', type=float, default=5.0)
ap.add_argument('--steer', type=float, default=0.0)
a = ap.parse_args()
c = carla.Client(a.host, a.port)
c.set_timeout(60.0)
w = c.get_world()
m = w.get_map()
print('server', c.get_server_version(), 'client', c.get_client_version(), 'map', m.name)
bps = w.get_blueprint_library().filter('vehicle.*' + a.filter + '*')
print('blueprints matching', a.filter, [b.id for b in bps])
if not bps:
    sys.exit(2)
bp = bps[0]
sp = m.get_spawn_points()
print('spawn points', len(sp))
v = None
for tf in sp[:20]:
    v = w.try_spawn_actor(bp, tf)
    if v:
        break
if not v:
    print('spawn failed')
    sys.exit(3)
w.wait_for_tick()
bb = v.bounding_box
print('spawned', v.type_id, 'at', v.get_transform().location, 'bbox extent', bb.extent)
phys = v.get_physics_control()
ws = phys.wheels
pos = [
    carla.Location(x=p.position.x / 100, y=p.position.y / 100, z=p.position.z / 100) for p in ws
]
if len(pos) == 4:
    wb = (
        (pos[0].x + pos[1].x) / 2 - (pos[2].x + pos[3].x) / 2,
        (pos[0].y + pos[1].y) / 2 - (pos[2].y + pos[3].y) / 2,
    )
    print(
        'wheelbase %.3f m, front track %.3f m, rear track %.3f m, radius %.3f m, max steer %.1f '
        'deg, mass %.0f kg'
        % (
            math.hypot(*wb),
            math.hypot(pos[0].x - pos[1].x, pos[0].y - pos[1].y),
            math.hypot(pos[2].x - pos[3].x, pos[2].y - pos[3].y),
            ws[0].radius / 100,
            ws[0].max_steer_angle,
            phys.mass,
        )
    )
v.apply_control(carla.VehicleControl(throttle=0.6, steer=a.steer))
t0 = time.time()
last = None
while time.time() - t0 < a.seconds:
    w.wait_for_tick()
    vel = v.get_velocity()
    last = math.hypot(vel.x, vel.y)
    wz = math.radians(v.get_angular_velocity().z)
    tf = v.get_transform()
    print(
        '  t=%.1f speed %.2f m/s yaw-rate %.3f rad/s radius %s pos (%.1f, %.1f) yaw %.1f'
        % (
            time.time() - t0,
            last,
            wz,
            ('%.2f m' % (last / abs(wz))) if abs(wz) > 0.02 and last > 0.3 else '-',
            tf.location.x,
            tf.location.y,
            tf.rotation.yaw,
        )
    )
    time.sleep(0.5)
v.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0))
w.wait_for_tick()
print(
    'RESULT',
    'DRIVES' if last and last > 0.5 else 'DID NOT MOVE',
    'speed %.2f m/s after %.0f s' % (last or 0, a.seconds),
)
v.destroy()
