import math
import sys
import time

import carla

c = carla.Client('localhost', 2000)
c.set_timeout(60.0)
w = c.get_world()
bp = w.get_blueprint_library().filter('vehicle.nift.shuttle')[0]
tf = w.get_map().get_spawn_points()[3]
v = w.try_spawn_actor(bp, tf) or sys.exit('spawn failed')
w.wait_for_tick()
spec = w.get_spectator()
views = [
    ('rear34', 7.0, 150, 2.6),
    ('side', 6.0, 90, 1.5),
    ('front34', 7.0, 35, 2.2),
    ('top', 10.0, 0, 9.0),
]
for name, d, ang, h in views:
    yaw = tf.rotation.yaw + ang
    loc = v.get_transform().location
    cam = carla.Location(
        loc.x - d * math.cos(math.radians(yaw)), loc.y - d * math.sin(math.radians(yaw)), loc.z + h
    )
    pitch = -math.degrees(math.atan2(h - 0.9, d)) if name != 'top' else -89
    spec.set_transform(carla.Transform(cam, carla.Rotation(pitch=pitch, yaw=yaw)))
    print('view', name, flush=True)
    time.sleep(float(sys.argv[1]) if len(sys.argv) > 1 else 8)
v.destroy()
