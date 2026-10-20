#!/usr/bin/env python3
"""
Show the CARLA world through a camera sensor, in a window or as snapshots.

File: camera_view.py
Author: Kane Weng

Description:
  Look at the CARLA world.  The server renders off-screen, so attach a camera
  sensor and stream its frames to a client to see the simulation. Normally the
  server and this tool are on the same machine (--host localhost); --host also
  takes a remote server, but see the bandwidth note below. This tool:
    - Spawn one `sensor.camera.rgb`, shows the frames in a window (tkinter + PIL, which
      ship with ROS/Ubuntu)
    - Publishes them as sensor_msgs/Image for RViz or rqt_image_view.

  Where the camera goes:
    * if an ego vehicle (role_name `ego`, spawned by carla_bridge_node) exists:
      chase camera rigidly attached behind and above it
    * otherwise: a fixed camera at --pose (map frame x y yaw, default = the
      filter-bench spawn on lanelet 46138), looking down the road

Details:
  - The tool NEVER calls world.tick().  With the bridge running (synchronous
    mode) frames arrive once per bridge tick; without it the server free-runs
    and delivers frames itself.
  - --fps is capped by the bridge tick rate (20 Hz default) and, with a window
    open, by how fast this process can render.  Measured on one machine against
    a 20 Hz bridge (2026-10-01):

        --fps   --no-window      with window
          20      19.96 Hz        10.3 Hz
          15      14.92 Hz         8.5 Hz
          10       9.98 Hz        ~10 Hz

    Two separate effects:
      * The server hits the requested average even for a non-divisor of the
        tick rate, by mixing 1- and 2-tick gaps -- but then frame spacing is
        uneven (at 15 Hz: 0.045-0.104 s, std dev 23 ms, vs 1.2 ms at 20 Hz).
        Prefer an exact divisor (20 / 10 / 6.67 / 5) for even spacing.
      * The tkinter+PIL render in the main loop costs ~0.1 s, so with a window
        the consumer tops out near 10 Hz and the maxsize=2 queue drops the
        rest.  Asking for more than 10 only adds dropped frames; --no-window
        (publish only, view in RViz/rqt) removes the cap.
    The 10 Hz default suits both paths and matches /scan
    (lidar.publish_divider).
  - Bandwidth: width*height*4 bytes per frame off the server, and the ROS
    message is uncompressed rgb8 (width*height*3): 640x360 @ 10 Hz is
    ~6.9 MB/s. That is fine on loopback, which is where this tool is meant to
    run. To watch from another machine, stream the desktop with a remote
    desktop tool rather than pushing this topic across the link.

Usage:
  ros2 run nift_carla carla_camera_view --host <ip>                 # window
  ros2 run nift_carla carla_camera_view --host <ip> --ros           # + /carla/camera/image_raw
  ros2 run nift_carla carla_camera_view --host <ip> --snapshot /tmp/view.png   # one frame, exit
  Press q / Esc in the window (or close it) to quit.
"""

import argparse
import math
import queue
import signal
import sys
import time

import carla
from nift_carla.carla_geo import CarlaGeo
from nift_carla.tools.road_height import RoadHeight
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
import numpy as np

BENCH_POSE = (100.94, 162.66, -1.612)     # config/scenarios/filter_bench.yaml spawn_pose


def find_ego(world, role='ego'):
    """Find the ego vehicle in the CARLA world."""
    for a in world.get_actors().filter('vehicle.*'):
        if a.attributes.get('role_name') == role:
            return a
    return None


def road_z(world, cmap, cx, cy):
    """Return the height of the road at (cx, cy)."""
    try:
        # Raycast: cast 2000 m ray straight down from 1000 m up & find first surface hit
        hit = world.ground_projection(carla.Location(x=cx, y=cy, z=1000.0), 2000.0)
        if hit is not None:
            return hit.location.z
    except Exception:
        pass
    # Fallback: the OpenDRIVE road model (not get_waypoint at z = 0: that picks other roads)
    return RoadHeight(cmap)(cx, cy)


def to_rgb(image) -> np.ndarray:
    """Turn bytes into an RGB image."""
    buf = np.frombuffer(bytes(image.raw_data), dtype=np.uint8)
    return buf.reshape((image.height, image.width, 4))[:, :, 2::-1].copy()   # BGRA -> RGB


class Window:
    """Minimal tkinter viewer; returns False from render() once the user closed it."""

    def __init__(self, title: str, width: int, height: int):
        import tkinter as tk

        from PIL import ImageTk
        self._tk, self._ImageTk = tk, ImageTk
        self.root = tk.Tk()
        self.root.title(title)
        self.root.geometry(f'{width}x{height}')
        self.root.resizable(False, False)
        self.label = tk.Label(self.root)    # a Label widget can hold an image
        self.label.pack()                   # put it in the window
        self.open = True

        # Safe exit
        self.root.protocol('WM_DELETE_WINDOW', self._close)
        self.root.bind('<Key>', lambda e: self._close() if e.keysym in ('q', 'Escape') else None)
        self._photo = None

    def _close(self):
        self.open = False

    def render(self, pil_image) -> bool:
        if not self.open:
            return False

        # Converts the PIL image into a TK image & put in the label
        self._photo = self._ImageTk.PhotoImage(pil_image)
        self.label.configure(image=self._photo)

        try:
            # Run pending redraws/layout
            self.root.update_idletasks()
            # Process all pending events (keys, close button)
            self.root.update()
        except self._tk.TclError:
            self.open = False

        return self.open

    def pump(self):
        if self.open:
            try:
                self.root.update()
            except self._tk.TclError:
                self.open = False

    def destroy(self):
        try:
            self.root.destroy()
        except Exception:
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument('--host', default='localhost')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--width', type=int, default=640)
    ap.add_argument('--height', type=int, default=360)
    ap.add_argument(
        '--fps',
        type=float,
        default=10.0,
        help='frames per second, capped by the bridge tick rate and (with a window) by render '
        'speed ~10 Hz; prefer an exact divisor of the tick rate. See the module docstring.',
    )
    ap.add_argument('--fov', type=float, default=90.0)
    ap.add_argument(
        '--pose',
        nargs=3,
        type=float,
        metavar=('X', 'Y', 'YAW'),
        default=list(BENCH_POSE),
        help='fixed camera pose in the map frame (m, m, rad) when no ego exists',
    )
    ap.add_argument('--datum', nargs=2, type=float, default=[MAP_ORIGIN_LAT, MAP_ORIGIN_LON],
                    metavar=('LAT', 'LON'))
    ap.add_argument(
        '--chase',
        nargs=3,
        type=float,
        metavar=('BACK', 'UP', 'PITCH'),
        default=[7.0, 3.0, -12.0],
        help='chase-camera offset behind/above the ego (m, m) and pitch (deg)',
    )
    ap.add_argument('--ros', action='store_true', help='publish sensor_msgs/Image on --topic')
    ap.add_argument('--topic', default='/carla/camera/image_raw')
    ap.add_argument('--no-window', action='store_true')
    ap.add_argument('--snapshot', default='', help='write the first frame to this PNG and exit')
    args = ap.parse_args(argv)

    from PIL import Image as PilImage
    from PIL import ImageDraw

    # CARLA connection
    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    cmap = world.get_map()
    sync = world.get_settings().synchronous_mode
    print(f'connected to {args.host}:{args.port}, map {cmap.name}, '
          f'{"SYNCHRONOUS (bridge must be ticking)" if sync else "asynchronous"}')

    # Setting up the camera
    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(args.width))
    bp.set_attribute('image_size_y', str(args.height))
    bp.set_attribute('fov', str(args.fov))
    bp.set_attribute('sensor_tick', f'{1.0 / args.fps:.4f}' if args.fps > 0 else '0.0')

    # Place the camera
    ego = find_ego(world)
    if ego is not None:     # An ego vehicle exists -> chase camera
        back, up, pitch = args.chase
        tf = carla.Transform(carla.Location(x=-back, z=up), carla.Rotation(pitch=pitch))
        cam = world.spawn_actor(bp, tf, attach_to=ego, attachment_type=carla.AttachmentType.Rigid)
        print(f'chase camera attached to ego {ego.type_id} (id {ego.id})')
    else:                   # No ego -> fixed camera near the bench pose
        geo = CarlaGeo.from_opendrive(cmap.to_opendrive(), args.datum[0], args.datum[1])
        x, y, yaw = args.pose
        cx, cy, _ = geo.map_to_carla(x, y)
        cyaw = geo.map_yaw_to_carla_yaw_deg(x, y, yaw)
        r = math.radians(cyaw)
        loc = carla.Location(
            x=cx - 6.0 * math.cos(r), y=cy - 6.0 * math.sin(r), z=road_z(world, cmap, cx, cy) + 3.0
        )
        cam = world.spawn_actor(bp, carla.Transform(loc, carla.Rotation(pitch=-12.0, yaw=cyaw)))
        print(
            f'no ego on the server; fixed camera at map ({x:.1f}, {y:.1f}) yaw {yaw:.2f} -> CARLA '
            f'{loc}'
        )

    # Thread-safe queue for getting frames across threads
    #   - Producer/consumer design
    #   - cap size of 2 to prevent unlimited growth
    frames: 'queue.Queue' = queue.Queue(maxsize=2)

    # Background thread (CARLA)
    def _on_image(img):
        try:
            frames.put_nowait(img)  # Producer: drops the frame into the queue
        except queue.Full:
            pass
    cam.listen(_on_image)

    node = pub = None
    if args.ros:
        import rclpy
        from sensor_msgs.msg import Image
        from std_msgs.msg import Header
        rclpy.init()
        node = rclpy.create_node('carla_camera_view')
        pub = node.create_publisher(Image, args.topic, 5)
        print(f'publishing {args.topic}')

    # Window and signals
    win = None
    if not args.no_window and not args.snapshot:
        win = Window('CARLA - Mcity Digital Twin', args.width, args.height)
    stop = [False]
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))

    # Main thread
    n = 0
    t_last = time.monotonic()
    try:
        while not stop[0]:
            try:
                img = frames.get(timeout=0.2)   # Consumer: takes frames out
            except queue.Empty:
                if win is not None:
                    win.pump()                  # keep the window responsive
                    if not win.open:
                        break
                if sync and n == 0 and time.monotonic() - t_last > 5.0:
                    print(
                        'no frames after 5 s: server is in synchronous mode and nobody is '
                        'ticking. '
                        'Start carla_bridge_node or restore asynchronous mode (see the server '
                        'manual).'
                    )
                    t_last = time.monotonic()
                continue

            # Update for frame arrival
            n += 1
            rgb = to_rgb(img)

            # ROS branch
            if pub is not None:
                msg = Image()
                msg.header = Header(frame_id='carla_camera')
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.height, msg.width = rgb.shape[:2]
                # tell subs how many channels exist, in what order, and with what bit depth.
                msg.encoding = 'rgb8'
                msg.step = msg.width * 3    # bytes
                msg.data = rgb.tobytes()
                pub.publish(msg)

            # Snapshot branch
            if args.snapshot:
                PilImage.fromarray(rgb).save(args.snapshot)
                print(
                    f'wrote {args.snapshot} ({rgb.shape[1]}x{rgb.shape[0]}, sim '
                    f't={img.timestamp:.2f} s)'
                )
                break

            # Window branch
            if win is not None:
                pil = PilImage.fromarray(rgb)
                # Draw a status line on the image
                label = (
                    f'CARLA {cmap.name.split("/")[-1]}  frame {img.frame}  t={img.timestamp:.1f}s'
                )
                if ego is not None:
                    v = ego.get_velocity()
                    label += f'  ego {math.hypot(v.x, v.y):.1f} m/s'
                ImageDraw.Draw(pil).text((8, 6), label, fill=(255, 255, 255))
                if not win.render(pil):
                    break
    finally:
        try:
            cam.stop()
            cam.destroy()
        except Exception:
            pass
        if win is not None:
            win.destroy()
        if node is not None:
            node.destroy_node()
            import rclpy
            rclpy.shutdown()
        print(f'camera destroyed after {n} frame(s)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
