"""
tf2_ros.Buffer: a time-stamped transform tree with tf2's lookup rules.

File: tf2_ros/buffer.py

Description:
  Checked against Humble's tf2 in nift_pub_deps:humble.
  - Each child frame keeps its transforms for cache_time (10 s). An insert older than the newest
    minus cache_time is dropped (TF_OLD_DATA, logged). Before an insert, entries older than the
    newest minus cache_time are pruned. The same stamp again replaces the old entry.
  - Static transforms (/tf_static) hold at every time.
  - lookup_transform(target, source, Time()) uses the latest time common to the chain (the oldest
    of the newest stamps of its dynamic links; 0 if all are static). Any other time interpolates
    (translation linearly, rotation by slerp) and raises ExtrapolationException outside the data.
  - A missing target (checked first) or source raises LookupException, two unconnected trees raise
    ConnectivityException, all with tf2's messages. Both are TransformException.
  - The same frame twice gives the identity, known frame or not. Its stamp is the requested time,
    or for Time() the frame's newest stamp when it is a dynamic child frame (0 otherwise).
  - Pruning happens before an insert, against the newest stamp before it: after inserting 30 s
    next to 5 s and 6 s, 6 s can still be looked up; after the next insert it is gone (probed in
    nift_pub_deps:humble).
  - timeout is accepted and ignored: in one thread nothing can arrive while we wait.
"""
import bisect
import math

from geometry_msgs.msg import TransformStamped

from fakeros._time import Duration


class TransformException(Exception):
    pass


class LookupException(TransformException):
    pass


class ConnectivityException(TransformException):
    pass


class ExtrapolationException(TransformException):
    pass


class InvalidArgumentException(TransformException):
    pass


class TimeoutException(TransformException):
    pass


_IDENTITY = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))     # (translation, quaternion w x y z)


def _qmul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _rot(q, v):
    _, x, y, z = _qmul(_qmul(q, (0.0, v[0], v[1], v[2])), (q[0], -q[1], -q[2], -q[3]))
    return (x, y, z)


def _inv(T):
    t, q = T
    qi = (q[0], -q[1], -q[2], -q[3])
    ti = _rot(qi, t)
    return ((-ti[0], -ti[1], -ti[2]), qi)


def _compose(A, B):
    """A = T(a <- b), B = T(b <- c) -> T(a <- c)."""
    ta, qa = A
    tb, qb = B
    r = _rot(qa, tb)
    return ((ta[0] + r[0], ta[1] + r[1], ta[2] + r[2]), _qmul(qa, qb))


def _slerp(q0, q1, s):
    dot = sum(a * b for a, b in zip(q0, q1))
    if dot < 0.0:
        q1 = tuple(-c for c in q1)
        dot = -dot
    if dot > 0.9995:
        out = tuple(a + s * (b - a) for a, b in zip(q0, q1))
    else:
        theta = math.acos(min(1.0, dot))
        sin_t = math.sin(theta)
        w0 = math.sin((1.0 - s) * theta) / sin_t
        w1 = math.sin(s * theta) / sin_t
        out = tuple(w0 * a + w1 * b for a, b in zip(q0, q1))
    n = math.sqrt(sum(c * c for c in out))
    return tuple(c / n for c in out)


def _ns(time):
    """rclpy Time, builtin_interfaces Time or None -> nanoseconds (0 = latest)."""
    if time is None:
        return 0
    if hasattr(time, 'nanoseconds'):
        return int(time.nanoseconds)
    if hasattr(time, 'sec') and hasattr(time, 'nanosec'):
        return int(time.sec) * 1_000_000_000 + int(time.nanosec)
    raise TypeError(f'tf2: a time must be rclpy.time.Time or builtin_interfaces/Time, not {time!r}')


def _stamp_msg(ns):
    from builtin_interfaces.msg import Time as TimeMsg
    return TimeMsg(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _secs(ns):
    return f'{ns / 1e9:.6f}'


def _strip(frame):
    return frame[1:] if frame.startswith('/') else frame


class _Cache:
    """One child frame: (stamp ns, parent, T(parent <- child)) sorted by stamp, or one static entry."""

    __slots__ = ('static', 'stamps', 'entries')

    def __init__(self, static):
        self.static = static
        self.stamps = []
        self.entries = []

    def parent(self):
        return self.entries[-1][1] if self.entries else None

    def latest(self):
        return self.stamps[-1] if self.stamps else 0

    def get(self, ns):
        """T(parent <- child) at ns, or (None, error text)."""
        if self.static or ns == 0:
            return self.entries[-1][2], None
        stamps = self.stamps
        if len(stamps) == 1:
            if stamps[0] == ns:
                return self.entries[0][2], None
            return None, (f'Lookup would require extrapolation at time {_secs(ns)}, but only time '
                          f'{_secs(stamps[0])} is in the buffer')
        if ns > stamps[-1]:
            return None, (f'Lookup would require extrapolation into the future.  Requested time '
                          f'{_secs(ns)} but the latest data is at time {_secs(stamps[-1])}')
        if ns < stamps[0]:
            return None, (f'Lookup would require extrapolation into the past.  Requested time '
                          f'{_secs(ns)} but the earliest data is at time {_secs(stamps[0])}')
        i = bisect.bisect_left(stamps, ns)
        if stamps[i] == ns:
            return self.entries[i][2], None
        t0, p0, T0 = self.entries[i - 1]
        t1, p1, T1 = self.entries[i]
        if p0 != p1:
            return T0, None
        s = (ns - t0) / (t1 - t0)
        tr = tuple(a + s * (b - a) for a, b in zip(T0[0], T1[0]))
        return (tr, _slerp(T0[1], T1[1], s)), None


class Buffer:
    def __init__(self, cache_time=None, node=None):
        self.cache_time = cache_time if cache_time is not None else Duration(seconds=10.0)
        self._cache_ns = int(self.cache_time.nanoseconds)
        self._frames = {}          # child -> _Cache
        self._known = set()        # every frame seen as a child or a parent
        self._node = node

    # ------------------------------------------------------------ insert
    def clear(self):
        self._frames = {f: c for f, c in self._frames.items() if c.static}

    def _warn(self, text):
        from fakeros._logging import get_logger
        get_logger('tf2_buffer').warning(text)

    def _insert(self, transform, authority, static):
        parent = _strip(transform.header.frame_id)
        child = _strip(transform.child_frame_id)
        if not child:
            self._warn(f'Ignoring transform from authority "{authority}" because child_frame_id '
                       'not set')
            return False
        if not parent:
            self._warn(f'Ignoring transform with child_frame_id "{child}" from authority '
                       f'"{authority}" because frame_id not set')
            return False
        if child == parent:
            self._warn(f'TF_SELF_TRANSFORM: Ignoring transform from authority "{authority}" with '
                       f'frame_id and child_frame_id  "{child}" because they are the same')
            return False
        tr, r = transform.transform.translation, transform.transform.rotation
        vals = (tr.x, tr.y, tr.z, r.x, r.y, r.z, r.w)
        if any(math.isnan(v) for v in vals):
            self._warn(f'TF_NAN_INPUT: Ignoring transform for child_frame_id "{child}" from '
                       f'authority "{authority}" because of a nan value in the transform')
            return False
        T = ((tr.x, tr.y, tr.z), (r.w, r.x, r.y, r.z))
        ns = _ns(transform.header.stamp)
        cache = self._frames.get(child)
        if cache is None:
            cache = self._frames[child] = _Cache(static)
        self._known.add(child)
        self._known.add(parent)
        if cache.static:
            cache.stamps = [ns]
            cache.entries = [(ns, parent, T)]
            return True
        if cache.stamps:
            latest = cache.stamps[-1]
            if ns + self._cache_ns < latest:
                self._warn(f'TF_OLD_DATA ignoring data from the past for frame {child} at time '
                           f'{_secs(ns)} according to authority {authority}')
                return False
            cut = bisect.bisect_left(cache.stamps, latest - self._cache_ns)
            if cut:
                del cache.stamps[:cut]
                del cache.entries[:cut]
        i = bisect.bisect_left(cache.stamps, ns)
        if i < len(cache.stamps) and cache.stamps[i] == ns:
            cache.entries[i] = (ns, parent, T)
        else:
            cache.stamps.insert(i, ns)
            cache.entries.insert(i, (ns, parent, T))
        return True

    def set_transform(self, transform, authority):
        return self._insert(transform, authority, False)

    def set_transform_static(self, transform, authority):
        return self._insert(transform, authority, True)

    # ------------------------------------------------------------ lookup
    def _path_up(self, frame):
        """frame, its parent, ... up to the root (latest parents)."""
        out = [frame]
        seen = {frame}
        while True:
            cache = self._frames.get(out[-1])
            p = cache.parent() if cache is not None else None
            if p is None or p in seen:
                return out
            out.append(p)
            seen.add(p)

    def _resolve(self, target, source, ns):
        """T(target <- source) and the time used, or raise."""
        target, source = _strip(target), _strip(source)
        suffix = f', when looking up transform from frame [{source}] to frame [{target}]'
        if target == source:
            # Humble: Time() gives the frame's own newest stamp if it is a dynamic child, else 0
            cache = self._frames.get(target)
            if ns == 0 and cache is not None and not cache.static:
                ns = cache.latest()
            return _IDENTITY, ns
        if target not in self._known:
            raise LookupException(f'"{target}" passed to lookupTransform argument target_frame does '
                                  'not exist. ')
        if source not in self._known:
            raise LookupException(f'"{source}" passed to lookupTransform argument source_frame does '
                                  'not exist. ')
        up_s = self._path_up(source)
        up_t = self._path_up(target)
        in_t = {f: i for i, f in enumerate(up_t)}
        common = next((i for i, f in enumerate(up_s) if f in in_t), None)
        if common is None:
            raise ConnectivityException(
                f"Could not find a connection between '{target}' and '{source}' because they are "
                'not part of the same tree.Tf has two or more unconnected trees.')
        chain_s = up_s[:common]                         # source side children below the ancestor
        chain_t = up_t[:in_t[up_s[common]]]             # target side children below the ancestor
        links = [self._frames[f] for f in chain_s + chain_t]
        if ns == 0:
            dyn = [c.latest() for c in links if not c.static]
            ns = min(dyn) if dyn else 0

        def down(chain):
            T = _IDENTITY
            for f in reversed(chain):                   # from the ancestor down to the frame
                E, err = self._frames[f].get(ns)
                if err is not None:
                    raise ExtrapolationException(err + suffix)
                T = _compose(T, E)
            return T

        T_a_s = down(chain_s)
        T_a_t = down(chain_t)
        return _compose(_inv(T_a_t), T_a_s), ns

    @staticmethod
    def _msg(target, source, T, ns):
        out = TransformStamped()
        out.header.frame_id = _strip(target)
        out.header.stamp = _stamp_msg(ns)
        out.child_frame_id = _strip(source)
        (x, y, z), (qw, qx, qy, qz) = T
        tr, r = out.transform.translation, out.transform.rotation
        tr.x, tr.y, tr.z = float(x), float(y), float(z)
        r.x, r.y, r.z, r.w = float(qx), float(qy), float(qz), float(qw)
        return out

    def lookup_transform(self, target_frame, source_frame, time, timeout=None):
        T, ns = self._resolve(target_frame, source_frame, _ns(time))
        return self._msg(target_frame, source_frame, T, ns)

    def lookup_transform_full(self, target_frame, target_time, source_frame, source_time,
                              fixed_frame, timeout=None):
        T_f_t, _ = self._resolve(fixed_frame, target_frame, _ns(target_time))
        T_f_s, ns = self._resolve(fixed_frame, source_frame, _ns(source_time))
        return self._msg(target_frame, source_frame, _compose(_inv(T_f_t), T_f_s), ns)

    def can_transform(self, target_frame, source_frame, time, timeout=None,
                      return_debug_tuple=False):
        try:
            self._resolve(target_frame, source_frame, _ns(time))
            ok, err = True, ''
        except TransformException as e:
            ok, err = False, str(e)
        return (ok, err) if return_debug_tuple else ok

    def can_transform_full(self, target_frame, target_time, source_frame, source_time,
                           fixed_frame, timeout=None, return_debug_tuple=False):
        try:
            self.lookup_transform_full(target_frame, target_time, source_frame, source_time,
                                       fixed_frame)
            ok, err = True, ''
        except TransformException as e:
            ok, err = False, str(e)
        return (ok, err) if return_debug_tuple else ok

    def all_frames_as_string(self):
        return ''.join(f'Frame {f} exists with parent {c.parent()}.\n'
                       for f, c in sorted(self._frames.items()))

    def all_frames_as_yaml(self):
        out = []
        for f, c in sorted(self._frames.items()):
            out.append(f'{f}: \n  parent: \'{c.parent()}\'\n  most_recent_transform: '
                       f'{_secs(c.latest())}\n')
        return ''.join(out)
