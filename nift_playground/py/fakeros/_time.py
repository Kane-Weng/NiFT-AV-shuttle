"""
Time, Duration and Clock as in Humble's rclpy.

File: fakeros/_time.py

Description:
  Same rules as rclpy (checked in nift_pub_deps:humble).
  - Time() is SYSTEM_TIME by default, a node's clock gives ROS_TIME. Subtracting or comparing
    times of different clock types raises TypeError, as on ROS.
  - seconds and nanoseconds are truncated with int(), not rounded. Negative times raise.
  - Time - Time is a Duration, Time +/- Duration is a Time. Time is not hashable.
  A ROS_TIME clock reads the fakeros world clock (sim time). SYSTEM_TIME and STEADY_TIME clocks
  read the wall clock, as they would on ROS, so a node that uses them is not deterministic.
"""
import enum
import time as _walltime

S_TO_NS = 1000 * 1000 * 1000


class ClockType(enum.IntEnum):
    UNINITIALIZED = 0
    ROS_TIME = 1
    SYSTEM_TIME = 2
    STEADY_TIME = 3


def _world():
    from ._world import world
    return world


class Duration:
    def __init__(self, *, seconds=0, nanoseconds=0):
        total = int(seconds * S_TO_NS)
        total += int(nanoseconds)
        if total >= 2 ** 63 or total < -2 ** 63:
            raise OverflowError('Total nanoseconds value is too large to store in C duration.')
        self._ns = total

    @property
    def nanoseconds(self):
        return self._ns

    def __repr__(self):
        return 'Duration(nanoseconds={0})'.format(self._ns)

    def __str__(self):
        if self._ns == Infinite._ns:
            return 'Infinite'
        return f'{self._ns} nanoseconds'

    def __eq__(self, other):
        if isinstance(other, Duration):
            return self._ns == other._ns
        raise TypeError("Can't compare duration with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Duration):
            return self._ns < other._ns
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Duration):
            return self._ns <= other._ns
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Duration):
            return self._ns > other._ns
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Duration):
            return self._ns >= other._ns
        return NotImplemented

    __hash__ = None

    def to_msg(self):
        from builtin_interfaces.msg import Duration as DurationMsg
        sec, nanosec = divmod(self._ns, S_TO_NS)
        return DurationMsg(sec=sec, nanosec=nanosec)

    @classmethod
    def from_msg(cls, msg):
        from builtin_interfaces.msg import Duration as DurationMsg
        if not isinstance(msg, DurationMsg):
            raise TypeError('Must pass a builtin_interfaces.msg.Duration object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec)


Infinite = Duration(nanoseconds=2 ** 63 - 1)       # RMW_DURATION_INFINITE


class Time:
    def __init__(self, *, seconds=0, nanoseconds=0, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if seconds < 0:
            raise ValueError('Seconds value must not be negative')
        if nanoseconds < 0:
            raise ValueError('Nanoseconds value must not be negative')
        total = int(seconds * S_TO_NS)
        total += int(nanoseconds)
        if total >= 2 ** 63:
            raise OverflowError('Total nanoseconds value is too large to store in C time point.')
        self._ns = total
        self._clock_type = clock_type

    @property
    def nanoseconds(self):
        return self._ns

    @property
    def clock_type(self):
        return self._clock_type

    def seconds_nanoseconds(self):
        return (self._ns // S_TO_NS, self._ns % S_TO_NS)

    def __repr__(self):
        return 'Time(nanoseconds={0}, clock_type={1})'.format(self._ns, self._clock_type.name)

    def __add__(self, other):
        if isinstance(other, Duration):
            try:
                return Time(nanoseconds=self._ns + other.nanoseconds, clock_type=self._clock_type)
            except OverflowError as e:
                raise OverflowError('Addition leads to overflow in C storage.') from e
        return NotImplemented

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        if isinstance(other, Time):
            if self._clock_type != other._clock_type:
                raise TypeError("Can't subtract times with different clock types")
            return Duration(nanoseconds=self._ns - other._ns)
        if isinstance(other, Duration):
            try:
                return Time(nanoseconds=self._ns - other.nanoseconds, clock_type=self._clock_type)
            except ValueError as e:
                raise ValueError('Subtraction leads to negative time.') from e
        return NotImplemented

    def _check(self, other):
        if self._clock_type != other._clock_type:
            raise TypeError("Can't compare times with different clock types")

    def __eq__(self, other):
        if isinstance(other, Time):
            self._check(other)
            return self._ns == other._ns
        raise TypeError("Can't compare time with object of type: ", type(other))

    def __ne__(self, other):
        return not self.__eq__(other)

    def __lt__(self, other):
        if isinstance(other, Time):
            self._check(other)
            return self._ns < other._ns
        return NotImplemented

    def __le__(self, other):
        if isinstance(other, Time):
            self._check(other)
            return self._ns <= other._ns
        return NotImplemented

    def __gt__(self, other):
        if isinstance(other, Time):
            self._check(other)
            return self._ns > other._ns
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, Time):
            self._check(other)
            return self._ns >= other._ns
        return NotImplemented

    __hash__ = None

    def to_msg(self):
        from builtin_interfaces.msg import Time as TimeMsg
        sec, nanosec = self.seconds_nanoseconds()
        return TimeMsg(sec=sec, nanosec=nanosec)

    @classmethod
    def from_msg(cls, msg, clock_type=ClockType.ROS_TIME):
        from builtin_interfaces.msg import Time as TimeMsg
        if not isinstance(msg, TimeMsg):
            raise TypeError('Must pass a builtin_interfaces.msg.Time object')
        return cls(seconds=msg.sec, nanoseconds=msg.nanosec, clock_type=clock_type)


class Clock:
    """ROS_TIME reads the world (sim) clock; SYSTEM_TIME and STEADY_TIME read the wall clock."""

    def __init__(self, *, clock_type=ClockType.SYSTEM_TIME):
        if not isinstance(clock_type, ClockType):
            raise TypeError('Clock type must be a ClockType enum')
        if clock_type is ClockType.UNINITIALIZED:
            raise ValueError('Cannot create a clock with an uninitialized clock type')
        self._clock_type = clock_type

    @property
    def clock_type(self):
        return self._clock_type

    def __repr__(self):
        return 'Clock(clock_type={0})'.format(self._clock_type.name)

    def now(self):
        if self._clock_type is ClockType.ROS_TIME:
            ns = _world().now_ns
        elif self._clock_type is ClockType.STEADY_TIME:
            ns = _walltime.monotonic_ns()
        else:
            ns = _walltime.time_ns()
        return Time(nanoseconds=ns, clock_type=self._clock_type)

    def create_jump_callback(self, threshold, *, pre_callback=None, post_callback=None):
        """Registered and never called: the world clock only moves forward, and reset rebuilds the nodes."""
        return JumpHandle(clock=self, threshold=threshold, pre_callback=pre_callback,
                          post_callback=post_callback)

    def sleep_until(self, until, context=None):
        raise RuntimeError('fakeros: nothing may sleep; the runner drives time')

    def sleep_for(self, rel_time, context=None):
        raise RuntimeError('fakeros: nothing may sleep; the runner drives time')


class ROSClock(Clock):
    """A node's clock. Always sim time here (use_sim_time is not consulted)."""

    def __init__(self):
        super().__init__(clock_type=ClockType.ROS_TIME)

    @property
    def ros_time_is_active(self):
        return True

    def _set_ros_time_is_active(self, enabled):
        pass


class JumpThreshold:
    def __init__(self, *, min_forward, min_backward, on_clock_change=True):
        self.min_forward = min_forward
        self.min_backward = min_backward
        self.on_clock_change = on_clock_change


class JumpHandle:
    def __init__(self, *, clock, threshold, pre_callback, post_callback):
        self._clock = clock
        self._threshold = threshold

    def unregister(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, t, v, tb):
        self.unregister()
