"""rclpy.utilities: what scripts import; ok / shutdown follow the world's running flag."""
from fakeros._world import world


def get_default_context(*, shutting_down=False):
    return None


def ok(*, context=None):
    return world.running


def shutdown(*, context=None):
    world.running = False


def try_shutdown(*, context=None):
    world.running = False


def remove_ros_args(args=None):
    """argv without the --ros-args ... [--] section, as rclpy does."""
    import sys
    args = sys.argv if args is None else args
    out, inside = [], False
    for a in args:
        if a == '--ros-args':
            inside = True
        elif inside and a == '--':
            inside = False
        elif not inside:
            out.append(a)
    return out


def get_rmw_implementation_identifier():
    return 'rmw_fakeros'


def timeout_sec_to_nsec(timeout_sec):
    if timeout_sec is None or timeout_sec < 0:
        return -1
    return int(float(timeout_sec) * 1e9)
