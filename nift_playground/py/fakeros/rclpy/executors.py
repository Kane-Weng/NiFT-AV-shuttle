"""rclpy.executors: the names import; spinning is the runner's job (world.step)."""
from fakeros._world import world


class ExternalShutdownException(Exception):
    pass


class ShutdownException(Exception):
    pass


class TimeoutException(Exception):
    pass


class Executor:
    def __init__(self, *, context=None):
        self._nodes = []

    def add_node(self, node):
        if node in self._nodes:
            return False
        self._nodes.append(node)
        return True

    def remove_node(self, node):
        if node in self._nodes:
            self._nodes.remove(node)

    def get_nodes(self):
        return list(self._nodes)

    def spin(self):
        raise RuntimeError('fakeros: the runner drives time (world.step), not Executor.spin')

    def spin_once(self, timeout_sec=None):
        world.drain()

    def spin_until_future_complete(self, future, timeout_sec=None):
        world.drain()

    def spin_once_until_future_complete(self, future, timeout_sec=None):
        world.drain()

    def shutdown(self, timeout_sec=None):
        return True


class SingleThreadedExecutor(Executor):
    pass


class MultiThreadedExecutor(Executor):
    def __init__(self, num_threads=None, *, context=None):
        super().__init__(context=context)
