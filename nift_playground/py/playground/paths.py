"""
Put the stand-ins and the repo packages on sys.path, in the order ARCH section 1 gives.

Natively `repo_root` is the checkout. In Pyodide it is `/nift`, where the bundle unpacks the same repo-relative paths. Never source ROS in a shell that runs the playground: the stand-ins shadow it only because they come first.
"""

import os
import sys

# repo-relative package roots, after the playground's own roots
REPO_PACKAGES = ('nift_common', 'nift_sensor', 'nift_waypoint', 'nift_control',
                 'nift_simulation/nift_carla', 'nift_bringup')
# nift_playground/py-relative roots, first on sys.path
PY_ROOTS = ('fakeros', 'fakelanelet2', 'fakesocketio', 'stubs', '.')

_state = {'root': None}


def default_root() -> str:
    """The checkout this file lives in (nift_playground/py/playground/paths.py)."""
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__)))))


def repo_root() -> str:
    """The root given to setup(), else the checkout this file lives in."""
    return _state['root'] or default_root()


def repo_path(*rel) -> str:
    return os.path.join(repo_root(), *rel)


def setup(root=None) -> str:
    """Put the paths first on sys.path (once) and install fakeros. Return the repo root."""
    root = os.path.abspath(root or default_root())
    if _state['root'] == root:
        return root
    py = os.path.join(root, 'nift_playground', 'py')
    wanted = [os.path.normpath(os.path.join(py, p)) for p in PY_ROOTS]
    wanted += [os.path.join(root, p) for p in REPO_PACKAGES]
    for p in reversed(wanted):
        while p in sys.path:
            sys.path.remove(p)
        sys.path.insert(0, p)
    _state['root'] = root
    import fakeros
    fakeros.install()
    return root
