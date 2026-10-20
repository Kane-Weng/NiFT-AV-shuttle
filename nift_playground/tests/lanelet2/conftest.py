"""Put the Lanelet2 stand-in first on sys.path, so `import lanelet2` gets it even where ROS is sourced."""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
PY_DIR = os.path.normpath(os.path.join(HERE, '..', '..', 'py', 'fakelanelet2'))
if PY_DIR not in sys.path:
    sys.path.insert(0, PY_DIR)
if HERE not in sys.path:
    sys.path.insert(0, HERE)
if 'lanelet2' in sys.modules and not getattr(sys.modules['lanelet2'], 'IS_STAND_IN', False):
    for name in [m for m in sys.modules if m == 'lanelet2' or m.startswith('lanelet2.')]:
        del sys.modules[name]
