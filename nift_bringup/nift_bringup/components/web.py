"""
Launch component for the rosbridge server of the web frontend.

File: web.py

Description:
  Component `web`: rosbridge for nift_web_frontend (nift_web/web.launch.py, port 9090).

  Check
    rosbridge_server is installed and port 9090 is free (a rosbridge left running holds it).
"""

from ament_index_python.packages import get_package_prefix, PackageNotFoundError
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from .base import Component
from .common import FAIL, OK, port_free, share

PORT = 9090


class Web(Component):
    name = 'web'

    def check(self, plan, graph):
        try:
            get_package_prefix('rosbridge_server')
        except PackageNotFoundError:
            return [(FAIL,
                     'rosbridge_server not installed: '
                     'sudo apt install ros-humble-rosbridge-server')]
        if not port_free(PORT):
            return [(FAIL, f'port {PORT} in use (another rosbridge running?)')]
        return [(OK, f'rosbridge_server installed, port {PORT} free')]

    def actions(self, plan):
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(share('nift_web', 'launch', 'web.launch.py')))]
