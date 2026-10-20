"""
Desktop window to pick, check and start a system.launch.py profile.

File: launcher.py

Description:
  Desktop launcher for system.launch.py: pick the components on a transit-style map, check them,
  start the launch. The terminal it was started from becomes the launch (os.execvp), so Ctrl-C and
  the log work as with `ros2 launch`.

  Map (right)
    Bottom row = inputs, each row up needs the ones below; lines are REQUIRES, colored when their
    prerequisite is on. bag / rviz / web need nothing and sit apart.
      solid      on                       pulsing   off, click switches it on alone
      gray       off, needs something first: a click says what and points at it
    Click an on node: off, with everything that needed it. Double-click: on, with everything it
    needs. localization and filter both give map -> base_link: the filter's subtitle says the mode
    (ekf alone, shadow beside localization), as does the side panel.
  Side panel (left)
    preset (config/profiles.yaml), mode Real vehicle / CARLA (sim) / Kinematic (sim: no GPU, no
    LiDAR), advanced launch arguments, warnings, the command.
    Check runs nift_bringup/preflight.py in the background; Start asks first if the selection was
    not checked or the check FAILED, and then starts with checks:=false.
  The rules are profiles.py's, asked through launcher_model.py (no Qt there; it is unit-tested).

Usage:
  ros2 run nift_bringup launcher [--profile <name>] [--dry-run]
    --profile  preset to open with (default none)
    --dry-run  Start adds dry_run:=true: the launch prints the plan and the checks, starts nothing
"""

import argparse
import html
import math
import os
import shlex
import signal
import sys
import threading
import time

from python_qt_binding.QtCore import QObject, QPointF, QRectF, Qt, QTimer, Signal
from python_qt_binding.QtGui import (
    QColor,
    QFont,
    QGuiApplication,
    QPainter,
    QPainterPath,
    QPalette,
    QPen,
)
from python_qt_binding.QtWidgets import (
    QApplication,
    QButtonGroup,
    QComboBox,
    QFormLayout,
    QFrame,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QToolButton,
    QToolTip,
    QVBoxLayout,
    QWidget,
)

from . import preflight
from .components.common import Level, share
from .launcher_model import (
    alternatives,
    LauncherModel,
    localization,
    NodeState,
    rule_errors,
    safety_filter,
)
from .profiles import (
    AFTER_AUTH,
    ARGS,
    COMPONENTS,
    DEFAULT_MAPS,
    installed_maps,
    load,
    localization_mode,
    MAP_TF,
    ProfileError,
    REQUIRES,
    SIM_ONLY,
)

# ===================================
#            map layout
# ===================================
# Grid units, x to the right, y up (row 0 = inputs). Every component needs a place
# (test_launcher_model.py checks it).
POSITIONS = {
    'perception': (-2.0, 0.0),
    'can': (0.0, 0.0),
    'gnss': (2.0, 0.0),
    'longitudinal': (0.0, 2.0),
    'localization': (2.0, 2.0),
    'filter': (3.0, 2.0),
    'planning': (3.0, 3.0),
    'static_path': (5.0, 3.0),
    'lateral': (2.0, 4.0),
    'bag': (5.6, 1.5),
    'rviz': (5.6, 0.75),
    'web': (5.6, 0.0),
}
TOOLS = ('bag', 'rviz', 'web')                  # no prerequisites: drawn in their own box
LABEL_SIDE = {'longitudinal': 'left', 'localization': 'left', 'filter': 'right',
              'planning': 'right',
              'static_path': 'right', 'lateral': 'above', 'bag': 'right', 'rviz': 'right',
              'web': 'right'}       # default: below
SUBTITLE = {'can': 'CAN bridge', 'gnss': 'beacon_node', 'localization': 'shuttle_tf_node',
            'filter': 'localization_node', 'perception': 'lidar_node', 'planning': 'waypoint_node',
            'static_path': 'scenario waypoints', 'longitudinal': 'ADRC speed',
            'lateral': 'Pure Pursuit',
            'bag': 'ros2 bag record', 'rviz': 'navigation.rviz', 'web': 'rosbridge :9090'}

# Where the alternatives of an "any one of" requirement meet before going on to the node. planning
# and lateral
# share the map -> base_link junction (localization or filter).
JUNCTIONS = {('lateral', ('planning', 'static_path')): (3.0, 4.0),
             ('planning', MAP_TF): (2.0, 3.0),
             ('lateral', MAP_TF): (2.0, 3.0)}
JUNCTION_LABEL = {(2.0, 3.0): 'left'}           # where its "or" goes; default: above
# Bends of a line, by (from, to); without an entry a line is straight.
BENDS = {('static_path', 'lateral'): [(4.0, 4.0)], ('gnss', 'filter'): [(3.0, 1.0)]}
# Not a requirement, drawn dashed: what perception gives longitudinal.
FEEDS = [('perception', 'longitudinal', 'obstacles')]

# ===================================
#            colors
# ===================================
BG, CANVAS, CARD, FG, DIM, GREY, GREY_LINE = (
    '#1e1e1e', '#16181b', '#2a2d31', '#e8e8e8', '#8a8f96', '#4a4f55', '#3a3e44')
RED, BLUE, SKY, TEAL, PURPLE, GREEN, GOLD, AMBER = (
    '#e5534b', '#3d8fd6', '#7cc4fa', '#27b39b', '#9c6ade', '#45b36b', '#d4a72c', '#f0a030')
COLOR = {'can': RED, 'longitudinal': RED, 'gnss': BLUE, 'localization': BLUE, 'filter': SKY,
         'planning': BLUE,
         'static_path': TEAL, 'lateral': PURPLE, 'perception': GREEN, 'bag': GOLD, 'rviz': GOLD,
         'web': GOLD}
LEVEL_COLOR = {Level.OK: '#3fb950', Level.WARN: AMBER, Level.FAIL: '#f85149',
               Level.SKIP: '#6e7681',
               Level.NONE: '#6e7681'}
LEVEL_RANK = [Level.FAIL, Level.WARN, Level.OK, Level.SKIP, Level.NONE]     # worst first

# The map is drawn on a DESIGN_W x DESIGN_H canvas, then scaled to the widget as a whole, so labels
# never
# collide in a small window. MARGINS (left, top, right, bottom) keep room for labels, the mode
# badge and the legend.
DESIGN_W, DESIGN_H, MARGINS, MAX_ZOOM = 1000.0, 720.0, (70, 100, 180, 150), 1.4
LINE_W, NODE_R, BIG_R = 9.0, 15.0, 19.0         # design pixels
PULSE_S, HIGHLIGHT_S = 1.4, 2.2
# Side panel, in units of the UI font's line height (so it grows with the font): width, and the
# height the
# preflight list may take before it scrolls. PANEL_PAD is in pixels.
PANEL_EM, RESULTS_LINES, PANEL_PAD = 23.5, 10, 22


def color(name, alpha=255):
    c = QColor(COLOR.get(name, GREY))
    c.setAlpha(int(alpha))
    return c


def lines():
    """
    Return [(from, to, points in grid units)] from REQUIRES.

    An "any one of" goes through its junction.
    """
    out = []
    for dst, needs in REQUIRES.items():
        for need in needs:
            junction = JUNCTIONS.get((dst, need)) if isinstance(need, tuple) else None
            for src in alternatives(need):
                end = [junction] if junction else [POSITIONS[dst]]
                out.append((src, dst, [POSITIONS[src], *BENDS.get((src, dst), []), *end]))
    return out


def grid_to_design():
    """
    Map POSITIONS grid units to design pixels.

    The map is centered between MARGINS, as large as fits.
    """
    xs, ys = [p[0] for p in POSITIONS.values()], [p[1] for p in POSITIONS.values()]
    left, top, right, bottom = MARGINS
    w, h = DESIGN_W - left - right, DESIGN_H - top - bottom
    unit = min(w / (max(xs) - min(xs)), h / (max(ys) - min(ys)))
    x0 = left + (w - unit * (max(xs) - min(xs))) / 2 - unit * min(xs)
    y0 = top + (h - unit * (max(ys) - min(ys))) / 2 + unit * max(ys)
    return lambda p: QPointF(x0 + unit * p[0], y0 - unit * p[1])


def worst(levels):
    return min(levels, key=LEVEL_RANK.index) if levels else None


# ===================================
#            map
# ===================================
class MapView(QWidget):
    """
    The node map.

    Shows the model's state; clicks go out as signals, the window changes the model.
    """

    clicked = Signal(str)
    double_clicked = Signal(str)

    def __init__(self, model):
        super().__init__()
        self.model = model
        self.to_px = grid_to_design()
        self.lines = lines()
        ends = [name for src, dst, _ in self.lines for name in (src, dst)]
        # interchanges
        self.radius = {c: BIG_R if ends.count(c) > 2 else NODE_R for c in COMPONENTS}
        self.states = model.states()
        self.levels = {}                    # component -> worst preflight Level
        self.highlight, self.highlight_until = set(), 0.0
        self.pressed = None                 # node under the last left press
        self.setMouseTracking(True)
        self.setMinimumSize(500, 360)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(40)

    def refresh(self, levels):
        self.states, self.levels = self.model.states(), levels
        self.update()

    def point_at(self, names):
        self.highlight, self.highlight_until = set(names), time.monotonic() + HIGHLIGHT_S
        self.update()

    # ---------------------------------------------------------------- geometry
    def _view(self):
        """(zoom, dx, dy): design canvas -> widget, centered."""
        k = min(self.width() / DESIGN_W, self.height() / DESIGN_H, MAX_ZOOM)
        return k, (self.width() - k * DESIGN_W) / 2, (self.height() - k * DESIGN_H) / 2

    def node_at(self, pos):
        k, dx, dy = self._view()
        x, y = (pos.x() - dx) / k, (pos.y() - dy) / k
        for name, p in POSITIONS.items():
            c = self.to_px(p)
            if math.hypot(x - c.x(), y - c.y()) <= self.radius[name] + 8:
                return name
        return None

    # ---------------------------------------------------------------- input
    def mousePressEvent(self, e):
        self.pressed = self.node_at(e.pos()) if e.button() == Qt.LeftButton else None
        if self.pressed:
            self.clicked.emit(self.pressed)

    # comes instead of the second press of a double-click
    def mouseDoubleClickEvent(self, e):
        name = self.node_at(e.pos())
        # both presses on this node
        if name and e.button() == Qt.LeftButton and name == self.pressed:
            self.double_clicked.emit(name)

    def mouseMoveEvent(self, e):
        self.setCursor(Qt.PointingHandCursor if self.node_at(e.pos()) else Qt.ArrowCursor)

    def event(self, e):
        if e.type() == e.ToolTip:
            name = self.node_at(e.pos())
            QToolTip.showText(e.globalPos(), self.tooltip(name)) if name else QToolTip.hideText()
            return True
        return super().event(e)

    def tooltip(self, name):
        state = self.states[name]
        rows = [f'<b>{name}</b>', html.escape(COMPONENTS[name])]
        if name in AFTER_AUTH:
            rows.append('<i>starts after beacon auth_ok</i>')
        if state == NodeState.AVAILABLE:
            rows.append('click: switch on')
        elif state == NodeState.ON:
            rows.append('click: switch off, with what needs it')
        elif state == NodeState.BLOCKED:
            rows.append(f'<span style="color:{AMBER}">{html.escape(self.model.hint(name))}</span>')
        return '<br>'.join(rows)

    # ---------------------------------------------------------------- painting
    def _segments(self):
        """
        Return [(from, to, points, color; None = gray)].

        Every line, and each junction's trunk (from = None).
        """
        on = self.model.on
        out = [(src, dst, pts, color(src) if src in on else None) for src, dst, pts in self.lines]
        for (dst, alts), junction in JUNCTIONS.items():
            lit = [a for a in alts if a in on]
            out.append((None, dst, [junction, POSITIONS[dst]], color(lit[0]) if lit else None))
        return out

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(CANVAS))
        k, dx, dy = self._view()
        p.translate(dx, dy)
        p.scale(k, k)
        to_px = self.to_px
        now = time.monotonic()
        pulse = 0.5 - 0.5 * math.cos(2 * math.pi * now / PULSE_S)          # 0 -> 1 -> 0, smooth
        flash = min(1.0, max(0.0, self.highlight_until - now) / HIGHLIGHT_S)   # 1 -> 0
        on = self.model.on

        self._tools_box(p, to_px)
        segments = self._segments()
        for src, dst, pts, col in sorted(segments, key=lambda s: s[3] is not None):    # gray below
            self._stroke(p, [to_px(q) for q in pts], col or QColor(GREY_LINE), LINE_W)
        for src, dst, label in FEEDS:
            self._feed(p, to_px(POSITIONS[src]), to_px(POSITIONS[dst]), label, src in on)

        if flash > 0:       # a blocked node was clicked: its lines from what it needs flash amber
            amber = QColor(AMBER)
            amber.setAlphaF(min(1.0, flash * (0.55 + 0.45 * pulse)))
            for src, dst, pts, _ in segments:
                if dst in self.highlight and (src is None or src in self.highlight or src in on):
                    self._stroke(p, [to_px(q) for q in pts], amber, LINE_W)

        # a shared one once
        for junction, alts in {j: alts for (_, alts), j in JUNCTIONS.items()}.items():
            self._junction(p, to_px(junction), lit=bool(on & set(alts)),
                           side=JUNCTION_LABEL.get(junction, 'above'))
        for name in COMPONENTS:
            self._node(p, name, to_px(POSITIONS[name]), pulse, flash)
        self._legend(p)
        self._mode_badge(p)
        p.end()

    def _stroke(self, p, pts, col, width):
        path = QPainterPath(pts[0])
        r = 22.0                            # rounded bends, like a transit map
        for i in range(1, len(pts) - 1):
            a, b, c = pts[i - 1], pts[i], pts[i + 1]
            d1, d2 = b - a, c - b
            l1, l2 = math.hypot(d1.x(), d1.y()), math.hypot(d2.x(), d2.y())
            k = min(r, l1 / 2, l2 / 2)
            path.lineTo(b - d1 * (k / l1))
            path.quadTo(b, b + d2 * (k / l2))
        path.lineTo(pts[-1])
        p.setPen(QPen(col, width, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.setBrush(Qt.NoBrush)
        p.drawPath(path)

    def _feed(self, p, a, b, label, lit):
        pen = QPen(color('perception') if lit else QColor(GREY_LINE), 3.0, Qt.DashLine,
                   Qt.RoundCap)
        pen.setDashPattern([2.5, 2.5])
        p.setPen(pen)
        p.drawLine(a, b)
        mid = (a + b) / 2
        p.save()
        p.translate(mid)
        p.rotate(math.degrees(math.atan2(b.y() - a.y(), b.x() - a.x())))
        p.setFont(self._font(11))
        p.setPen(QColor(DIM))
        p.drawText(QRectF(-60, -20, 120, 16), Qt.AlignCenter, label)
        p.restore()

    def _junction(self, p, c, lit, side='above'):
        p.setPen(QPen(QColor(FG if lit else GREY), 2.5))
        p.setBrush(QColor(CANVAS))
        p.drawEllipse(c, 7, 7)
        p.setFont(self._font(12, italic=True))
        p.setPen(QColor(DIM))
        rect = (QRectF(c.x() - 52, c.y() - 9, 40, 18) if side == 'left'
                else QRectF(c.x() - 20, c.y() - 30, 40, 18))
        p.drawText(rect, Qt.AlignCenter, 'or')

    def _node(self, p, name, c, pulse, flash):
        state, r = self.states[name], self.radius[name]
        if state in (NodeState.ON, NodeState.LOCKED):
            p.setPen(QPen(QColor('#ffffff'), 3.0))
            p.setBrush(color(name))
        elif state == NodeState.AVAILABLE:
            p.setPen(Qt.NoPen)
            p.setBrush(color(name, 90 * pulse))                  # soft halo
            p.drawEllipse(c, r + 4 + 6 * pulse, r + 4 + 6 * pulse)
            p.setPen(QPen(color(name, 140 + 115 * pulse), 4.0))
            p.setBrush(color(name, 40 + 60 * pulse))
        else:
            p.setPen(QPen(QColor(GREY), 3.5))
            p.setBrush(QColor(CARD))
        p.drawEllipse(c, r, r)

        if state == NodeState.LOCKED:
            self._lock(p, c)
        if flash > 0 and name in self.highlight:
            hl = QColor(AMBER)
            hl.setAlphaF(flash)
            p.setPen(QPen(hl, 3.0, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(c, r + 7, r + 7)

        self._label(p, name, c, r, state)
        if name in self.levels:
            self._badge(p, c, r, self.levels[name], left=LABEL_SIDE.get(name) == 'right')

    def _label(self, p, name, c, r, state):
        title, sub = name, SUBTITLE.get(name, '')
        if name == 'can' and self.model.sim:
            title, sub = self.model.vehicle, 'can · always on in sim'
        # ekf (alone) | shadow (beside localization)
        if name == 'filter' and name in self.model.on:
            sub = f'{sub} · {localization_mode(self.model.on)}'
        tags = ['sim only'] if name in SIM_ONLY and not self.model.sim else []
        dim = state == NodeState.BLOCKED
        side = LABEL_SIDE.get(name, 'below')
        w, gap = 170, r + 10
        rects = {'below': QRectF(c.x() - w / 2, c.y() + gap - 4, w, 40),
                 'above': QRectF(c.x() - w / 2, c.y() - gap - 38, w, 40),
                 'left': QRectF(c.x() - gap - w, c.y() - 19, w, 40),
                 'right': QRectF(c.x() + gap, c.y() - 19, w, 40)}
        align = {'below': Qt.AlignHCenter, 'above': Qt.AlignHCenter, 'left': Qt.AlignRight,
                 'right': Qt.AlignLeft}[side]
        rect = rects[side]
        p.setFont(self._font(15, bold=True))
        p.setPen(QColor(DIM if dim else FG))
        p.drawText(QRectF(rect.x(), rect.y(), rect.width(), 22), align | Qt.AlignVCenter, title)
        p.setFont(self._font(12))
        p.setPen(QColor(GREY if dim else DIM))
        p.drawText(QRectF(rect.x(), rect.y() + 20, rect.width(), 18), align | Qt.AlignVCenter, sub)
        for tag in tags:
            self._tag(p, c, r, tag)

    def _tag(self, p, c, r, text):
        p.setFont(self._font(11, bold=True))
        w = p.fontMetrics().horizontalAdvance(text) + 12
        rect = QRectF(c.x() - w / 2, c.y() + r + 6, w, 16)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(GREY))
        p.drawRoundedRect(rect, 8, 8)
        p.setPen(QColor(FG))
        p.drawText(rect, Qt.AlignCenter, text)

    def _lock(self, p, c):
        p.setPen(QPen(QColor('#ffffff'), 1.8))
        p.setBrush(Qt.NoBrush)
        p.drawArc(QRectF(c.x() - 4, c.y() - 8, 8, 9), 0, 180 * 16)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor('#ffffff'))
        p.drawRoundedRect(QRectF(c.x() - 6, c.y() - 3, 12, 9), 2, 2)

    def _badge(self, p, c, r, level, left):
        """Preflight result pill at the node's upper corner, on the side away from its label."""
        p.setFont(self._font(11, bold=True))
        text = level.value
        w = p.fontMetrics().horizontalAdvance(text) + 10
        rect = QRectF(c.x() - r * 0.4 - w if left else c.x() + r * 0.4, c.y() - r - 6, w, 16)
        p.setPen(QPen(QColor(CANVAS), 2))
        p.setBrush(QColor(LEVEL_COLOR[level]))
        p.drawRoundedRect(rect, 8, 8)
        p.setPen(QColor('#ffffff') if level != Level.WARN else QColor('#1e1e1e'))
        p.drawText(rect, Qt.AlignCenter, text)

    def _tools_box(self, p, to_px):
        pts = [to_px(POSITIONS[t]) for t in TOOLS]
        xs, ys = [q.x() for q in pts], [q.y() for q in pts]
        heading = 'TOOLS · no prerequisites'
        p.setFont(self._font(12, bold=True))
        w = max(max(xs) - min(xs) + 175, p.fontMetrics().horizontalAdvance(heading) + 26)
        rect = QRectF(min(xs) - 34, min(ys) - 52, w, max(ys) - min(ys) + 84)
        p.setPen(QPen(QColor(GREY_LINE), 1.5, Qt.DashLine))
        p.setBrush(Qt.NoBrush)
        p.drawRoundedRect(rect, 14, 14)
        p.setPen(QColor(DIM))
        p.drawText(QRectF(rect.x() + 13, rect.y() + 7, w - 20, 18), Qt.AlignLeft, heading)

    def _legend(self, p):
        x, y = 18, DESIGN_H - 34
        p.setFont(self._font(12))
        items = [('on', 'on'), ('pulse', 'click: on'), ('blocked', 'needs something first')]
        for kind, text in items:
            c = QPointF(x + 8, y)
            if kind == 'on':
                p.setPen(QPen(QColor('#ffffff'), 2))
                p.setBrush(QColor(BLUE))
            elif kind == 'pulse':
                p.setPen(QPen(QColor(BLUE), 3))
                p.setBrush(color('gnss', 70))
            else:
                p.setPen(QPen(QColor(GREY), 3))
                p.setBrush(QColor(CARD))
            p.drawEllipse(c, 7, 7)
            p.setPen(QColor(DIM))
            w = p.fontMetrics().horizontalAdvance(text)
            p.drawText(QRectF(x + 22, y - 9, w + 4, 18), Qt.AlignVCenter, text)
            x += w + 46
        p.drawText(QRectF(x, y - 9, 400, 18), Qt.AlignVCenter,
                   'double-click: on, with what it needs')
        y -= 26
        x = 18
        for col, text in ((RED, 'vehicle: can → longitudinal → lateral'),
                          (BLUE, 'position: gnss → localization or filter → planning → lateral')):
            p.setPen(QPen(QColor(col), 5, Qt.SolidLine, Qt.RoundCap))
            p.drawLine(QPointF(x, y), QPointF(x + 22, y))
            p.setPen(QColor(DIM))
            w = p.fontMetrics().horizontalAdvance(text)
            p.drawText(QRectF(x + 30, y - 9, w + 4, 18), Qt.AlignVCenter, text)
            x += w + 64

    def _mode_badge(self, p):
        text, col = (('REAL VEHICLE', '#c0392b') if not self.model.sim else
                     ('SIM · KINEMATIC', '#1f7a6c') if self.model.kinematic else ('SIM · CARLA',
                                                                                  '#1f7a6c'))
        p.setFont(self._font(15, bold=True))
        w = p.fontMetrics().horizontalAdvance(text) + 28
        rect = QRectF(DESIGN_W - w - 16, 14, w, 28)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(col))
        p.drawRoundedRect(rect, 14, 14)
        p.setPen(QColor('#ffffff'))
        p.drawText(rect, Qt.AlignCenter, text)

    def _font(self, px, bold=False, italic=False):
        """
        Return a font for map text in design pixels.

        So it scales with the map whatever the screen's DPI or font size.
        """
        f = QFont(self.font())
        f.setPixelSize(px)
        f.setBold(bold)
        f.setItalic(italic)
        return f


# ===================================
#            preflight thread
# ===================================
class PreflightRunner(QObject):
    """preflight.run() off the UI thread; `finished` arrives on the UI thread (queued signal)."""

    # plan and command it checked, results or the exception, seconds
    finished = Signal(object, object, object, float)

    def start(self, plan, command):
        threading.Thread(target=self._run, args=(plan, command), daemon=True).start()

    def _run(self, plan, command):
        t0 = time.monotonic()
        try:
            results = preflight.run(plan)
        except Exception as e:      # shown in the panel, never lost
            results = e
        self.finished.emit(plan, command, results, time.monotonic() - t0)


# ===================================
#            window
# ===================================
STYLE = f"""
QWidget {{ background: {BG}; }}
QLabel#title {{ font-size: 21pt; font-weight: bold; }}
QLabel#section {{ color: {DIM}; font-size: 8.5pt; font-weight: bold; letter-spacing: 1px; margin-top: 8px; }}
QLabel#dim {{ color: {DIM}; }}
QLineEdit {{ background: {CARD}; border: 1px solid #3a3e44; border-radius: 6px; padding: 4px 6px; }}
QLabel#command {{ background: {CARD}; border: 1px solid #3a3e44; border-radius: 6px; padding: 8px; }}
QComboBox {{ padding: 5px 8px; }}
QPushButton {{ background: {CARD}; border: 1px solid #3a3e44; border-radius: 6px; padding: 8px 12px; }}
QPushButton:hover {{ border-color: {DIM}; }}
QPushButton:disabled {{ color: #5c6168; background: #232528; }}
QPushButton#mode_sim:checked {{ background: {BLUE}; border-color: {BLUE}; color: white; font-weight: bold; }}
QPushButton#mode_real:checked {{ background: #c0392b; border-color: #c0392b; color: white; font-weight: bold; }}
QToolButton {{ border: none; color: {DIM}; font-weight: bold; padding: 4px 0; }}
QScrollArea {{ border: none; }}
QScrollBar:vertical {{ background: {BG}; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{ background: #3a3e44; border-radius: 5px; min-height: 30px; }}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
QFrame#panel {{ border-right: 1px solid #2c2f33; }}
QToolTip {{ background: {CARD}; color: {FG}; border: 1px solid #3a3e44; padding: 6px; }}
"""  # noqa: E501


def dark_palette():
    palette = QPalette()
    for role, value in ((QPalette.Window, BG), (QPalette.WindowText, FG), (QPalette.Base, CARD),
                        (QPalette.AlternateBase, BG), (QPalette.Text, FG), (QPalette.Button, CARD),
                        (QPalette.ButtonText, FG), (QPalette.Highlight,
                                                    BLUE), (QPalette.HighlightedText, '#ffffff'),
                        (QPalette.ToolTipBase, CARD), (QPalette.ToolTipText,
                                                       FG), (QPalette.PlaceholderText, GREY)):
        palette.setColor(role, QColor(value))
    return palette


class LauncherWindow(QMainWindow):

    def __init__(self, profiles, preset='none', dry_run=False):
        super().__init__()
        self.setWindowTitle('NiFT Launcher')
        self.model = LauncherModel(profiles, preset)
        self.dry_run = dry_run
        self.results, self.results_for, self.results_s = None, None, 0.0     # last preflight
        # the plan it checked (row labels)
        self.results_plan = None
        self.checking = False
        # set by Start; main() execs it once the window is closed
        self.start_command = None
        self.runner = PreflightRunner()
        self.runner.finished.connect(self._checked)

        self.map = MapView(self.model)
        self.map.clicked.connect(self._click)
        self.map.double_clicked.connect(self._double_click)
        self._before_click = set()
        self.status = QLabel()
        self.status.setWordWrap(True)
        right = QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(0)
        right.addWidget(self.map, 1)
        right.addWidget(self.status)

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(self._panel())
        body.addLayout(right, 1)
        central = QWidget()
        central.setLayout(body)
        self.setCentralWidget(central)
        # app-wide: a widget palette does not mix with style sheets
        QApplication.setPalette(dark_palette())
        self.setStyleSheet(STYLE)
        self._sync_controls()
        self._refresh()
        avail = QApplication.primaryScreen().availableGeometry()       # logical pixels
        self.resize(min(self.panel.width() + int(DESIGN_W), int(0.95 * avail.width())),
                    min(820, int(0.9 * avail.height())))
        self._say('Click a station to switch it on or off. '
                  'A gray one needs something first: click it to see '
                  'what, double-click to switch it on together with what it needs.', DIM)

    # ---------------------------------------------------------------- side panel
    def _panel(self):
        col = QVBoxLayout()
        col.setContentsMargins(PANEL_PAD, 20, PANEL_PAD, 12)
        col.setSpacing(6)

        title = QLabel('NiFT Launcher')
        title.setObjectName('title')
        col.addWidget(title)
        col.addWidget(self._dim('nift_bringup · system.launch.py'))

        col.addWidget(self._section('PROFILE PRESET'))
        self.preset = QComboBox()
        for name, p in self.model.profiles.items():
            self.preset.addItem(f'{name}   ·  {p["mode"]}', name)
            self.preset.setItemData(self.preset.count() - 1, p.get('description', ''),
                                    Qt.ToolTipRole)
        # also on choosing the same one again: reset
        self.preset.activated.connect(self._preset_chosen)
        col.addWidget(self.preset)
        self.preset_desc = self._dim('')
        col.addWidget(self.preset_desc)

        col.addWidget(self._section('MODE'))
        modes = QHBoxLayout()
        modes.setSpacing(0)
        self.mode_buttons = QButtonGroup(self)
        for mode, text, tip in (('real', 'Real vehicle', 'the controller drives the shuttle'),
                                ('sim', 'CARLA',
                                 'simulation in CARLA + the Mcity twin: needs a CARLA server '
                                 '(NVIDIA GPU)'),
                                ('kinematic', 'Kinematic',
                                 'simulation without CARLA or a GPU: bicycle model with the '
                                 'shuttle plant, no LiDAR (perception off)')):
            b = QPushButton(text)
            b.setObjectName('mode_real' if mode == 'real' else 'mode_sim')
            b.setToolTip(tip)
            b.setCheckable(True)
            b.clicked.connect(lambda _, m=mode: self._mode_chosen(m))
            self.mode_buttons.addButton(b)
            modes.addWidget(b)
            setattr(self, f'mode_{mode}', b)
        col.addLayout(modes)
        self.mode_banner = QLabel()
        self.mode_banner.setWordWrap(True)
        col.addWidget(self.mode_banner)

        self.notes = QLabel()
        self.notes.setWordWrap(True)
        self.notes.setTextFormat(Qt.RichText)
        col.addWidget(self.notes)
        self.filter_label = self._dim('')
        col.addWidget(self.filter_label)

        self.args_toggle = QToolButton()
        self.args_toggle.setText('ADVANCED ARGUMENTS')
        self.args_toggle.setCheckable(True)
        self.args_toggle.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.args_toggle.setArrowType(Qt.RightArrow)
        self.args_toggle.toggled.connect(self._toggle_args)
        col.addWidget(self.args_toggle)
        self.args_box = QWidget()
        form = QFormLayout(self.args_box)
        form.setContentsMargins(0, 0, 0, 0)
        form.setSpacing(5)
        self.arg_fields = {}
        for key, (_, description) in ARGS.items():
            # the installed maps to pick from; a path can still be typed
            if key == 'map_file':
                widget = QComboBox()
                widget.setEditable(True)
                widget.addItems([''] + installed_maps())
                widget.setToolTip(description)
                field_ = widget.lineEdit()
            else:
                field_ = widget = QLineEdit()
            field_.setToolTip(description)
            field_.textChanged.connect(lambda text, k=key: (self.model.set_arg(k, text),
                                                            self._refresh()))
            label = QLabel(key)
            label.setToolTip(description)
            label.setStyleSheet(f'color: {DIM};')
            form.addRow(label, widget)
            self.arg_fields[key] = field_
        self.args_box.setVisible(False)
        col.addWidget(self.args_box)

        head = QHBoxLayout()
        head.addWidget(self._section('COMMAND'))
        head.addStretch(1)
        copy = QPushButton('Copy')
        copy.setStyleSheet('padding: 2px 10px; margin-top: 8px;')
        copy.clicked.connect(lambda: QApplication.clipboard().setText(self.command_box.text()))
        head.addWidget(copy)
        col.addLayout(head)
        self.command_box = QLabel()
        self.command_box.setObjectName('command')
        self.command_box.setWordWrap(True)
        self.command_box.setTextInteractionFlags(Qt.TextSelectableByMouse)
        mono = QFont('monospace')
        mono.setStyleHint(QFont.TypeWriter)
        self.command_box.setFont(mono)
        col.addWidget(self.command_box)

        # preflight results below the command, in their own area that scrolls past RESULTS_LINES
        col.addWidget(self._section('PREFLIGHT'))
        self.results_label = QLabel()
        self.results_label.setWordWrap(True)
        self.results_label.setTextFormat(Qt.RichText)
        self.results_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.results_area = QScrollArea()
        self.results_area.setWidget(self.results_label)
        self.results_area.setWidgetResizable(True)
        self.results_area.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        col.addWidget(self.results_area)
        col.addStretch(1)

        # everything above scrolls when the window is short; Check / Start stay at the bottom
        self.panel_inner = QWidget()
        self.panel_inner.setLayout(col)
        self.scroll = QScrollArea()
        self.scroll.setWidget(self.panel_inner)
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.check_button = QPushButton('Check')
        self.check_button.clicked.connect(self._check)
        self.start_button = QPushButton('Start')
        self.start_button.clicked.connect(self._start)
        self.buttons = QHBoxLayout()
        self.buttons.setContentsMargins(PANEL_PAD, 10, PANEL_PAD, 18)
        self.buttons.addWidget(self.check_button, 1)
        self.buttons.addWidget(self.start_button, 2)

        self.panel = QFrame()
        self.panel.setObjectName('panel')
        outer = QVBoxLayout(self.panel)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(self.scroll, 1)
        outer.addLayout(self.buttons)
        return self.panel

    def _fit_panel(self):
        """
        Set the panel width.

        PANEL_EM lines of the UI font, wider still if a title or a button would clip.
        """
        scrollbar = self.scroll.verticalScrollBar().sizeHint().width()
        need = max(self.panel_inner.minimumSizeHint().width() + scrollbar,
                   self.buttons.minimumSize().width())
        self.panel.setFixedWidth(max(round(PANEL_EM * self.fontMetrics().height()), need))

    def _section(self, text):
        label = QLabel(text)
        label.setObjectName('section')
        return label

    def _dim(self, text):
        label = QLabel(text)
        label.setObjectName('dim')
        label.setWordWrap(True)
        return label

    def _toggle_args(self, shown):
        self.args_box.setVisible(shown)
        self.args_toggle.setArrowType(Qt.DownArrow if shown else Qt.RightArrow)

    # ---------------------------------------------------------------- actions
    def _click(self, name):
        self._before_click = set(self.model.on)
        blocked = self.map.states[name] == NodeState.BLOCKED
        change = self.model.click(name)
        if blocked:
            self.map.point_at([name, *change.highlight])
        self._report(change)

    def _double_click(self, name):
        # undo the double-click's first click: a double-click only adds
        self.model.on = self._before_click
        change = self.model.turn_on_with_prereqs(name)
        if change.message and not change.turned_on:         # still blocked: point at what it needs
            self.map.point_at([name, *change.highlight])
        self._report(change)

    def _mode_chosen(self, mode):
        change = self.model.set_mode('real') if mode == 'real' else \
            self.model.set_backend('kinematic' if mode == 'kinematic' else 'carla')
        self.arg_fields['sim_backend'].setText(self.model.args['sim_backend'])
        self._report(change)

    def _preset_chosen(self, index):
        name = self.preset.itemData(index)
        self.model.select(name)
        self._sync_controls()
        self._refresh()
        self._say(f'preset {name}: {self.model.profiles[name].get("description", "")}', FG)

    def _sync_controls(self):
        """Preset box and argument placeholders from the model's preset."""
        self.preset.setCurrentIndex(self.preset.findData(self.model.preset))
        p = self.model.profiles[self.model.preset]
        self.preset_desc.setText(p.get('description', ''))
        prof_args = p.get('args') or {}
        for key, f in self.arg_fields.items():
            f.setPlaceholderText(str(prof_args.get(key, ARGS[key][0])) or '—')
        self._map_placeholder()
        # select() cleared it
        self.arg_fields['sim_backend'].setText(self.model.args['sim_backend'])

    def _map_placeholder(self):
        """map_file shows the map that '' gives: the preset's, else the mode's default."""
        prof_args = self.model.profiles[self.model.preset].get('args') or {}
        self.arg_fields['map_file'].setPlaceholderText(
            str(prof_args.get('map_file') or DEFAULT_MAPS['sim' if self.model.sim else 'real']))

    def _report(self, change):
        """
        Refresh after a click or a mode switch, and say what happened.

        Amber: nothing changed.
        """
        self._refresh()
        if change.message:
            self._say(change.message, FG if change.turned_on or change.turned_off else AMBER)

    def _say(self, text, tone):
        self.status.setText(text)
        self.status.setStyleSheet(f'background: {CANVAS}; color: {tone}; padding: 10px 18px; '
                                  'font-size: 10.5pt;')

    def _refresh(self):
        """
        Show everything from the model.

        Mode, command, warnings, check results, buttons, the map.
        """
        sim = self.model.sim
        kinematic = self.model.kinematic
        self._map_placeholder()
        self.mode_real.setChecked(not sim)
        self.mode_sim.setChecked(sim and not kinematic)
        self.mode_kinematic.setChecked(kinematic)
        self.mode_banner.setText('<b>● REAL VEHICLE</b><br>the controller drives the shuttle'
                                 if not sim else
                                 '<b>● KINEMATIC SIMULATION</b><br>no CARLA, no LiDAR: '
                                 'can is the kinematic sim, always on' if kinematic else
                                 '<b>● CARLA SIMULATION</b><br>can is the CARLA bridge, always on')
        self.mode_banner.setStyleSheet(f'background: {"#5a1d1a" if not sim else "#123d37"}; '
                                       f'color: {"#ffb4ab" if not sim else "#9ee6d8"}; '
                                       'border-radius: 6px; padding: 7px 10px; margin-top: 4px;')

        command = self.model.command()
        self.command_box.setText(shlex.join(command))
        try:
            plan, errors = self.model.plan(), []
        except ProfileError as e:
            plan, errors = None, rule_errors(e)
        self._notes(plan, errors)
        self.start_button.setEnabled(plan is not None and not self.checking)
        self.start_button.setText(('Start — REAL VEHICLE' if not sim else
                                   'Start kinematic sim' if kinematic else 'Start in CARLA')
                                  + ('  (dry run)' if self.dry_run else ''))
        self.start_button.setStyleSheet(
            '' if plan is None or self.checking else
            f'QPushButton {{ background: {"#c0392b" if not sim else "#1f7a6c"}; border: none; '
            'color: white; font-weight: bold; }')
        self._fit_panel()
        self._show_results(command)
        self.map.refresh(self._levels(command))

    def _notes(self, plan, errors):
        """
        Show the broken rules (red) and the plan's warnings (amber).

        An argument named in an error gets a red frame.
        """
        box = ('<table width="100%" cellpadding="8" '
               'style="background:{bg}; color:{fg}; margin-top:4px">'
               '<tr><td><b>{head}</b><br>{body}</td></tr></table>')
        boxes = []
        if errors:
            boxes.append(box.format(bg='#4a1c1c', fg='#ffb4ab', head='Cannot start',
                                    body='<br>'.join(map(html.escape, errors))))
        if plan is not None and plan.warnings:
            boxes.append(box.format(bg='#4a3510', fg='#ffd8a0', head='⚠ Warning',
                                    body='<br>'.join(map(html.escape, plan.warnings))))
        self.notes.setText(''.join(boxes))
        self.notes.setVisible(bool(boxes))
        shown = [f'Safety filter: <b>{safety_filter(plan)}</b>'] if plan else []
        mode, _, note = localization(plan).partition(': ') if plan else ('', '', '')
        if mode:
            shown.append(f'Localization: <b>{mode}</b> ({html.escape(note)})')
        if plan and 'planning' in plan.on:
            shown.append(f'Map: <b>{html.escape(plan.args["map_file"])}</b>')
        self.filter_label.setText('<br>'.join(shown))
        self.filter_label.setVisible(plan is not None)
        for key, field_ in self.arg_fields.items():
            bad = any(f'{key}:=' in e for e in errors)
            field_.setStyleSheet(f'border: 1px solid {LEVEL_COLOR[Level.FAIL]};' if bad else '')

    # ---------------------------------------------------------------- preflight
    def _check(self):
        try:
            plan = self.model.plan()
        except ProfileError:
            return
        self.checking = True
        self.check_button.setEnabled(False)
        self.check_button.setText('Checking…')
        self.runner.start(plan, self.model.command())
        self._refresh()

    def _checked(self, plan, command, results, seconds):
        self.checking = False
        self.check_button.setEnabled(True)
        self.check_button.setText('Check')
        self.results, self.results_for, self.results_s = results, command, seconds
        self.results_plan = plan
        self._refresh()
        QTimer.singleShot(0, lambda: self.scroll.ensureWidgetVisible(self.results_area))

    def _current_results(self, command):
        if isinstance(self.results, list) and self.results_for == command:
            return self.results
        return None

    def _levels(self, command):
        results = [(c, lv) for c, lv,
                   _ in self._current_results(command) or [] if lv != Level.NONE]
        return {c: worst([lv for n, lv in results if n == c]) for c, _ in results}

    def _show_results(self, command):
        self.results_label.setText(self._results_html(command))
        scrollbar = self.scroll.verticalScrollBar().sizeHint().width()
        # the panel's and the list's own
        width = self.panel.width() - 2 * PANEL_PAD - 2 * scrollbar
        tallest = RESULTS_LINES * self.fontMetrics().lineSpacing()
        self.results_area.setFixedHeight(min(self.results_label.heightForWidth(width) + 4,
                                             tallest))

    def _results_html(self, command):
        if self.checking:
            return f'<span style="color:{DIM}">checking … (up to ~5 s)</span>'
        if self.results is None:
            return f'<span style="color:{DIM}">not checked yet</span>'
        if isinstance(self.results, Exception):
            return (f'<span style="color:{LEVEL_COLOR[Level.FAIL]}">preflight crashed: '
                    f'{type(self.results).__name__}: {html.escape(str(self.results))}</span>')
        stale = self.results_for != command
        counts = {lv: sum(1 for _, x, _ in self.results if x == lv)
                  for lv in (Level.FAIL, Level.WARN)}
        head = ', '.join(f'{n} {lv.value}' for lv, n in counts.items() if n) or 'all passed'
        head = f'<b>{head}</b> <span style="color:{DIM}">({self.results_s:.1f} s)</span>'
        if stale:
            head = (f'<span style="color:{AMBER}"><b>changed since the check: Check again</b>'
                    '</span><br>' + head)
        rows = ''.join(
            f'<tr><td style="padding:3px 8px 3px 0"><span style="background:{LEVEL_COLOR[lv]}; '
            f'color:{"#1e1e1e" if lv == Level.WARN else "white"}">'
            f'&nbsp;{lv.value}&nbsp;</span></td>'
            f'<td style="padding:3px 0">'
            f'<b>{html.escape(preflight.label(c, self.results_plan))}</b>&nbsp; '
            f'{html.escape(msg)}</td></tr>'
            for c, lv, msg in self.results)
        style = f' style="color:{DIM}"' if stale else ''
        return f'{head}<table{style}>{rows}</table>'

    # ---------------------------------------------------------------- start
    def _start(self):
        try:
            self.model.plan()
        except ProfileError:
            return
        command = self.model.command()
        results = self._current_results(command)
        problem = None
        if results is None:
            problem = 'This selection has not been checked.'
        elif preflight.failed(results):
            fails = [f'{c}: {msg}' for c, lv, msg in results if lv == Level.FAIL]
            problem = 'The check FAILED:\n  ' + '\n  '.join(fails)
        checks = True
        if problem:
            where = ('on the REAL VEHICLE' if not self.model.sim else
                     'in the kinematic sim' if self.model.kinematic else 'in CARLA')
            box = QMessageBox(QMessageBox.Warning, 'Start anyway?',
                              f'{problem}\n\nStart anyway {where}, '
                              'with the launch\'s own checks off '
                              '(checks:=false)?', QMessageBox.Yes | QMessageBox.Cancel, self)
            box.button(QMessageBox.Yes).setText('Start anyway')
            box.setDefaultButton(QMessageBox.Cancel)
            if box.exec_() != QMessageBox.Yes:
                return
            checks = False
        self.start_command = self.model.command(checks=checks, dry_run=self.dry_run)
        self.close()


def enable_high_dpi():
    """
    Scale the UI by the screen's DPI.

    Xft.dpi 240 -> x2.5, not rounded to 2 or 3. Call it before the QApplication.
    """
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    QGuiApplication.setHighDpiScaleFactorRoundingPolicy(
        Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)


def main(argv=None):
    parser = argparse.ArgumentParser(prog='ros2 run nift_bringup launcher',
                                     description='Pick the components of system.launch.py '
                                                 'on a map, check, start.')
    parser.add_argument('--profile', default='none', help='preset to open with (default none)')
    parser.add_argument('--dry-run', action='store_true',
                        help='Start adds dry_run:=true: the launch prints the plan and the '
                             'checks, starts nothing')
    args, _ = parser.parse_known_args(argv)        # ignore --ros-args and the like

    profiles = load(share('nift_bringup', 'config', 'profiles.yaml'))
    if args.profile not in profiles:
        parser.error(f'unknown profile {args.profile}; known: {", ".join(profiles)}')

    signal.signal(signal.SIGINT, signal.SIG_DFL)        # Ctrl-C in the terminal ends the launcher
    enable_high_dpi()
    app = QApplication(sys.argv[:1])
    app.setStyle('Fusion')
    window = LauncherWindow(profiles, args.profile, args.dry_run)
    window.show()
    app.exec_()

    command = window.start_command
    if command:
        print('\n$ ' + shlex.join(command) + '\n', flush=True)
        os.execvp(command[0], command)      # this terminal becomes the launch


if __name__ == '__main__':
    main()
