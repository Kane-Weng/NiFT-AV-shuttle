#!/usr/bin/env python3
"""
Plot one sim test-route run: the route on the map, its turns and its goal approach.

File: route_plot.py

Description:
  One PNG from a bag of one route (sim_route_run's run dir, or any sim bag with the same
  topics):

    overview       the route on the Lanelet2 map: planned path (/planned_path, dashed),
                   driven base_link (sim ground truth), path start and end, where the
                   shuttle came to rest (or stalled); boxes mark the zoom panels
    zoom panels    the path end (arrival, rest vs the end) and up to --max-turns turn
                   windows (route_metrics.turn_windows, the sharpest if there are more),
                   each with its largest CTE, the tightest point of the path and its metrics
    goal approach  speed and the controller's target speed against the distance to go to
                   the path end, over the last APPROACH_M m before the shuttle stopped,
                   arrival and rest marked; pedal and brake counts of the 0x101 frames below,
                   on the same x axis

  Several bags of the same route overlay their driven traces (one colour each, up to
  len(COLOURS)) to compare variants; turn windows and zooms come from the first bag.

  The map is the one run.json names, else the sim map (lanelet2_odr2024.osm from
  nift_waypoint's maps/), projected like waypoint_node (UtmProjector at the map datum).

Usage:
  ros2 run nift_carla sim_route_plot <bag | run dir> [-o out.png] [--map-file <osm>]
  ros2 run nift_carla sim_route_plot runA runB --labels baseline 'lookahead 3 m' -o cmp.png
"""

import argparse
import os
import sys

from matplotlib import rc_context
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.figure import Figure
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from nift_bringup.profiles import DEFAULT_MAPS, map_path
from nift_carla.tools import route_metrics as rm
from nift_common.can_protocol import CAN_CMD_MAX
from nift_common.site import MAP_ORIGIN_LAT, MAP_ORIGIN_LON
import numpy as np

# the look of the documentation figures
SURF, INK, INK2 = '#fcfcfb', '#0b0b0b', '#52514e'
LANE_FILL, LANE_EDGE, CL = '#e7e6e1', '#cfcec8', '#b9b8b1'
BLUE, ORANGE, AQUA, CRIT = '#2a78d6', '#eb6834', '#1baf7a', '#d03b3b'
GRID = '#ecebe7'
COLOURS = [BLUE, ORANGE, AQUA]      # one per bag, in this order (distinct as a set)
APPROACH_M = 15.0                   # goal approach: the last metres before the shuttle stopped
DASH = (0, (5, 3))

STYLE = {
    'font.family': 'DejaVu Sans', 'font.size': 9, 'axes.edgecolor': LANE_EDGE,
    'axes.labelcolor': INK2, 'xtick.color': INK2, 'ytick.color': INK2, 'axes.titlesize': 10,
    'axes.titleweight': 'bold', 'axes.titlecolor': INK, 'axes.titlelocation': 'left',
    'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
    'legend.frameon': False, 'axes.spines.top': False, 'axes.spines.right': False}


def _lead(colour=CRIT):
    return {'arrowstyle': '-', 'color': colour, 'lw': 1}


# ---------------------------------------------------------------- map

def load_lanes(map_file, origin):
    """Load [(left bound, right bound, centreline)] of every lanelet, map frame."""
    import lanelet2
    from lanelet2.io import Origin
    from lanelet2.projection import UtmProjector
    lmap = lanelet2.io.load(map_file, UtmProjector(Origin(*origin)))

    def xy(line):
        return np.array([(p.x, p.y) for p in line])
    return [(xy(ll.leftBound), xy(ll.rightBound), xy(ll.centerline)) for ll in lmap.laneletLayer]


def draw_map(ax, lanes, box):
    """Draw lane fills, bounds and dashed centrelines of the lanelets that overlap box."""
    x0, x1, y0, y1 = box
    near = []
    for ln in lanes:
        pts = np.vstack(ln[:2])
        lo, hi = pts.min(0), pts.max(0)
        if lo[0] < x1 and hi[0] > x0 and lo[1] < y1 and hi[1] > y0:
            near.append(ln)
    ax.add_collection(PolyCollection([np.r_[le, ri[::-1]] for le, ri, _ in near],
                                     facecolors=LANE_FILL, edgecolors='none', zorder=0))
    ax.add_collection(LineCollection([b for ln in near for b in ln[:2]], colors=LANE_EDGE,
                                     linewidths=0.4, zorder=1))
    ax.add_collection(LineCollection([ln[2] for ln in near], colors=CL, linewidths=0.5,
                                     linestyles=(0, (4, 3)), zorder=1))
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(True)
        sp.set_color(LANE_EDGE)


def square_box(pts, pad):
    """Return a square (x0, x1, y0, y1) around points, padded by pad m."""
    pts = np.asarray(pts)
    cx, cy = (pts.min(0) + pts.max(0)) / 2.0
    half = max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])) / 2.0 + pad
    return cx - half, cx + half, cy - half, cy + half


def scalebar(ax, box):
    """Draw a round-number scale bar of about a sixth of the panel, bottom left."""
    width, height = box[1] - box[0], box[3] - box[2]
    length = next(v for v in (1, 2, 5, 10, 20, 50, 100, 200, 500) if v >= width / 6.0)
    x, y = box[0] + 0.05 * width, box[2] + 0.05 * height
    ax.plot([x, x + length], [y, y], color=INK, lw=2, solid_capstyle='butt', zorder=10)
    ax.text(x + length / 2, y + 0.015 * height, f'{length:g} m', ha='center', va='bottom',
            color=INK, fontsize=8, zorder=10)


def free_corner(box, pts):
    """Return the axes-fraction position of the panel corner farthest from pts."""
    best, where = -1.0, (0.04, 0.96)
    for fx, fy in ((0.04, 0.96), (0.96, 0.96), (0.04, 0.16), (0.96, 0.16)):
        cx, cy = box[0] + fx * (box[1] - box[0]), box[2] + fy * (box[3] - box[2])
        d = np.min(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)) if len(pts) else 1e9
        if d > best:
            best, where = d, (fx, fy)
    return where


# ---------------------------------------------------------------- one run

class RunData:
    """A run's metrics, its odom matched to the path, and its colour / label in the figure."""

    def __init__(self, arg, colour, label):
        self.run = rm.load_run(arg)
        self.m = rm.compute(self.run)
        self.tr = rm.track(self.run)
        self.colour, self.label = colour, label
        tr = self.tr
        self.xy = tr['od'][:, 1:3]
        self.dtg = rm.distance_to_go(tr['path'], tr['s_cum'], tr['s'], self.xy)
        k = rm.rest_index(tr['od'][:, 4])
        self.rest_i = k if k is not None else len(tr['od']) - 1

    def at(self, t, values):
        return np.interp(t, self.tr['od'][:, 0], values)


def outcome_line(m):
    """Return the run's headline: the table row's values."""
    ok = m['completed']
    head = 'completed' if ok else rm.outcome_text(m).replace('**no**:', 'NOT completed:')
    parts = [head]
    if m['cte_rms'] is not None:
        parts.append(f'CTE RMS / max {m["cte_rms"]:.2f} / {m["cte_max"]:.2f} m')
    parts.append(f'heading error max {rm._f(m["heading_err_max_deg"], "{:.0f}")}°')
    parts.append(f'steering rate max {rm._f(m["steer_rate_max_deg_s"], "{:.0f}")} °/s')
    if ok:
        parts.append(f'arrival {m["arrival_speed"]:.2f} m/s, rest {m["rest_vs_end"]:+.2f} m '
                     'vs the path end')
    parts.append(f'cruise {rm._f(m["cruise_speed"])} m/s')
    return ' · '.join(parts)


# ---------------------------------------------------------------- panels

def draw_traces(ax, runs):
    """Draw the planned path(s) dashed, the driven traces and where each run stopped."""
    first = runs[0]
    ax.plot(*first.tr['path'].T, color=INK, lw=1.1, ls=DASH, zorder=3)
    for r in runs[1:]:
        p = r.tr['path']
        if p.shape != first.tr['path'].shape or np.abs(p - first.tr['path']).max() > 0.3:
            ax.plot(*p.T, color=r.colour, lw=0.9, ls=DASH, zorder=3, alpha=0.8)
    for r in runs:
        ax.plot(*r.xy.T, color=r.colour, lw=2.0, zorder=4, solid_capstyle='round')
    for r in runs:
        ax.plot(*r.xy[r.rest_i], 'o', ms=7, mfc=CRIT, mec='white', mew=1.2, zorder=7)


def panel_overview(ax, runs, lanes, zooms):
    """Draw the whole route with boxes for the zoom panels; return the legend handles."""
    first = runs[0]
    corners = [(zb[0], zb[2]) for _, zb in zooms] + [(zb[1], zb[3]) for _, zb in zooms]
    box = square_box(np.vstack([first.tr['path']] + [r.xy for r in runs] + [corners]), 6.0)
    draw_map(ax, lanes, box)
    draw_traces(ax, runs)
    path = first.tr['path']
    ax.plot(*path[0], 'o', ms=8, mfc='white', mec=INK, mew=2, zorder=6)
    ax.plot(*path[-1], 's', ms=8, mfc=INK, mec='white', mew=1.2, zorder=6)
    if not first.m['completed']:
        ax.annotate(first.m['outcome'].replace('_', ' '), first.xy[first.rest_i],
                    xytext=(10, -14), textcoords='offset points', fontsize=8, color=CRIT,
                    zorder=8)
    for label, zb in zooms:
        ax.add_patch(Rectangle((zb[0], zb[2]), zb[1] - zb[0], zb[3] - zb[2], fc='none', ec=INK2,
                               lw=0.9, zorder=8))
        ax.annotate(label, (zb[0], zb[3]), xytext=(1, 2), textcoords='offset points',
                    fontsize=8, fontweight='bold', color=INK, zorder=9)
    scalebar(ax, box)
    ax.annotate('N', (0.94, 0.96), xytext=(0.94, 0.88), xycoords='axes fraction', ha='center',
                fontweight='bold', color=INK,
                arrowprops={'arrowstyle': '-|>', 'color': INK, 'lw': 1.2})
    ax.set_title(f'Route, {first.m["path_length"]:.0f} m planned')
    driven = 'driven (base_link, sim ground truth)'
    handles = [Line2D([], [], color=INK, lw=1.1, ls=DASH, label='planned path (/planned_path)')]
    handles += [Line2D([], [], color=r.colour, lw=2, label=r.label if len(runs) > 1 else driven)
                for r in runs]
    handles += [Line2D([], [], ls='', marker='o', mfc='white', mec=INK, mew=2, ms=7,
                       label='path start'),
                Line2D([], [], ls='', marker='s', mfc=INK, mec='white', ms=7, label='path end'),
                Line2D([], [], ls='', marker='o', mfc=CRIT, mec='white', ms=7, label='at rest')]
    return handles


def zoom_turn(ax, runs, lanes, k, win):
    """Draw turn window k; return its box."""
    first = runs[0]
    s0, s1, dh = win
    sel = [(r, (r.tr['s'] >= s0) & (r.tr['s'] <= s1)) for r in runs]
    path, s_cum = first.tr['path'], first.tr['s_cum']
    in_win = (s_cum >= s0) & (s_cum <= s1)
    pts = np.vstack([path[in_win]] + [r.xy[w] for r, w in sel if w.any()])
    box = square_box(pts, 4.0)
    draw_map(ax, lanes, box)
    draw_traces(ax, runs)
    # tightest point of the planned path in the window
    rs, srs = first.tr['path_rs'], first.tr['s_rs']
    r3 = rm.radius3(rs)
    inside = (srs >= s0) & (srs <= s1)
    notes = []
    # largest CTE of the first run in the window (the metric's samples only)
    m = first.tr['mask'] & sel[0][1]
    if m.any():
        i = int(np.argmax(np.where(m, np.abs(first.tr['cte']), -1.0)))
        corner = free_corner(box, pts)
        ax.annotate(f'{abs(first.tr["cte"][i]):.2f} m off the path', first.xy[i], xytext=corner,
                    textcoords='axes fraction', ha='left' if corner[0] < 0.5 else 'right',
                    va='top' if corner[1] > 0.5 else 'bottom', fontsize=8, color=INK, zorder=9,
                    bbox={'boxstyle': 'round,pad=0.3', 'fc': 'white', 'ec': CRIT, 'lw': 1},
                    arrowprops=_lead())
    for r in runs:
        w = next((t for t in r.m['turns']
                  if abs(t['s0'] - s0) < 2.0 and abs(t['s1'] - s1) < 2.0), None)
        if w is None:
            continue
        txt = (f'CTE RMS {rm._f(w["cte_rms"])}, max {rm._f(w["cte_max"])} m · heading max '
               f'{rm._f(w["heading_err_max_deg"], "{:.0f}")}°\nsteering rate max '
               f'{rm._f(w["steer_rate_max_deg_s"], "{:.0f}")} °/s · v min {rm._f(w["v_min"])} m/s')
        notes.append(f'{r.label}: {txt}' if len(runs) > 1 else txt)
    if inside.any() and np.isfinite(r3[inside]).any():
        j = int(np.argmin(np.where(inside, r3, np.inf)))
        ax.plot(*rs[j], 'D', ms=5, mfc=CRIT, mec='white', mew=0.8, zorder=7)
        notes.append(f'tightest path radius {r3[j]:.1f} m (◆)')
    scalebar(ax, box)
    ax.set_title(f'Turn {k}: {dh:+.0f}°, s {s0:.0f}–{s1:.0f} m', fontsize=9.5)
    ax.text(0.0, -0.02, '\n'.join(notes), transform=ax.transAxes, va='top', fontsize=7.5,
            color=INK2)
    return box


def zoom_end(ax, runs, lanes):
    """Draw the path end with arrival and rest; return its box."""
    first = runs[0]
    path, s_cum = first.tr['path'], first.tr['s_cum']
    near = s_cum >= s_cum[-1] - 8.0
    pts = np.vstack([path[near]] + [r.xy[max(r.rest_i - 1, 0):r.rest_i + 1] for r in runs])
    box = square_box(pts, 3.0)
    draw_map(ax, lanes, box)
    draw_traces(ax, runs)
    ax.plot(*path[-1], 's', ms=8, mfc=INK, mec='white', mew=1.2, zorder=6)
    notes = []
    for r in runs:
        m = r.m
        why = m['outcome'].replace('_', ' ')
        if m['arrival_speed'] is not None:
            t_end = r.tr['t_end']
            ax.plot(r.at(t_end, r.xy[:, 0]), r.at(t_end, r.xy[:, 1]), 'X', ms=8, color=CRIT,
                    mec='white', zorder=7)
            txt = (f'arrival {m["arrival_speed"]:.2f} m/s at {m["arrival_vs_end"]:+.2f} m · '
                   f'rest {m["rest_vs_end"]:+.2f} m vs the end')
        elif m['rest_vs_end'] is not None:
            txt = f'no arrival ({why}) · stopped {m["rest_vs_end"]:+.2f} m vs the end'
        else:
            txt = f'no arrival ({why})'
        notes.append(f'{r.label}: {txt}' if len(runs) > 1 else txt)
    # the lane direction at the end: what arrival and rest are measured along
    e = rm.end_direction(path, s_cum)
    ax.annotate('', path[-1] + 2.0 * e, xytext=path[-1], zorder=6,
                arrowprops={'arrowstyle': '-|>', 'color': INK2, 'lw': 1})
    scalebar(ax, box)
    ax.set_title('Path end', fontsize=9.5)
    notes.append('(× arrival, ● rest, arrow: lane direction)')
    ax.text(0.0, -0.02, '\n'.join(notes), transform=ax.transAxes, va='top', fontsize=7.5,
            color=INK2)
    return box


def panel_approach(a1, a2, runs):
    """Draw speed / target speed and pedal / brake over the last APPROACH_M before the stop."""
    x_lo, x_hi, v_top = np.inf, -np.inf, 0.0
    multi = len(runs) > 1
    for r in runs:
        od, tel, can, m = r.tr['od'], r.run.tel, r.run.can, r.m
        x_rest = r.dtg[r.rest_i]
        x_lo = min(x_lo, x_rest - (1.0 if multi else 3.2))     # room for the rest label
        x_hi = max(x_hi, x_rest + APPROACH_M)
        sel = (r.dtg <= x_rest + APPROACH_M + 0.5) & (np.arange(len(od)) <= r.rest_i + 40)
        t0 = od[sel, 0].min() if sel.any() else od[0, 0]
        sel &= od[:, 0] >= t0
        a1.plot(r.dtg[sel], od[sel, 4], color=r.colour, lw=2, zorder=3,
                label=f'{r.label}: speed' if multi else 'speed (sim ground truth)')
        v_top = max(v_top, od[sel, 4].max() if sel.any() else 0.0)
        t_end = r.tr['t_end']
        if len(tel):
            tsel = (tel[:, 0] >= t0) & (tel[:, 0] <= (t_end if t_end is not None else np.inf))
            if tsel.any():
                a1.plot(r.at(tel[tsel, 0], r.dtg), tel[tsel, 1], lw=1.4, ls=(0, (4, 2)),
                        color=r.colour if multi else ORANGE, zorder=2,
                        label=f'{r.label}: target' if multi else 'controller target speed')
                v_top = max(v_top, tel[tsel, 1].max())
        if m['arrival_speed'] is not None:
            xa = float(r.at(t_end, r.dtg))
            a1.plot(xa, m['arrival_speed'], 'X', ms=9, color=CRIT, mec='white', zorder=5)
            for a in (a1, a2):
                a.axvline(xa, color=CRIT, lw=1, ls=':', zorder=1)
            if not multi:
                a1.annotate(f'arrival declared: {m["arrival_vs_end"]:+.2f} m from the end, '
                            f'{m["arrival_speed"]:.2f} m/s', (xa, m['arrival_speed']),
                            xytext=(0.36, 0.93), textcoords='axes fraction', fontsize=8,
                            color=INK, ha='left', va='top', arrowprops=_lead())
        a1.plot(x_rest, 0.0, 'o', ms=8, color=CRIT, mec='white', zorder=5)
        if not multi:
            if m['completed']:
                what = f'at rest\n{m["rest_vs_end"]:+.2f} m vs the path end'
            else:
                what = (f'{m["outcome"].replace("_", " ")}\n'
                        f'at s {m["stop_s"]:.0f} of {m["path_length"]:.0f} m')
            a1.annotate(what, (x_rest, 0.0), xytext=(x_rest - 0.2, 0.22),
                        textcoords=('data', 'axes fraction'), fontsize=8, color=INK, ha='left',
                        va='bottom', arrowprops=_lead())
        if len(can):
            csel = (can[:, 0] >= t0) & (can[:, 0] <= od[min(r.rest_i + 40, len(od) - 1), 0])
            xc = r.at(can[csel, 0], r.dtg)
            a2.step(xc, can[csel, 3], where='post', color=r.colour if multi else BLUE, lw=1.5,
                    label=f'{r.label}: pedal' if multi else 'pedal')
            a2.step(xc, can[csel, 2], where='post', color=r.colour if multi else CRIT, lw=1.5,
                    ls=(0, (2, 1.5)) if multi else '-',
                    label=f'{r.label}: brake' if multi else 'brake')
    for a in (a1, a2):
        if x_lo < 0.0 < x_hi:
            a.axvline(0.0, color=INK, lw=1, zorder=1)
        a.grid(axis='y', color=GRID, lw=0.8)
        a.set_xlim(x_hi, x_lo)
    if x_lo < 0.0 < x_hi:
        a1.text(0.0, 1.0, ' path end', transform=a1.get_xaxis_transform(), ha='left', va='top',
                fontsize=8, color=INK)
    a1.set_ylim(-0.05, max(v_top * 1.15, 1.0))
    a1.set_ylabel('m/s')
    a1.legend(loc='lower left', fontsize=8, ncol=2 if multi else 1)
    a1.set_title(f'Goal approach: the last {APPROACH_M:.0f} m before the shuttle stopped')
    a2.set_ylabel('CAN 0x101 counts')
    a2.set_ylim(-10, CAN_CMD_MAX + 15)
    a2.set_yticks([0, 128, CAN_CMD_MAX])
    a2.legend(loc='center left', fontsize=8, ncol=2 * len(runs))
    a2.set_xlabel('distance to go to the path end (m)   ← negative = past the end')


# ---------------------------------------------------------------- figure

def plot_runs(args, out, map_file=None, labels=None, max_turns=3, origin=None):
    """Write the figure of one or more runs of the same route to out; return out."""
    if len(args) > len(COLOURS):
        raise ValueError(f'at most {len(COLOURS)} bags in one figure')
    labels = list(labels or [])
    runs = []
    for i, a in enumerate(args):
        label = labels[i] if i < len(labels) else rm.bag_name(rm.find_bag(a)[0])
        runs.append(RunData(a, COLOURS[i], label))
    end = runs[0].tr['path'][-1]
    other = [r.label for r in runs[1:] if np.hypot(*(r.tr['path'][-1] - end)) > 5.0]
    if other:
        print(f'WARNING: {", ".join(other)}: path ends more than 5 m from {runs[0].label}\'s; '
              'not the same route? Turn windows and zooms come from the first bag',
              file=sys.stderr)
    map_file = map_path(map_file or runs[0].run.info.get('map_file') or DEFAULT_MAPS['sim'])
    origin = origin or (MAP_ORIGIN_LAT, MAP_ORIGIN_LON)
    lanes = load_lanes(map_file, origin)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with rc_context(STYLE):
        figure(runs, lanes, max_turns).savefig(out, dpi=150)
    return out


def figure(runs, lanes, max_turns):
    """
    Build the figure: overview and zoom panels on top, goal approach below.

    Map grid: the overview takes 2 x 2 cells; the end zoom and the turn zooms fill the cells
    to its right (3 columns for up to 2 zooms, else 4); a spare cell holds the legend.
    """
    first = runs[0]
    wins = first.tr['windows']
    if len(wins) > max_turns:      # keep the sharpest turns, in route order
        keep = sorted(sorted(range(len(wins)), key=lambda i: -abs(wins[i][2]))[:max_turns])
        shown = [(i + 1, wins[i]) for i in keep]
    else:
        shown = [(i + 1, w) for i, w in enumerate(wins)]
    n_zoom = 1 + len(shown)
    cols = 3 if n_zoom <= 2 else 4
    cell = 14.0 / cols
    height = 2 * cell + 6.6 + 0.2 * (len(runs) - 1)
    fig = Figure(figsize=(14.0, height))
    outer = fig.add_gridspec(2, 1, height_ratios=[2 * cell, 4.4], hspace=0.17, left=0.04,
                             right=0.98, top=1 - (0.75 + 0.2 * len(runs)) / height,
                             bottom=0.5 / height)
    maps = outer[0].subgridspec(2, cols, hspace=0.3, wspace=0.12)
    lower = outer[1].subgridspec(2, 1, height_ratios=[2.8, 1.6], hspace=0.08)
    ax_over = fig.add_subplot(maps[0:2, 0:2])
    cells = [(0, 2), (1, 2)] if cols == 3 else [(0, 2), (0, 3), (1, 2), (1, 3)]
    zoom_axes = [fig.add_subplot(maps[r, c]) for r, c in cells[:n_zoom]]
    spare = [fig.add_subplot(maps[r, c]) for r, c in cells[n_zoom:]]

    zooms = [('end', zoom_end(zoom_axes[0], runs, lanes))]
    for ax, (k, w) in zip(zoom_axes[1:], shown):
        zooms.append((str(k), zoom_turn(ax, runs, lanes, k, w)))
    handles = panel_overview(ax_over, runs, lanes, zooms)
    if spare:
        spare[0].axis('off')
        spare[0].legend(handles=handles, loc='upper left', fontsize=8.5)
        for ax in spare[1:]:
            ax.axis('off')
    else:
        ax_over.legend(handles=handles, loc='upper left', fontsize=8)
    if len(wins) > len(shown):
        ax_over.text(1.0, -0.01, f'{len(wins)} turn windows, the {len(shown)} sharpest shown',
                     transform=ax_over.transAxes, ha='right', va='top', fontsize=8, color=INK2)

    a1 = fig.add_subplot(lower[0])
    a2 = fig.add_subplot(lower[1], sharex=a1)
    panel_approach(a1, a2, runs)
    a1.tick_params(labelbottom=False)

    info = first.run.info
    title = f'Route {first.m["route"]}' + (f' ({info["name"]})' if info.get('name') else '')
    title += f': {info.get("description") or first.m["name"]}'
    fig.text(0.01, 1 - 0.15 / height, title, ha='left', va='top', fontweight='bold',
             fontsize=12, color=INK)
    y = 1 - 0.5 / height
    for r in runs:
        line = f'{r.label}: {outcome_line(r.m)}' if len(runs) > 1 else outcome_line(r.m)
        box = None
        if not r.m['completed']:
            box = {'boxstyle': 'round,pad=0.25', 'fc': 'white', 'ec': CRIT, 'lw': 1}
        fig.text(0.01, y, line, ha='left', va='top', fontsize=8.5, color=INK, bbox=box)
        y -= 0.22 / height
    return fig


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+',
                    help=f'rosbag2 dir(s) or sim_route_run dir(s) of one route, up to '
                         f'{len(COLOURS)}')
    ap.add_argument('-o', '--out', help='PNG (default: plot.png in the run dir, else <bag>.png)')
    ap.add_argument('--map-file', help="Lanelet2 map: a file in nift_waypoint's maps/ or a path "
                                       f"(default: run.json's, else {DEFAULT_MAPS['sim']})")
    ap.add_argument('--origin', nargs=2, type=float, metavar=('LAT', 'LON'),
                    help=f'map datum (default {MAP_ORIGIN_LAT} {MAP_ORIGIN_LON})')
    ap.add_argument('--labels', nargs='+', help='legend label per bag (default: the run names)')
    ap.add_argument('--max-turns', type=int, default=3, help='turn zoom panels, at most (3)')
    args = ap.parse_args(argv)
    try:
        bag, run_dir = rm.find_bag(args.bags[0])
        out = args.out or (os.path.join(run_dir, 'plot.png') if run_dir
                           else bag.rstrip(os.sep) + '.png')
        print(plot_runs(args.bags, out, map_file=args.map_file, labels=args.labels,
                        max_turns=args.max_turns, origin=args.origin))
    except (OSError, ValueError) as e:
        print(f'sim_route_plot: {e}', file=sys.stderr)
        return 2
    return 0


if __name__ == '__main__':
    sys.exit(main())
