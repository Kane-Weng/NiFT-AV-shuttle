# Validation against the ROS reference runs

The playground runs the reference cases of `tests/reference/` natively with the CLI and is compared with the real ROS 2 runs. This folder holds the comparison, the test and what was found (2026-10-20, validation agent of U11).

## Files

```
README.md                    this report
cases.py                     the 16 reference cases and their CLI arguments
conftest.py                  runs the CLI, every case in its own process, all at once
reference_compare.py         metrics against a reference run (numpy only)
ros_order.py                 the CLI with the ROS tick order (diagnosis, see below)
test_against_reference.py    the test
```

```bash
python3 -m pytest nift_playground/tests/validation                          # route 03 and the GNSS outage, about 3 s
NIFT_PLAYGROUND_SLOW=1 python3 -m pytest nift_playground/tests/validation   # all 16 cases, about 20 s on 32 cores
python3 nift_playground/tests/validation/reference_compare.py - <run dir named like the reference> ...
```

## Results

Each case ran twice. "shipped" is `python3 -m playground.cli` as it is. "ros_order" is the same CLI with the tick order changed at run time by `ros_order.py` (next section). Numbers are pos (cm, at the same time since the shuttle starts moving, while both navigate), shape (cm, track shape up to the shorter driven length), dur (s, nav duration minus the reference) and final (cm, final position).

| case | outcome | ROS dur | shipped: pos / shape / dur / final | ros_order: pos / shape / dur / final |
|---|---|---|---|---|
| direct_01 | completed, same | 120.08 | 1.0 / 1.0 / -0.08 / 14.0 | 0.1 / 0.1 / +0.02 / 4.9 |
| ekf_01 | completed, same | 120.10 | 0.8 / 0.8 / -0.10 / 12.9 | 0.1 / 0.1 / +0.00 / 0.0 |
| direct_02 | completed, same | 104.81 | 1.1 / 0.1 / -0.11 / 18.5 | 0.0 / 0.0 / -0.01 / 0.0 |
| ekf_02 | completed, same | 104.80 | 0.5 / 0.0 / -0.10 / 12.9 | 0.0 / 0.0 / -0.00 / 0.0 |
| direct_03 | completed, same | 48.10 | 22.7 / 3.4 / -0.10 / 8.5 | 2.5 / 0.8 / +0.10 / 8.0 |
| ekf_03 | completed, same | 48.10 | 13.1 / 1.6 / -0.10 / 7.1 | 1.5 / 0.1 / +0.00 / 0.3 |
| direct_04 | completed, same | 54.50 | 15.9 / 3.3 / -0.20 / 23.9 | 0.8 / 0.2 / -0.00 / 0.7 |
| ekf_04 | completed, same | 54.40 | 4.8 / 1.2 / -0.10 / 11.5 | 0.9 / 0.0 / +0.00 / 0.2 |
| direct_05 | completed, same | 114.38 | 32.1 / 4.3 / -0.28 / 15.4 | 3.1 / 0.5 / +0.02 / 1.2 |
| ekf_05 | completed, same | 114.30 | 23.9 / 1.6 / -0.20 / 4.0 | 1.4 / 0.1 / +0.00 / 3.9 |
| direct_11 | stalled, same | - | 23.3 / 17.9 / - / 23.3 | 0.3 / 0.2 / - / 0.3 |
| ekf_11 | stalled, same | - | 24.5 / 8.1 / - / 24.5 | 0.5 / 0.1 / - / 0.5 |
| direct_12 | completed, same | 113.12 | 166 / 13.2 / -1.02 / 1.9 | 10.6 / 1.1 / +0.08 / 0.4 |
| ekf_12 | completed, same | 112.50 | 59.6 / 6.1 / -0.40 / 3.7 | 1.2 / 1.0 / +0.10 / 4.4 |
| gnss_outage30s_01 | SAFE_STOP, same | 24.55 | 1.1 / 1.1 / +0.00 / 0.0 | 0.1 / 0.1 / +0.00 / 0.0 |
| gnss_deadzone_04 | SAFE_STOP, same | 36.70 | 1.6 / 1.5 / -0.00 / 1.9 | 0.0 / 0.0 / -0.00 / 0.1 |

In every case and both modes the outcome and the status sequence are the reference's, and the planned path is the same point for point (0.0 at the CSV's 1 µm precision). In the two GNSS-loss runs beacon_node's "GNSS stale" line and the controller's SAFE_STOP come within 2 ms of the reference, in both modes.

With `compare_runs.py` as it is (t_rel axis, whole track), shipped route 03 gives the numbers known before, 22.7 cm at the same time and 8.5 cm track.

## Why the shipped playground is off

### 1. The controller sees GNSS two ticks older on ROS (the main cause)

The map and route are ruled out. The planned path is identical on every route. The commands are identical until the shuttle moves. On route 03 the first different frame is at t_rel 1.75 s, the first tick the shuttle moves, with pedal 236 in the playground and 237 on ROS. From there the pedal stays one count lower. The ADRC pedal depends only on the speed it reads from `/gnss/vel`, so the playground controller reads a higher, fresher speed.

The experiment that settles it. Delaying what trajectory_controller receives on `/gnss/vel` by 1 tick does not match. Delaying it by 2 ticks makes every pedal value match. Then `ros_order.py` reproduces how ROS gets there.

- On ROS every node is its own process. When `/clock` reaches t, each node's timer fires on what had arrived by then.
- The sim sends its GNSS payloads of tick t to beacon_node over the websocket. They arrive after beacon_node's 50 Hz timer has fired for t, so `/gnss/*` at t carries the fix of t - dt.
- beacon_node -> shuttle_tf_node (or localization_node) -> TF arrives after trajectory_controller's 20 Hz timer has fired for t. So the controller acts on the fix of t - 2 dt.
- The runner of ARCH section 7 delivers the sim's messages before the timers (step 3) and drains after each timer (step 4). beacon_node fires before the controller (creation order), so the controller acts on the fix of t itself.

With the ROS order the playground matches the reference to the ROS run-to-run spread. Route 02 matches to 0.0 mm over 105 s, and routes 01 to 05 and 11 to 3.1 cm or better in both localization modes (the spread of ROS repeats is 3.0 cm). The size of the shipped gap fits too. Two ticks at the arrival speed of 1.84 m/s is 18 cm, and the shipped shuttle stops 13 to 24 cm short of the reference on the straight routes, about 0.1 s early.

Lockstep stays as it is with the ROS order. The controller's frame for t still reaches the virtual ECU before tick t + dt.

Not causes. The "1 or 2 beacon fires per tick" pattern: both fires of a tick publish the same payload, on ROS too (duplicate stamps in `gnss_fix.csv`), so the phase of the pattern changes nothing. The EKF: ekf runs match as well as direct runs. The GNSS stale gate and SAFE_STOP timing: within 2 ms in both modes.

### 2. ROS starts one tick late in 3 of 23 runs (inherent, handled by the comparison)

In direct_01, direct_12 and ekf_01 the first 0x101 frame reached the virtual ECU one tick after its stamp. `kinematic_sim_node._lockstep_wait()` only waits while the ECU is ACTIVE, so the very first frame races the next tick. The whole run is then 0.05 s late against its own t_rel, which is 10 cm at cruise. The playground is deterministic and never late. `reference_compare.py` lines both runs up on the first tick the shuttle moves (`onset_shift` reports the shift), which removes it. ekf_01 then matches to 1 mm.

A second start race is harmless. In 11 of 23 ROS runs the controller's actuation timer fires once before its first path loop, so one NEUTRAL + brake 50 frame goes out a tick before the first FORWARD frame. The shuttle moves at the same tick either way (direct_02 has that frame and matches to 0.0 mm).

### 3. The 10 Hz goal check quantises the end (inherent)

trajectory_controller switches to IDLE in its 10 Hz path loop once the rear axle is within the goal tolerance. A shift of a few centimetres before the end can move that check by one loop, 0.1 s, and the rest point by up to 1.84 m/s x 0.1 s = 18 cm. Seen with the ROS order on direct_03 (+0.098 s, 8 cm, after 2.5 cm of drift) and ekf_12 (+0.1 s, 4.4 cm). So the final position needs 10 cm, not 5 cm, and the track shape is judged up to the shorter driven length.

### 4. Micrometre noise in the ROS GNSS chain (minor)

The remaining 2 to 3 cm on routes 03 and 05 start at a single steering command that rounds to the other 0.01 degree (route 03, t_rel 8.5 s, 0.41 against 0.42). Before that, the ROS pose on TF is 1 to 3 µm off the true pose and the playground's is exact. The likely source is the real Lanelet2 UtmProjector round trip (map -> WGS84 in the sim, WGS84 -> map in the nodes) at the last bits, or numpy and libm differences (Humble image against numpy 1.26.4 here). Not worth chasing for the playground. It stays within the ROS spread.

## Proposed fixes

| what | owner | file | change |
|---|---|---|---|
| tick order | fakeros and runner, contract by the architect | `py/fakeros/_world.py` `World.step` (or `py/playground/runner.py` `Runner.step`), ARCH section 7 | Use the order of `ros_order.py`: clock to t, the sim ticks with its messages held, every due timer fires without delivery in between, then everything is delivered (the sim's messages first, then the timers' output, cascading). Tests in `tests/fakeros/` that pin "drain after each timer" change with it. Page commands between ticks stay as they are. Then tighten the shipped bounds in `test_against_reference.py` to the ros_order ones. |
| comparison axis | reference | `tests/reference/tools/compare_runs.py` | Line runs up on the first moving tick, report the position only while both navigate, and the track shape up to the shorter driven length, as `reference_compare.py` does. Otherwise a ROS start race reads as 10 cm. |
| tolerance | reference | `tests/reference/README.md` | Final position within 10 cm (the 10 Hz goal check), duration within 0.2 s stays. Route 11 by where it stalls. |
| first-frame race | Kane (node finding, no change made) | `nift_carla/kinematic_sim_node.py` `_lockstep_wait` | Lockstep does not cover the first 0x101 frame, so 3 of 23 runs start a tick late. Harmless, but it makes a ROS run repeat to 10 cm, not 0.5 cm. Waiting while a start is pending would make runs repeat exactly. |
| µm noise | lanelet2 (low priority) | `tests/lanelet2/` | Check `forward(reverse(p)) - p` of real Lanelet2 at the micrometre level against the stand-in's, if exact equality of steering commands ever matters. |

Also seen, not compared. The playground's very first TF (t_rel -0.15 s, before any navigation) has yaw 0 and sits 1.08 m off along x, because shuttle_tf_node gets the first fix before the first heading (the sim-gnss agent's node finding). The ROS bags do not show it. It is gone one tick later and never reaches the planner or the controller.

## The test

`test_against_reference.py` runs every case in both modes (32 tests, the fast subset is route 03 and the GNSS outage in both modes). Every run is a real CLI process. The checks:

- Both modes. The same outcome and status sequence, the same path point for point (1 µm), and for GNSS loss the "GNSS stale" line, the SAFE_STOP line and the stop time within 0.15 s.
- ros_order. Position within 10 cm while both navigate, track shape within 5 cm, duration within 0.2 s, final position within 10 cm. Route 12 by shape and final position only, as the reference README asks.
- shipped. Position within 35 cm, shape within 5 cm (20 cm for the route 11 stall, 15 cm for route 12), duration within 0.3 s, final position within 25 cm. These are the measured gap of cause 1 plus a margin, a regression guard until the tick order changes.

All 32 pass (18 s on 32 cores), and the full playground suite passes with them (5127 passed, 28 slow skipped).
