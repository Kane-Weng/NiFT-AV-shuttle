# ekf + real GNSS feed: SAFE_STOP on route 04

The question (pyodide agent, 2026-10-20). With `--localization ekf --gnss-feed real` route 04 ends in SAFE_STOP about 27 s after the start, natively too. Is that the playground, or does the real ROS stack do the same?

**Answer.** The real stack does the same, at the same sim time and with the same log lines. It is stack behaviour, not a playground bug. The real-feed model in `sim_gnss.py` draws a 2.04 s outage of both antennas with seed 1, and that is longer than the EKF can bridge (`bridge_s` 1.5 s).

## What happens (seed 1, route 04)

All times are sim seconds. They do not depend on when navigation starts (see below).

| t | where | what |
|---|---|---|
| 28.50 | `sim_gnss.GnssSim` | a bad-link spell starts (`burst_per_min` 2.1, lasts to 39.77). Gap onsets go up x15 (`burst_factor`), to 1.2 per s per antenna while moving, 0.42 per s of them shared (`gap_shared_fraction` 0.35) |
| 28.549 | `sim_gnss.GnssSim` | a shared gap starts, both antennas, 2.036 s long, to 30.585, with backlog (`hold_fraction` 0.9). Lengths are lognormal, median 0.23 s, sigma 0.72, so 2.04 s is z = 3.0, about 1 onset in 800 |
| 28.55 | Octane | last deliveries: rear stamped 28.497, front stamped 28.539 |
| 29.00 | `localization_node` | no accepted position for `ok_age_s` 0.5 s, so DEGRADED bridging. Last position measured at 28.539 - `gnss_delay_s` 0.061 = 28.478 |
| 29.05 | `beacon_node` | `gnss_timeout_s` 0.5: "GNSS stale (front 0.50 s, rear 0.55 s)", /gnss/* paused |
| 29.10 | `trajectory_controller` | target speed capped at `degraded_speed_cap_mps` 1.0. The pedal ramps 255 to 218 in 1 s, no brake. The true speed only drops from 1.965 to 1.92 m/s, and the filter's speed stays 1.95 m/s |
| 30.00 | `localization_node` | age 1.522 s > `bridge_s` 1.5 s: LOST outage (2.97 m driven, `bridge_m` 3.0 m would have tripped 15 ms later) |
| 30.05 | `trajectory_controller` | "Localization unusable while AUTONOMOUS (LOST outage, data 1.522 s old, 2.97 m bridged) -> SAFE STOP" |
| 30.606 | Octane | the feed is back, 0.55 s too late, first with held (old) epochs. beacon_node drops a rear delivery (baseline) and re-anchors at 30.80 |

With `localization:=direct` the same gap stops the shuttle earlier. beacon_node pauses /gnss/* at 29.05 and the controller's `gnss_timeout_s` 0.3 s gives "GNSS lost while AUTONOMOUS (last /gnss/vel 0.35 s ago)" at 29.35. The ideal feed has no gaps, so neither mode stops there.

At cruise (1.965 m/s) `bridge_s` 1.5 s and `bridge_m` 3.0 m trip within a few ms of each other. So the EKF rides through a full outage of about 1.44 s (1.5 s minus the 0.061 s delay), and the 1.0 m/s cap has no time to slow the shuttle before LOST.

## Evidence that ROS does the same

Docker `nift_pub_deps:humble`, workspace from the public repo at `902436a` (clean), `sim_route_run 04` with `localization:=ekf gnss_feed:=real gnss_seed:=1` (and direct, and other seeds). `ros_hook/sitecustomize.py` logs the feed model inside `kinematic_sim_node` without changing it.

- **The feed is bit-identical.** Up to sim 30 s, the 886 deliveries (antenna, step, stamp) and every gap onset (start, length, backlog) are the same as in the playground. That holds at time_factor 1 and 5, ekf and direct, although navigation starts at sim 2.95 s in the playground, 3.6 to 6.8 s on ROS at time_factor 1 and 12.85 s at time_factor 5. The random stream depends on sim time, not on the route.
- **Same outcome, same lines, same sim time.** ROS ekf: stale 29.051, SAFE_STOP 30.051 with "data 1.522 s old, 2.97 m bridged". ROS direct: stale 29.051, SAFE_STOP 29.351. The playground gives 29.05, 30.05 and 29.35. All 10 WARN/ERROR lines match.
- **Other seeds match too.** Seed 6 stops at 32.95 ("1.514 s old, 3.00 m") and seed 20 at 27.40 ("1.547 s old, 3.04 m") on ROS and in the playground. All 15 and 14 lines match in sim time, apart from one beacon age printed one 50 ms tick apart.
- **Trajectory.** `reference_compare.py` against the new reference gives 1.4 cm while both navigate and 0.4 cm shape. On its t_rel axis the stop is 3.85 s off, only because the two runs start navigating at different sim times. With the playground start delayed to the reference's sim time (`START_DELAY_S=4.80`), it matches to 1 mm while navigating, 5.5 cm final, SAFE_STOP within one tick (0.049 s).

## How often (playground, route 04, seeds 1 to 30)

| mode | arrived | SAFE_STOP |
|---|---|---|
| ekf | 21 | 9 |
| direct | 9 | 21 |

The direct rate fits the model's calibration in `FeedConfig` ("SAFE_STOPs with today's 0.3 s watchdog 2.1 per minute of driving"). The EKF cuts it to about a third. Seed 1 is the plain kind: one long shared gap. Most of the other eight EKF stops are a second kind. Short per-antenna gaps overlap, and one antenna's backlog then sends old epochs next to the other's fresh ones. beacon_node drops deliveries ("delivery dropped: baseline") and the filter has no accepted position for 1.5 s, although deliveries keep arriving. All of it is repo code. Seeds 6 and 20 are of this kind, and ROS matches them.

## For Kane

- Not a bug in a node, and not the playground. The real-feed model is meant to reproduce the garage bags, including their stops. Whether the EKF should ride through more is a tuning question: `bridge_s` and `bridge_m` (localization.yaml) set the ride-through, and the feed checks set how fast it comes back after a backlog. The 1.0 m/s DEGRADED cap does not act within the 1.5 s window at 2 m/s.
- On ROS the place on the route where a real-feed stop happens depends on how long the stack takes to start (wall clock), because the seed fixes the gaps in sim time. Two ROS runs with the same seed can stop at different places, or one can arrive.

## For the other agents

- **validation.** For real-feed cases compare on sim time (`t`), not `t_rel`, or start the playground at the reference's sim time (`gap_probe.py` with `START_DELAY_S`). `reference_compare.py`'s onset alignment cannot line up a fault that is fixed in sim time.
- **reference.** `runs/ekf_realfeed_04` is recorded (time_factor 1, seed 1, `record_reference.sh` without the hook). It is not in `index.csv.gz` or the README tables yet, as those are yours. Two small things in `extract_reference.py` events. In two runs a /rosout line came out 0.5 to 3 s late in `events.csv`, while `launch.log`'s wall stamps put it at the playground's time (receive-time mapping). In a time_factor 5 run the bag held only 1 /rosout message, so `events.csv` was empty while `launch.log` had the lines.

## Files and commands

```
README.md            this report
gap_probe.py         playground.cli plus a log of the feed model (gaps, spells, deliveries); START_DELAY_S delays /start_navigation
ros_hook/sitecustomize.py   the same log inside kinematic_sim_node on ROS (PYTHONPATH + GNSS_PROBE)
record_probe.sh      record_reference.sh with the hook; writes to <bag dir>/extracted/<label> and <bag dir>/<label>.probe.jsonl
```

```bash
cd nift_playground/py
python3 -m playground.cli --route 04 --localization ekf --gnss-feed real          # SAFE_STOP after 27.1 s
python3 ../tests/investigations/ekf_realfeed/gap_probe.py /tmp/probe.json --route 04 --localization ekf --gnss-feed real
START_DELAY_S=4.80 python3 ../tests/investigations/ekf_realfeed/gap_probe.py /tmp/p.json --route 04 \
  --localization ekf --gnss-feed real --out /tmp/ekf_realfeed_04
python3 ../tests/validation/reference_compare.py - /tmp/ekf_realfeed_04

# ROS (workspace as in tests/reference/README.md)
nift_playground/tests/investigations/ekf_realfeed/record_probe.sh $WS/ws $WS/bags ros_ekf_s1 04 \
  --time-factor 1 --extra "'localization:=ekf gnss_feed:=real gnss_seed:=1'"
```
