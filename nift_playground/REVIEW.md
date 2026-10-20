# Review of nift_playground (wave C, 2026-10-20)

Scope: `py/fakeros`, `py/fakelanelet2`, `py/fakesocketio`, `py/stubs`, `py/playground`, `tools/gen_msgs`, `tests/` (not `tests/reference/runs`). `web/` and the Pyodide worker and bundle were still being built and are not reviewed. The fakeros and fakelanelet2 parts come from two sub-reviews. The coordinator passed their findings on, and the agents that own those files are fixing them now. Those items are marked **fix in progress**. Every other item is **open**.

## Test results

- Python 3.12 (native): `python3 -m pytest nift_playground/tests -q --ignore=nift_playground/tests/reference` gave **5123 passed** in 50 s. The first run with `-x` failed `test_bundle_manifest.py::test_session_reads_only_bundled_files`, because the lanelet2 node cache was missing from `tools/bundle_manifest.txt`. The pyodide agent added it at 03:24 while the run was going, and the second run passed.
- Python 3.10.12 (`nift_pub_deps:humble`, `PYTHONPATH` unset, read-only mount): **219 passed**. This run left out `test_msgs.py`, which the msgs agent ran on 3.10 before.
- Reset gives the same run as a fresh start. I checked this with a scratch script: route 03 in ekf mode, 300 ticks after the start. A run after a `reset` and a run in a new process produced the same rows for every trace table (odom, pose_tf, cmd, dbw, telemetry, gnss_fix, loc_status) and the same page messages.
- No leak across resets. Over 12 resets the count of live Python objects stayed at 117,729. The hub had 1 server and 0 errors after each one. The world's timers, topics and nodes were rebuilt and did not pile up.

## Findings, by severity

### Critical

**C1. Obstacle messages use `id`, but ARCH and the page use `oid`.** Status: open. Owner: runner.
`py/playground/session.py:249`, `:260`, `:273`, `:278` read `msg['id']`, and `:146` writes `{'id', 'poly'}`. `runner.py:217` sends `polygons` as `{'id', 'poly'}`. `scenario.py:33` (`KEEP_ID`) keeps `id` in the log for that reason. ARCH 8.1 and 8.3 (changed on 2026-10-20) and `web/src/protocol.ts:46`, `:132`, `:139`, `:147` use `oid`. I tested what happens. An `obstacle_add` with `oid` and a reply `id` names the obstacle after the reply id, so a later move or remove cannot find it. Without a reply id it fails with `KeyError: 'id'`. The page also gets `polygons` with no `oid`. Nothing about obstacles works from the page, and `test_session.py:142-155` did not catch it because it sends `id`.
Fix: read and write `oid` in session, runner (`obstacles_message`) and `set_obstacles`. Then remove `KEEP_ID` so `strip()` always drops the reply `id`. Change the tests to `oid`. Fix ARCH 8.2, which still says the `obstacles` polygons carry "`id`" (the architect owns that line).

**C2. `fakelanelet2/lanelet2/_cxx.py` is a line-by-line translation of GPLv3 libstdc++ code** (sort, nth_element, hashtable iteration order). Status: fix in progress (a clean-room `_stdorder.py` is being written from `CLEANROOM_SPEC.md` by an independent implementer). Owner: lanelet2. See the licence assessment below.

### High

**H1. A bad `set_param` value breaks the session for good.** Status: open. Owner: runner.
`session.py:363` stages any value without checking it. `_on_reset` (`:373-379`) applies the staged values. `_build` (`:134-141`) tears down the old runner before it builds the new one. I tested `set_param waypoint_node map_publish_period_s = 'fast'` and then `reset`. fakeros raised `InvalidParameterTypeException`, as Humble would. After that `self.runner` was `None`, and every later `reset` got "not initialised". `self.staged` is never cleared on failure, so the page cannot recover with a reset. Only an `init` works, because `init` ignores the staged values. The comment at `:130` says the stack in force keeps running, but that holds only for a `LaunchError`.
Fix: if `runner.build()` raises, drop the staged values and rebuild with the previous options (the world is a singleton, so build again rather than keep the old runner). Report the error with `fatal: false`. Better still, check each staged value against the declared parameter's type when `set_param` arrives (fakeros knows the types once the node is built). Add a test.

**H2. No licence texts ship with the Lanelet2 stand-in** (BSD-3 Lanelet2, BSL-1.0 boost R-tree, MIT GeographicLib). Status: fix in progress (a hardening agent is adding `LICENSES/` and file headers). Owner: lanelet2.

**H3. The runner's tick order differs from ROS** (found by validation). The controller sees the GNSS of the same tick here. On ROS it sees the GNSS from two ticks earlier. Status: fix in progress (fakeros is moving to the ROS order). Owner: fakeros and runner. ARCH 7 needs the same change (architect).

### Medium

**M1. `*.csv` is in the root `.gitignore` (line 23).** So `tests/reference/index.csv` and every `tests/reference/runs/**.csv` will never be committed, and the validation tests will fail on a fresh clone. Status: open. Owner: validation and Kane. Fix: add `!nift_playground/tests/reference/index.csv`. Gzip the runs, as the log already plans (they are 30 MB now, with several `loc_status.csv` files of 1.2 to 2.8 MB each), or keep them out of git and say where they come from.

**M2. `_utm.py` hypot was ported from glibc (LGPL).** Status: fix in progress (being re-derived from Borges, arXiv:1904.09481). Owner: lanelet2.

**M3. Lanelet2 ties were only checked natively, with a compare that tolerates ties.** Status: fix in progress (a strict mode is being added, including Pyodide). Owner: lanelet2.

**M4. A node cache miss is silent in Pyodide.** Another map or origin falls back to browser maths, and ties may then break differently with no warning. Status: fix in progress. Owner: lanelet2.

**M5. fakeros `_params.py` list parameters are shared across resets through `_FILE_CACHE`.** A node that changes a list it got from a parameter changes it for the next run too. Status: fix in progress. Owner: fakeros.

**M6. fakeros `_logging.py`.** A call site that changes severity does not raise as Humble does. Status: fix in progress. Owner: fakeros.

**M7. `tools/gen_msgs` depends on a local Docker image** that is not documented. The same goes for `nift_pub_deps:humble` in `tests/lanelet2/compare_real.py`. Status: fix in progress. Owner: msgs and lanelet2. A reader of the public repo cannot rebuild `spec.json` or the fixtures. Write down how to build the image, or give a public base image and the pip and apt steps.

**M8. A scenario that is broken JSON gives a fatal error.** `scenario.py:80` calls `json.loads` without a try, so a `JSONDecodeError` escapes. `session.py:394` catches only `ScenarioError` and `LaunchError`, so `handle_obj` reports a Python error with `fatal: true` for a bad URL hash. Status: open. Owner: runner. Fix: wrap the `json.loads` calls in `decode()` and turn errors into `ScenarioError`. Also check that the decoded value is a dict and that event ticks are `>= 0`.

### Low

- **L1. Unknown parameters of a live sim fault are not checked.** `sim_backend.py:170-179` checks unknown keys only for link presets. For sim presets an unknown key reaches `faults_from_dicts`. A `TypeError` from there is not a `ValueError`, so `session.py:302` lets it out as a Python error instead of a refused command. Owner: sim-gnss. Fix: check against `SIM_PRESETS[preset][0]` the same way, and catch `TypeError` too.
- **L2. `--param` without `=` is accepted.** In `cli.py:315-318` the second `partition` reuses `_`, so the check at `:317` tests the `.` and not the `=`. `--param a.b` gives the value `None`. Owner: runner. Fix: use separate names and check `'=' in p`.
- **L3. `launch_set.faults_arg` writes a fixed file in the temp folder** (`launch_set.py:217`). Two native runs at once (pytest-xdist, two CLIs) overwrite each other's faults. Owner: runner. Fix: one folder per Session, made at a fixed relative name under a per-process temp folder, while the path that is echoed stays the same.
- **L4. `observer.planner_warnings` grows with no limit during a run** (`observer.py:86`, `:132`). Only the last one is ever read. Owner: runner. Fix: keep a count and the last text.
- **L5. `os.environ.update(...)` in the runner** (`runner.py:126`) leaves the five `MCITY_OCTANE_*` keys in a native test process after the run. It is harmless now but can surprise later tests. Owner: runner.
- **L6. Notes on `tests/reference/README.md:118`.** It says beacon_node's disconnect handler fails because python-socketio 5.17 passes a reason. The 5.17 client retries a `disconnect` handler without the reason when it raises `TypeError` (`socketio/client.py:450-453`), and the fake copies that (`hub.py:113-119`). So the fake matches the real library. The error seen at shutdown has another cause, most likely the retry's call to `get_logger().warn` after `rclpy.shutdown()`. Owner: reference. Fix the note before it goes to Kane as a node bug.
- **L7. Lanelet2 stand-in small items** (status: fix in progress, owner lanelet2): `core.py:479` layer id order against `unordered_map`, `LineString3d` hashed by `id()`, the length check in `_nodecache`, a crash on a `None` bbox, the range check in `reverse`, the module-level `_next_id`, and dead code.
- **L8. fakeros small items** (status: fix in progress, owner fakeros): `ParameterUninitializedException`, QoS depth, `use_sim_time` ignored, filter order, `_LEVELS` not reset, destroyed nodes still usable, tf2 same-frame stamp and prune order, and `FixedArray` float handling.

### Leftovers and dead code (open)

- `fakesocketio/socketio/hub.py:53-54`. This is the "wave A world has no call_soon yet" fallback. `World.call_soon` exists now (`_world.py:94`). Delete it. Owner: sim-gnss.
- `runner.py:28-56`. These are thin wrappers (`get_world`, `world_set_node`, `world_set_log_sink`, `make_sim_backend`) from before fakeros landed. The comment at `:103` says "spike core". Fold them in. Also drop the unused `perceived_only` argument at `:211`, and reset `wall_s` in `teardown()` (`:159`). Owner: runner.
- `fakeros/_core.py`, the spike's `World.run_until` (`_world.py:194`) and the "spike form" `world.params` (`_world.py:27`, `:75`, `_node.py:12`) are only kept for `tests/spike/`. pytest does not run `tests/spike/run_spike.py` (its name does not start with `test_`). Either turn it into a `test_spike.py` or delete `tests/spike/` and these paths. Owner: architect and fakeros.
- `session.py:75` `obstacle_yaw` is not cleared on `scenario_load` (`:396-399`). It is harmless today, since every replayed obstacle sets its yaw again. Clear it with the obstacles.
- ARCH 5 (line 233) says the real python-dotenv may shadow the stub when it is installed. `paths.setup` puts `py/stubs` first, so the stub always wins. Fix the sentence. Owner: architect.

## ARCH and the "nodes unchanged" rule

The only things that touch repo code are the ones ARCH 2.1 lists. `sim_backend.py:75-80` swaps `kinematic_sim_node.OctaneEmulator` and puts it back. The wrapper calls `_tick_once` and reads `_sim_ns`, `dt`, `gnss`, `car`, `plant`, `ecu`, `_dbw_status` and `octane`. `schedule_fault` replaces `node.gnss` with a copy, as ARCH 8.4 allows. The only parameter override of a repo node is `waypoint_node map_publish_period_s` (`launch_set.py:42`), which ARCH 6.3 allows and the page shows. I found no other monkeypatch of a repo module. The operator, observer and perception nodes have no timers (except perception, which runs only with its option), and the observer publishes nothing. So they do not change the stack's timing.

## Pyodide

The reviewed Python starts no threads and opens no sockets. It does not need any C extension except numpy and pyyaml. `KinematicSimNode._thread` is built but never started. The temp file (L3) and the node cache work in Pyodide's MEMFS once the cache is in the bundle (fixed at 03:24). `cli.py` and `Trace.write` use files, but they run natively only.

## Publication checklist (README section 3)

- **Secrets.** I ran the section 3 `git grep` on the scope. Every hit is harmless: the sim key `sim` from `carla_params.yaml`, `x-api-key` protocol names, and a fake `secret` in `test_bundle_manifest.py:159` that tests the `.env` refusal. No `.env` files exist, and `build_bundle.py` refuses them.
- **Machines and networks.** There are no IPs except `127.0.0.1`, no hostnames, no tailscale addresses and no home paths in the reviewed files. The Octane URL does not appear. The server is `http://127.0.0.1:<port>`, from `carla_params.yaml`.
- **"Mcity" in names.** The only uses are the `MCITY_OCTANE_*` and `MCITY_*_BEACON_ID` environment names, which come from `octane_client.py` in the repo, and the MIT credit link in `README.md:137`. No file, package or identifier of the playground's own is named Mcity.
- **Size.** Every untracked file in scope is under 1 MB. The largest are `golden.json` (560 KB), the lanelet2 example map (584 KB), the node cache (544 KB) and the fixtures (428 and 120 KB). `tests/pyodide/node_modules` (with a 10 MB wasm) is ignored by `tests/pyodide/.gitignore`. `tests/reference/runs` is 30 MB and is ignored by `*.csv` (see M1).
- **Cache folders.** `__pycache__` and `.pytest_cache` in the tree are ignored by the root `.gitignore`.
- **Licence headers.** The repo's own files carry no headers (Apache-2.0 in the root `LICENSE`), and the playground follows that. Third-party derived code is the exception. See below.

## Licence assessment of the Lanelet2 stand-in (for Kane)

What it contains:

1. **Ports of Lanelet2's own logic** (BSD-3-Clause). These cover the centerline, routing graph, traffic rules and projection behaviour. BSD-3 allows this in Apache-2.0 code if the copyright notice and the licence text go with the source. Today they do not (H2), so a fix is needed. It is in progress.
2. **The boost R-tree insertion and query order** (`_rtree.py`, Boost Software License 1.0). BSL is permissive. Its terms ask that the licence text go with copies of the source, all or part. A Python port of the algorithm is a derived work in source form, so ship `LICENSE_1_0.txt` and say in the file header which boost files it follows. Low risk once that is done. Fix in progress.
3. **libstdc++ algorithms** (`_cxx.py`: introsort, nth_element, hashtable bucket order; GPLv3 with the GCC Runtime Library Exception). The exception covers target code made by compiling against libstdc++. It does not cover source that is translated from it. A line-by-line Python translation is a derivative of GPLv3 source, so it cannot sit in an Apache-2.0 repo under Apache terms. **This is the one real problem (C2).** The fix in progress is right: write a short spec of the observable behaviour (which element order the C++ calls produce) and have someone who has not read the libstdc++ source implement `_stdorder.py` from it. Then delete `_cxx.py` and drop it from all history before any push. Keep the spec and the fact that the work was clean-room in the repo, so this can be shown later. Matching the behaviour is fine, because behaviour is not protected.
4. **glibc hypot** in `_utm.py` (LGPL-2.1+). The same reasoning applies on a smaller scale. A translated glibc routine is LGPL source. Re-deriving it from the published paper (Borges 2019) is the right fix and is in progress.
5. **GeographicLib's UTM series** (MIT). This is permissive, and only the notice is needed (H2).

Verdict: after the clean-room `_stdorder.py` replaces `_cxx.py`, `hypot` is re-derived, and `LICENSES/` holds the BSD-3, BSL-1.0 and MIT texts with per-file headers naming the origin, the stand-in can ship under Apache-2.0 at low risk. Before that, do not push it. Since nothing has been committed yet, `_cxx.py` never enters git history if it is deleted before the first commit. `README.md:139` should then say "re-implements from a written spec of the behaviour" and point to `LICENSES/`.

## Test gaps (open, owner runner unless noted)

- No test sends obstacle messages with `oid` (C1), and none checks the `obstacles` message shape against `protocol.ts`.
- No test covers a reset that fails at node construction, or recovery after it (H1).
- `test_replay_is_exact` does not cover `obstacle_move` with yaw, box obstacles, or obstacles that survive a reset.
- Reset against a fresh process is checked only indirectly. Add the check I ran as a test (run N ticks, reset, run N ticks, compare with a new Session).
- `tests/spike` is not collected (see leftovers).
- Scenario decode of bad input (M8).
- Lanelet2 strict tie mode in Pyodide (M3, in progress).
