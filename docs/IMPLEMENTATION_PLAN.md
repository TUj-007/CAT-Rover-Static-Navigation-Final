# Autonomous static navigation implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Deliver a separate camera-free Raspberry Pi package that autonomously surveys reachable static space, saves a map, and navigates a user-defined route on later runs.

**Architecture:** Timestamped devices feed wheel odometry and confidence-gated LiDAR localization. Probabilistic mapping and validated pose-graph corrections support autonomous frontier surveys; saved-map localization, footprint-aware A*, and collision-checked tracking support missions. Companion Mega firmware supplies strict USB command handling and four-wheel feedback control with the corrected driver layout.

**Tech Stack:** Python 3.13-compatible standard library, NumPy, SciPy, pyserial 3.5, rplidar 0.9.2; Arduino AVR C++; local HTML/JavaScript canvas.

**Spec:** `docs/superpowers/specs/2026-10-11-autonomous-static-navigation-design.md`

## Global constraints

- Deliver everything under the new `raspberry_pi_static_nav` folder beside the existing code trees. No camera or RC receiver is required.
- The first mapping run autonomously explores reachable space; completion is based on reachable observable frontiers, not a timer. Failures save a partial map with a reason.
- Target a level, bounded, static indoor course with obstacles intersecting the LiDAR plane. Unknown and inaccessible space remain explicitly distinguishable from observed free space.
- Front wheels use TWO MD10 shields: left D3/D4, right D9/D12. Rear wheels use TWO BTS7960 modules: left D5/D6/D26/D27, right D7/D8/D28/D29.
- Wheel order is FL, RL, FR, RR; encoder A pins D18/D19/D20/D21, B pins D22/D23/D24/D25. Use A-channel RISING decoding, matching the latest firmware, and measured wheel-output counts in both controllers.
- Reported example geometry is 0.10 m wheel diameter, 0.4425 m track, 0.565 x 0.40 m body. Reported 600 counts/revolution remain unverified calibration examples.
- Survey origin defaults to [0,0,0]. Later navigation accepts an initial map pose and verifies scan alignment before moving. Never reset pose between route legs.
- Keep the static map immutable during navigation. Live obstacles occupy a separate layer.
- Reject invalid/stale sensing and uncertain localization before sending motion. Recovery intervals permit zero commands only.
- Save maps atomically with version, identity, observation mask, frame convention, keyframes, and sensor/calibration metadata.
- Hardware confirmation is explicit; simulation is explicit. No automatic flashing, fabricated hardware fallback, or silent use of unverified confirmation flags.
- The workspace root is not a Git checkout: omit commit steps and do not create a repository or commit into `CAT_Rover_codes`.

## Review focus

1. A stale LiDAR frame remains cached during reconnect: invalidate it and send zero immediately; covered by Tasks 2 and 7.
2. A symmetric room yields several similarly plausible poses: reject ambiguous corrections and prohibit motion; covered by Tasks 4 and 7.
3. A start or route file references another map or corrupt numeric data: reject before opening motion-capable devices; covered by Tasks 1 and 8.
4. No viable frontiers remain because recovery failed: save partial status rather than report full coverage; covered by Tasks 6 and 9.
5. A tight passage permits straight motion but not a pivot: test the full oriented footprint and swept turn, not just the centre point; covered by Tasks 5 and 9.

## File structure

Create `raspberry_pi_static_nav/pyproject.toml`, `requirements.txt`, `config.example.json`, `README.md`, `.gitignore`, and the `cat_rover_nav` Python package. Use these focused modules:

- `models.py`, `config.py`, `geometry.py`: shared data, configuration, SE(2) operations.
- `protocol.py`, `devices.py`, `odometry.py`: USB framing, timestamped sensor workers, encoder prediction/history.
- `grid.py`, `storage.py`: occupancy evidence, survey bounds, atomic versioned map persistence/export.
- `localization.py`, `slam.py`: scan matching, observability, keyframes, loop constraints, optimization/rebuilding.
- `planning.py`, `motion.py`: oriented-footprint A*, speed tracking, swept collision validation.
- `exploration.py`: frontier extraction, observation target selection, coverage result.
- `runtime.py`, `__main__.py`: guarded autonomous orchestration and CLI.
- `routes.py`, `web.py`, `static/index.html`: map route editing, validation, local viewer.
- `simulation.py`, `replay.py`: deterministic synthetic hardware and archived telemetry analysis.
- `firmware/rover_controller/HardwareConfig.h`, `rover_controller.ino`, `CommandParser.h`: controller settings, control loop, independently testable command parser.
- `tests/test_config_protocol.py`, `test_devices.py`, `test_grid_storage.py`, `test_localization.py`, `test_planning.py`, `test_exploration.py`, `test_runtime.py`, `test_routes.py`, `test_e2e.py`, and `firmware_parser_test.cpp`.
- `docs/WIRING_CALIBRATION.md`, `ANTIGRAVITY_AUDIT.md`, `VALIDATION.md`.

All Python commands below run from `raspberry_pi_static_nav` with an available Python executable. Use `python -m unittest discover -s tests -v`; use the bundled Python executable locally when system Python is unavailable. Create test temporary files beneath the new package when the default Windows sandbox temp directory cannot be written.

## Task 1: Shared models, configuration, geometry, and framing

**Files:** `models.py`, `config.py`, `geometry.py`, `protocol.py`, `config.example.json`, package initialization and metadata; `tests/test_config_protocol.py`.

**Interfaces:**

- `Pose(x: float, y: float, yaw: float)`; `Telemetry(ms: int, epoch: int, mode: int, fault: int, counts: tuple[int,int,int,int], received: float)`.
- `Scan(points: ndarray[N,2], angles: ndarray[N], acquired: float, completed: float, generation: int)` stores valid external rays in the rover frame; freshness uses acquisition start.
- `MatchResult(pose: Pose, accepted: bool, score: float, overlap: float, reason: str)` and `RunResult(status: str, reason: str, stations_reached: int, coverage: dict)`.
- `load_config(path: Path) -> dict`; `compose(a: Pose,b: Pose) -> Pose`, `inverse(p: Pose) -> Pose`, `transform(points: ndarray,pose: Pose) -> ndarray`.
- `frame(payload: str) -> bytes`, `parse_telemetry(raw: bytes,now: float) -> Telemetry`, `velocity_command(packet: Telemetry,sequence: int,left: float,right: float) -> bytes`.

- [ ] Write tests for nonfinite/bool/string numeric settings, conflicting ports, wrong wheel-array length, invalid bounds, uint32 wrap, malformed CRC, duplicate fields, and speeds outside +/-0.30 m/s; test SE(2) inverse/composition.
- [ ] Run `python -m unittest discover -s tests -p test_config_protocol.py -v` and verify the tests fail because the new modules are absent.
- [ ] Implement the interfaces and validated example config. Initial cruise is 0.08 m/s, wheel target limit 0.25 m/s, yaw limit 0.50 rad/s, scan age limit 0.35 s, telemetry age limit 0.30 s, resolution 0.05 m, map size 6 m; confirmation defaults false. Use a 0.90 s initial reaction budget covering the 0.35 s scan allowance plus the companion 0.30 s watchdog and scheduling margin; loaded braking must be measured. Add configurable survey bounds, calibrated occlusion sectors, match limits, and bounded run/recovery deadlines.
- [ ] Rerun focused tests; require all pass. Add dependency metadata with no OpenCV/Picamera dependency and no imports that open hardware.

## Task 2: Timestamped sensors and odometry

**Files:** `devices.py`, `odometry.py`; `tests/test_devices.py`.

**Consumes:** Task 1 models/config/framing.

**Produces:** `MegaLink.get() -> Telemetry|None`, `MegaLink.send(packet,left,right)`, `MegaLink.identity() -> dict`, `MegaLink.close()`; `Lidar.get() -> Scan|None`, `Lidar.close()`; `scan_usable(scan,cfg,now) -> bool`; `Odometry.update(packet) -> bool`, `.pose`, `.v`, `.w`, `pose_at(time: float) -> Pose|None`.

- [ ] Write fake-port tests for reset/counter discontinuity, actual signed reverse counts, scan coverage after chassis masking, near external obstacles, stale scans, reconnect generation changes, serial backlog, and immediate cached-scan invalidation.
- [ ] Run `python -m unittest discover -s tests -p test_devices.py -v`; confirm expected missing-interface failures.
- [ ] Implement bounded reader workers, finite startup/reconnect attempts, best-effort cleanup, timestamped odometry history, and fresh generations. Pin all private driver access to rplidar 0.9.2 and document it. Invalid/no-return/masked rays never establish free space. A physical footprint filter must not discard points outside the measured body. Do not relabel backlog as fresh by clearing and reusing a frame.
- [ ] Require a CRC-framed `Q` identity query and `V,2,1,1` response: protocol version 2, x1 encoder convention, layout 1 (front MD10/rear BTS). This is a documented extension to the existing T/C protocol that rejects accidental use of the Antigravity firmware.
- [ ] Rerun focused tests; require all pass.

## Task 3: Occupancy mapping and map persistence

**Files:** `grid.py`, `storage.py`; `tests/test_grid_storage.py`.

**Produces:** `OccupancyGrid(cfg)`, `update(pose: Pose,scan: Scan)`, `cell(x,y)`, `world(cell)`, `state() -> ndarray`, `observed`, `revision`, `footprint_clear(pose,cfg,layers=()) -> bool`; `save_map(path,grid,keyframes,metadata)`, `load_map(path) -> tuple[OccupancyGrid,list,dict]`, `export_map(path,grid)`.

- [ ] Test unknown-space preservation, occupied hit/free ray evidence, bounded angular interpolation only between valid nearby rays, no artificial free stopping bubble, repeated-noise removal, survey clipping reports, atomic saves, corrupt versions/shapes, nonfinite metadata, and round-trip identity stability.
- [ ] Run the focused test file and confirm initial failure.
- [ ] Implement clamped log-odds evidence and a separate observation mask. Only certify the initially occupied physical footprint, and reject scans that place it on observed obstacles. Use versioned NPZ arrays with `allow_pickle=False` plus JSON metadata; keyframe scans use numeric arrays/offsets. Hash map geometry/evidence to bind routes to maps. Atomically replace complete archives; export a PGM and JSON metadata.
- [ ] Rerun focused tests; require all pass.

## Task 4: Confidence-gated localization and pose-graph mapping

**Files:** `localization.py`, `slam.py`; `tests/test_localization.py`.

**Consumes:** Tasks 1-3.

**Produces:** `ScanMatcher(cfg).match(scan: Scan,predicted: Pose,grid: OccupancyGrid) -> MatchResult`; `Mapper(cfg).observe(scan,odom_pose) -> MatchResult`, `.grid`, `.keyframes`, `.map_from_odom`, `optimize() -> bool`; `Localizer(grid,cfg,initial_pose).observe(scan,odom_pose) -> MatchResult`.

- [ ] Test correction of known translation/yaw error, encoder bias, low overlap, outliers, corridor degeneracy, several similarly good room alignments, bounded correction limits, initial pose outside the search basin, false loop closures, and map rebuilding after a validated closure.
- [ ] Run focused tests; confirm initial failure.
- [ ] Use a distance-transform correlative search around odometry prediction, followed by robust nearest-neighbour refinement. Score independent beam subsets, require overlap and a residual threshold, and test geometric observability/competing pose hypotheses. Reject a match on inadequate confidence; do not paint it into the map. Use accepted geometry to maintain the map-to-odometry transform.
- [ ] Add motion-separated keyframes and local submaps. Validate nonadjacent loop candidates using scan alignment, odometric consistency bounds, and independent residuals. Optimize fixed-first-pose SE(2) constraints with SciPy robust least squares; reject an optimization that worsens validated constraints or exceeds bounded corrections. Rebuild the map from accepted stored scans. In navigation, match against the loaded map and never update its evidence.
- [ ] Rerun focused tests; require all pass and inspect numerical residuals, not only return flags.

## Task 5: Footprint-aware planning and motion checks

**Files:** `planning.py`, `motion.py`; `tests/test_planning.py`.

**Produces:** `Planner(grid,cfg).plan(start: Pose,goal: Pose,layers=()) -> list[Pose]`; `track(pose,path,measured_v,cfg) -> tuple[float,float]`; `safe_command(pose,left,right,scan,grid,cfg,measured_v,layers=()) -> bool`; `LiveObstacleLayer.update(scan,pose,now)`.

- [ ] Test corner-cut rejection, unknown paths, outside goals, rectangle-clear straight corridors narrower than the circumscribed circle, blocked pivots, braking horizon, side-wall false veto avoidance, new obstacle detours, and wheel target/acceleration bounds.
- [ ] Run focused tests; confirm initial failure.
- [ ] Use A* over position and 16 heading bins with forward/reverse/turn primitives; sample the full footprint along each primitive and forbid diagonal shortcuts. Penalize poor clearance/reverse motion while preserving reachability. Check current live scan points as well as unknown/occupied cells. Track only collision-checked path sections; bound acceleration with actual elapsed time and decelerate near the goal.
- [ ] Keep live obstacle evidence separate with expiry and repeated observations; never expire a currently observed obstacle. Reject motion before storing/transmitting it if any swept footprint violates map bounds, survey bounds, live obstacles, or reaction/braking clearance.
- [ ] Rerun focused tests; require all pass.

## Task 6: Autonomous frontier exploration

**Files:** `exploration.py`; `tests/test_exploration.py`.

**Produces:** `Explorer(grid,cfg).choose(pose: Pose,planner: Planner) -> Pose|None`, `report_target_failure(target,reason)`, `coverage() -> dict`, `completion() -> tuple[str,str]`.

- [ ] Test room-with-occlusion frontier selection, reachable observation poses on known free space, inaccessible components, minimum information gain, target timeout retries, and distinction between completion and exhausted failures.
- [ ] Run focused tests; confirm initial failure.
- [ ] Cluster free/unknown boundaries and score candidate observation poses by visible unknown gain divided by route cost. Place targets back from unknown boundaries to preserve footprint clearance, and validate them through Task 5 planning. Support stopped scan acquisition/clearance-checked heading changes at survey targets. Retry failed regions only when new evidence makes success plausible; track unresolved failures separately from unreachable components.
- [ ] Report observed area, reachable frontier counts, inaccessible/unknown area, survey bounds, and completion reason. A deadline or exhausted retries always yields `partial`, never `complete`.
- [ ] Rerun focused tests; require all pass.

## Task 7: Corrected firmware and guarded runtime

**Files:** firmware files, `runtime.py`, `__main__.py`; `tests/test_runtime.py`, `tests/firmware_parser_test.cpp`.

**Produces:** `run_exploration(cfg,devices,output) -> RunResult`, `run_navigation(cfg,devices,map_data,route) -> RunResult`; CLI commands `explore`, `navigate`, `diagnose`.

- [ ] Write fake-device runtime tests for zero at startup, identity mismatch, missing first scan, immediate stop on stale/unusable scan, ambiguous localization, Mega reset, stalled planner, no progress, sensor recovery, timeout, Ctrl+C, partial checkpoint, and final stop. Write native C++ parser vectors for correct/corrupt CRC, missing fields, overflowing integers, wrong epoch, duplicate sequence, stale echoes, speed limits, and mandatory first zero.
- [ ] Run focused Python tests and the C++ harness where a compiler is available; confirm initial failure.
- [ ] Implement the companion controller using the corrected pin groups and x1 encoder order, atomic reads, bounded buffers, strict parser, 300 ms command watchdog, zero handshake, query/version response, per-wheel feedforward/PI, configurable polarity/deadband, PWM rise limit, reversal delay, stall checking, and hardware/calibration gates. Stop commands bypass ramping. Keep BTS enables low at boot/stop. Do not retain unchecked M/I diagnostic bypass commands.
- [ ] Build runtime states `WAIT_SENSORS`, `LOCALIZING`, `EXPLORING`, `NAVIGATING`, `RECOVERING_STOPPED`, `COMPLETE`, `PARTIAL`, `FAULT`. Bound startup and recovery. Align scans with odometry history or reject unsupported timing/motion. Evaluate freshness/confidence immediately before transmission; reject attempts to refresh a command derived from outdated state.
- [ ] Checkpoint surveys atomically, log pose confidence and stopping reason, and close devices after sending zero. Require explicit motion/calibration confirmation for hardware commands. Expose an ordinary CLI start; do not install an automatic boot service.
- [ ] Rerun focused tests; compile `arduino-cli compile --fqbn arduino:avr:mega firmware/rover_controller` if the toolchain exists. Record unavailable checks explicitly rather than equating parser tests with an AVR build.

## Task 8: Route editing, map viewer, and input validation

**Files:** `routes.py`, `web.py`, `static/index.html`; `tests/test_routes.py`.

**Produces:** `load_route(path,grid,metadata,cfg) -> dict`, `validate_route(route,grid,metadata,cfg) -> list[list[Pose]]`; CLI `view-map --map PATH --route PATH --port 8080`.

- [ ] Test map identity mismatch, finite coordinates/headings, list ordering, negative dwell, out-of-bounds points, blocked/disconnected goals, physical footprint validation, and browser world/pixel coordinate agreement.
- [ ] Run focused tests; confirm initial failure.
- [ ] Implement a local canvas map with start click and heading drag, named intermediate clicks, endpoint selection, deletion, and planned route preview. Save only validated route JSON with map identity, start pose, and ordered goals. Bind to localhost by default; configurable Pi LAN access requires explicit bind address. Restrict file access to the chosen map/route paths and bound HTTP request sizes. The viewer cannot command motors.
- [ ] Navigation validates the route before opening devices, then verifies the real initial scan alignment before motion. Reach goals in saved order, apply optional dwell, and stop at the endpoint.
- [ ] Rerun focused tests; inspect the map/route page visually and confirm click-to-metre direction and heading conventions.

## Task 9: End-to-end simulation, migration documentation, and delivery

**Files:** `simulation.py`, `replay.py`, `tests/test_e2e.py`, README and docs.

**Produces:** CLI `simulate --scenario NAME --output PATH`, `replay-telemetry --archive PATH`; validation evidence and complete installation/run instructions.

- [ ] Write deterministic tests with finite simulated time: empty room survey, occluded static room exploration, asymmetric multi-leg saved-map mission from a nonzero start pose, biased odometry/noisy LiDAR, ambiguous localization halt, tight corridor with blocked pivot, obstacle added after mapping, and interrupted survey saved as partial.
- [ ] Run focused tests and require genuine initial failures; simulations must exercise production mapping/localization/exploration/planning/runtime, not substituted success flags.
- [ ] Implement synthetic range returns and wheel counts with configurable slip/noise, latency/dropout, and independent collision checks against geometric ground truth. Do not teleport between goals. Use recorded raw scans only if provided; the Antigravity JSONL contains telemetry but lacks raw scans and cannot validate its original map reconstruction.
- [ ] Run `python -m unittest discover -s tests -v`, `python -m compileall -q cat_rover_nav`, and each named CLI simulation. Measure coverage, independent collision count, final position error, localization rejection behavior, completion reason, and runtime. Stop optional testing after required scenarios pass unless a concrete risk remains.
- [ ] Write installation, explore/map/route/navigate commands, exact firmware upload instructions, calibration sequence, x1 count convention, corrected pin map, map/route format, survey scope, and all executed/unavailable checks. Antigravity audit records retained improvements and corrected issues, including incoming CRC validation, dropout motion, invented free space, and missing saved-map localization.
- [ ] Review delivered changes against the approved spec and this plan. Resolve actionable issues and rerun affected checks. Deliver a link to the separate folder and concise Pi commands; make no claim of physical validation or perfect operation.

## Plan self-review and execution handoff

The tasks cover all seven acceptance requirements in the approved design, including loop correction, persistence, autonomous frontiers, route editing, firmware layout, and injected failures. Interfaces and shared frame/wheel conventions are specified above. Implementation choices that affect physical operation remain explicit configuration/calibration values rather than silently inferred facts.

Recommended execution: **Native**, implementing the tasks in this session. These components depend on shared timing, frame, and confidence conventions, so a single implementer reduces integration churn. Subagent-driven implementation remains available if the user chooses it. Do not begin product implementation before the user reviews this plan and chooses the execution method, as required by the writing-plans skill.
