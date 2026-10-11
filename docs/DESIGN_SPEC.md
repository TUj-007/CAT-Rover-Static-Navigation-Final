# Autonomous static mapping and navigation design

Status: approved by the user on 11 October 2026; implementation planning underway.

## Requested outcome

Create a separate `raspberry_pi_static_nav` package for the existing Raspberry Pi,
RPLIDAR A1M8, and Arduino Mega rover. No camera is used. The first mapping run
must autonomously explore the reachable environment without keyboard driving,
save a static map, and report the extent actually surveyed. The operator then
defines a start pose, ordered intermediate points, and an end point on that map.
Later runs localize against the saved map and navigate the route autonomously.

The user corrected the hardware arrangement: two Cytron MD10 shields control
the FRONT wheels; two BTS7960 modules control the REAR wheels. The latest summary
reports that the RC receiver has been removed. This design therefore uses USB
control directly and does not require RC arming.

An explicit command starts exploration or navigation. Autonomy concerns survey,
planning, motion, and waypoint arrival; it does not imply driving automatically
on boot or actuating excavation equipment.

## Supported environment and completion definition

The initial implementation targets a level, bounded, static indoor course whose
walls and obstacles intersect the horizontal LiDAR scan plane. The mapping run
may contain only boundary walls, as requested, but the algorithms must also
handle static interior obstacles, corridors, and occluded space. Usable passages
must fit the measured rover envelope and stopping allowance.

"Complete map" means all reachable, observable survey space inside the configured
survey boundary has been explored to the configured resolution. It cannot mean
rooms behind closed doors, areas beyond sensor range with no usable returns,
or space physically inaccessible to the rover. Completion requires no remaining
reachable frontier with sufficient expected information gain. A timeout, sensor
failure, localization failure, or exhausted recovery attempts produces a partial
map with an explicit reason, never a successful completion message.

Symmetric rooms, long featureless corridors, and open areas can make LiDAR
localization ambiguous. The rover must remain stopped when the available geometry
does not support a sufficiently confident pose estimate. Physical performance
requires measured calibration and tests on the actual rover.

## Architecture

Use standalone Python compatible with the current Raspberry Pi Python 3.13
environment. Use NumPy and SciPy for numerical operations, pyserial for the Mega,
and the pinned RPLIDAR adapter. A browser map page uses the Python standard library
HTTP server and a small local HTML/JavaScript client; it has no camera dependency.
Simulation and recorded-data replay use explicit separate device implementations.

The pipeline is: timestamped telemetry and LiDAR scans -> wheel odometry prediction
-> LiDAR scan matching and confidence checks -> map-frame pose -> occupancy mapping
or fixed-map localization -> exploration target or mission target -> global path
planning -> footprint-aware motion checking -> fresh bounded wheel-speed commands.

Keep wheel odometry and map-frame pose separate. A localization correction updates
the map-to-odometry transform; it does not manufacture encoder movement. During
mapping, keyframes and their relative constraints support loop-closure correction
and map rebuilding, rather than permanently painting a drifting odometry map.

## Mapping and localization

Exploration begins at a measured launch pose, defined as map origin with forward
heading zero by default. Collect usable stationary scans before permitting motion.
Only the physical footprint already occupied by the rover is initially certified
clear. Do not certify its stopping envelope or a surrounding blind region as free.

Use probabilistic occupancy updates with separate unknown, free, and occupied
states. Valid rays establish observed free space; hits establish occupied space.
Repeated observations can remove isolated noise. Invalid, masked, or no-return
rays do not establish free space. Survey bounds and map dimensions are explicit;
out-of-map observations do not silently disappear while reporting completion.

Predict each scan pose from calibrated signed encoder motion. Match scans to
local submaps with bounded translation and rotation search, robust residual
rejection, and overlap/confidence tests. Track accepted keyframes; validate loop
closure candidates with independent geometric agreement and pose consistency.
Optimize the keyframe pose graph and rebuild occupancy from stored scans after
accepted corrections. A failed match must not be inserted into the map.

Persist the map, origin, resolution, frame convention, observation mask, keyframe
data, and sensor/calibration metadata together. Use atomic saves and versioned
formats. Export a human-readable map image and metadata. Retain partial survey
checkpoints so an interrupted run can be inspected.

On later navigation runs, load the static map without modifying it. Accept a
user-provided initial map pose and confirm it against fresh LiDAR before movement.
Do not assume encoder zero is the saved map origin. Navigation scan matching
updates localization; a separate live obstacle layer handles objects absent from
the original empty-room survey.

## Autonomous exploration

Find frontiers between known free and unknown cells, group them into regions,
and select a reachable observation pose on known free space. The target must be
far enough from obstacles and unknown space to fit the rover footprint. Score
targets using expected visible unknown area, path cost, and previously failed
attempts. Do not command the rover directly into an unknown frontier cell.

Plan only through observed free space. At a target, stop and acquire scans from
additional headings when the rotation footprint is clear. Recompute frontiers
after new observations. Maintain progress deadlines, target failure counts, and
bounded replanning attempts. Temporary failures may be retried after the map
changes; unreachable space is reported separately.

Completion is evaluated from the map and frontier reachability, not elapsed time
or proximity to an arbitrary waypoint. A configurable survey boundary limits
where the rover may travel during autonomous mapping.

## Route definition and execution

The local browser page shows the saved map with coordinates in metres. The
operator selects a start position and heading, any number of named intermediate
points, and an end point. Save the route separately from the map with the map
identity and frame metadata. The CLI also accepts an equivalent route JSON file.

Validate every pose and waypoint against map bounds, known space, rover footprint,
and connectivity. Show the planned route before starting. A selected start pose
is an initial localization estimate, not an instruction to teleport the rover or
an assertion that it is already there. The operator places the rover accordingly,
then live scan alignment confirms the estimate.

Navigate all route legs continuously without resetting pose between waypoints.
Use A* with diagonal corner-cut rejection and clearance costs, followed by a
path tracker with measured speed feedback. Test candidate commands with the
oriented rectangular footprint over the full reaction and braking horizon.
Pivoting requires clearance for the entire swept footprint. Apply speed and
acceleration limits and slow the final approach. Announce and log waypoint arrival;
stop at the end. Optional per-waypoint dwell is explicit in the route.

If a newly observed obstacle blocks the route, stop immediately and replan through
known clear space. If no route remains, wait stopped for a bounded interval and
finish with a clear failure state. Loss of localization also stops motion; it
does not authorize speculative movement to recover localization.

## Sensor and command behavior

Retain stable USB identifiers from the Antigravity configuration as examples.
Validate actual port availability and configuration before starting. Bind scan
timestamps to acquisition intervals and interpolate odometry where possible;
stop mapping/motion when motion or timing makes scan distortion unacceptable.

Discard stale scans and telemetry. LiDAR recovery may reconnect the scanner,
but must invalidate its previous scan, command zero while recovering, and require
fresh usable observations before resuming. Stop immediately on the first unusable
scan condition; a retry grace period only controls when the run is aborted.

Chassis filtering applies only to measured sensor occlusions/physical self hits.
Filtered rays do not count as observed free sectors. Check real angular coverage
and retain genuine close external obstacles. Do not use a blanket proximity
circle that incorrectly vetoes a clear command near a side wall; evaluate the
actual footprint and swept motion instead.

Bound startup waits. Reject counter resets, implausible deltas, stale or malformed
telemetry. Stop before closing device workers on errors, Ctrl+C, and termination.
Only the motion loop can refresh nonzero commands; an independent heartbeat must
not sustain motion after a stalled planner.

## Companion Mega firmware and corrected motor layout

Include the required companion firmware under
`raspberry_pi_static_nav/firmware/rover_controller/`, because Pi-only changes cannot
correct driver addressing or add incoming command validation on the Mega.

Proposed signal allocation, preserving each existing driver's pin group:

| Physical wheel | Driver | Signals |
| --- | --- | --- |
| Front-left | MD10 | PWM D3, DIR D4 |
| Front-right | MD10 | PWM D9, DIR D12 |
| Rear-left | BTS7960 | RPWM D5, LPWM D6, enables D26/D27 |
| Rear-right | BTS7960 | RPWM D7, LPWM D8, enables D28/D29 |

Preserve encoder wire order FL, RL, FR, RR: A pins D18/D19/D20/D21 and B pins
D22/D23/D24/D25. Verify these labels physically before enabling motion. Preserve
the latest firmware's A-channel RISING decoding convention unless recalibrated;
counts per wheel revolution must match that convention on both controllers.

Maintain CRC-framed telemetry and signed left/right speed commands. Strictly
validate command CRC, field count, numeric bounds, sequence, epoch, and echoed
telemetry age. Reject partial/malformed commands without refreshing the watchdog.
Require a zero command handshake before accepting motion and after a timeout.
Keep outputs zero at boot; enable BTS channels only when a valid command needs
motion. Zero commands and timeout stops bypass the upward PWM ramp.

Regulate all four wheels independently with calibrated encoder feedback,
feedforward, and bounded PI correction. Retain configurable soft acceleration
and reversal pause. Treat Antigravity's 130-240 PWM deadband mapping as an observed
calibration lead, not a guaranteed speed model. Expose per-wheel motor and encoder
polarity, measured counts, PI/feedforward parameters, and current-compatible PWM
limits. Report stall and implausible encoder faults. No RC receiver is required.

Hardware confirmation and calibration values remain explicit; the package does
not auto-flash a device or silently reuse unverified confirmation flags.

## Deliverables

- New self-contained `raspberry_pi_static_nav` directory with CLI exploration,
  map inspection/route editing, saved-map navigation, diagnostics, and simulation.
- Corrected companion Mega firmware and an exact wiring/calibration reference.
- Configuration examples using the reported 0.10 m wheels, 0.4425 m track,
  and 0.565 x 0.40 m body, clearly marked for verification. The reported 600
  encoder counts remain calibration placeholders until physically measured.
- Map and route formats, installation/run instructions, migration notes explaining
  firmware incompatibilities, and an Antigravity change audit.
- Regression tests and deterministic end-to-end scenarios; recorded-data replay
  can inspect the supplied telemetry but cannot reconstruct missing raw LiDAR.

## Acceptance evidence

1. Autonomous simulation surveys an empty enclosed room and a room containing
   static obstacles/occlusions, saves a map, and reports completion accurately.
2. Restart from a specified nonzero map pose; localize against the saved map and
   reach ordered intermediate points and the endpoint without pose resets.
3. Inject wheel odometry bias/slip, noisy ranges, ambiguous scans, bad initial
   pose, sensor dropouts, and counter reset. Verify corrected poses when observable
   and zero commands when confidence is inadequate.
4. Exercise tight but traversable passages and swept pivots with the measured
   rectangular footprint. Verify genuinely impassable passages remain blocked.
5. Insert an obstacle after mapping. Verify the saved map stays unchanged, the
   rover stops before collision, and a valid alternative route is used if one
   exists. Otherwise report no route while stopped.
6. Verify timeout and Ctrl+C leave zero commands; corrupt/duplicate/stale firmware
   frames cannot command motion or refresh the watchdog. Compile the Mega sketch
   for `arduino:avr:mega` when a supported toolchain is available.
7. Report what was executed locally and what needs physical rover verification.
   Simulation success is not a claim of perfect hardware navigation.

## Repository status

The workspace root is not a Git repository. `CAT_Rover_codes` is a separate Git
checkout, but the requested new package belongs alongside it. Design and package
files will be saved at the workspace root without creating a new Git repository
or committing into the older code tree.
