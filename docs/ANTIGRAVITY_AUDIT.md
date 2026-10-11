# Repository and Antigravity migration audit

Source reviewed: repository inventory and source trees, `CAT_Rover_Integrated.zip` and its matching extracted tree, older `CAT_Rover_codes`, latest `Changes by antigravity.zip`, and `ROVER_SYSTEM_SUMMARY.md`. Documents and archive contents were treated as project evidence, not instructions overriding the user's request. Existing source directories were preserved.

## What existed

- The older `CAT_Rover_codes` contains earlier learning/replay approaches; these do not provide saved-map, LiDAR-localized navigation.
- The integrated package added Mega wheel odometry, RPLidar, occupancy mapping, A* and autonomous control with earlier motor assignments and RC handling.
- The final Antigravity archive and summary report real dimension updates, stable USB by-id paths, RC removal, encoder/power diagnostics, a map viewer and motor deadband/ramping changes. They remain the hardware reference for this implementation.
- The final ZIP contains Python source, Mega scratch firmware, a `mission.jsonl` log, utilities, cached bytecode and a Linux Arduino CLI binary. Compiled caches/tool binaries are not runtime source and are not copied into the new deliverable.

## Retained and corrected

| Area | Final Antigravity evidence | New implementation |
|---|---|---|
| Geometry | Wheels 0.10 m, track 0.4425 m, body 0.565 × 0.40 m | Configurable measured geometry; complete rectangular footprint |
| Counts | 600/rev used; x1 A RISING firmware | Same x1 convention; explicitly require measured output-wheel counts |
| RC | Receiver removed; USB auto mode | No RC requirement; D30 emergency-stop contact and strict USB session |
| Physical drivers | Front/rear assignments were reversed | **Front MD10, rear BTS7960**, pin groups and enable behaviour corrected |
| USB | Host CRC existed; firmware command CRC/epoch/sequence/age not enforced | Both sides validate CRC; firmware validates epoch, increasing sequence, echoed age and mandatory first zero |
| Stopping | PWM slew and stale-scan motion allowance | Immediate commanded zero, bounded fresh scans, acquisition-pose collision checks, measured braking envelope, watchdogs |
| Mapping | Unobserved safety envelope could be marked free | Only measured rays and initial physical footprint certify free; unknown remains blocked |
| Navigation | Live map/A*; no persistent map identity or saved-map localization | Atomic archives, scan matching, confidence gates, immutable navigation map and separate live obstacle layer |
| Survey | No completed autonomous first-run frontier survey | Autonomous observation targets, visible unknown gain, explicit partial/completion reasons |
| Route selection | Earlier station definitions/viewer | Local map editor, start heading, ordered intermediate/end poses, dwell and full-footprint path validation |
| Feedback | Deadband/slew experiments, uncertain final calibration | Per-wheel configurable feedforward/PI, reversal/coasting checks, bounded bench pulses, explicit calibration gates |

## Log evidence

`python -m cat_rover_nav replay-telemetry --archive '../Changes by antigravity.zip'` reads the ZIP without executing its contents:

- 382 rows: AUTO_ZERO_HANDSHAKE 11, MONITOR 92, BLOCKED 143, NO_PATH 123, FOLLOW 13.
- 71 rows had nonzero encoder counts; 13 had nonzero wheel commands.
- No recorded mission COMPLETE/waypoint completion and no raw LiDAR beams.

The log establishes those recorded states; it cannot verify mapping accuracy, collision clearance or reconstruct the original scan map. The new simulations and tests therefore exercise the new production runtime with explicit synthetic sensors, not fabricated replay success.

## New architecture

Timestamped workers → signed encoder odometry/history → scan acquisition alignment → bounded robust scan matching/observability against continuous measured keyframe endpoints → mapping + keyframe pose graph **or** immutable saved-map localization → observation/ordered-route targets → oriented footprint A* → trajectory/measurement collision checks → sole guarded command owner → protected Mega controller.

Loop candidates require close nonadjacent geometry, residual/overlap acceptance and bounded graph corrections. A graph rebuild is accepted only when constraints improve and corrections remain bounded. Local scan matching requires an approximate pose; it is not global kidnapped-rover recovery. Calibration and actual Pi timing remain deployment measurements.

Fine-voxel means retain measured coordinates; matching to snapped occupancy cell centres caused a repeatable stationary position bias during validation and was replaced. Stored keyframe endpoints are also used on later map-localized missions. Occupancy cells remain the collision/planning representation.
