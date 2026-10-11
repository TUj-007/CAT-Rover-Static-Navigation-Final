# Validation evidence

Release prepared 2026-10-12. Development host: Windows x64, Python 3.12.14, NumPy 2.5.4, SciPy 1.18.1, pyserial 3.5, rplidar 0.9.2. Python source targets 3.11+ including 3.13; the Raspberry Pi/Python 3.13 execution was not available here.

## Executed checks

| Check | Result |
|---|---|
| `python -m unittest discover -s tests -v` | **46 tests passed**, 33.855 s in final run |
| `python -m compileall -q cat_rover_nav` | Passed |
| Map editor JavaScript, `node --check` | Passed syntax check |
| Native C++ `firmware_parser_test.cpp` with Zig C++11 | Passed CRC/parser/session and reversal/coasting vectors |
| Arduino AVR Mega build, delivered disabled confirmation gates | Passed: 10,682 bytes flash, 793 bytes globals |
| Arduino AVR Mega build, separate test copy with both gates enabled | Passed: 11,930 bytes flash, 797 bytes globals |
| Five named production-runtime CLI simulations | Expected complete/partial results, zero ground-truth collisions |
| Antigravity ZIP telemetry audit | 382 rows; no raw scans available |
| Independent code review | Concrete findings fixed; targeted final review reported no additional critical/important defects |

Build copies and toolchains are outside this package. The enabled build copy was used only for compilation; **no firmware was uploaded and no physical motor commands were sent**. Final build/test logs are included in `docs/evidence/`.

## Production-runtime simulation results

World is a 4.5 × 4.0 m asymmetric rectangular room, with static/live boxes for obstacle cases. Scans contain 360 synthetic rays with deterministic ±2 mm range noise. Four encoder streams include unequal scale biases between -0.3% and +1%. Simulation drives the real autonomy code using synthetic devices and a virtual clock; it never teleports between mission legs. Saved-map missions start at `[0.15,-0.20,0.10]`, not the survey origin.

| Scenario | Survey / mission | Interior observed | Survey travel | End error | Collisions |
|---|---|---:|---:|---:|---:|
| `room` | Complete / 3 ordered points complete | 100% | 0 m (room visible at launch) | 0.0979 m / 0.1176 rad | 0 |
| `obstacle` | Autonomous occlusion survey complete / 3 points complete | 100% | 2.3134 m | 0.0966 m / 0.0008 rad | 0 |
| `live-obstacle` | Empty room map complete / new-obstacle detour complete | 100% | 0 m | 0.0885 m / 0.1196 rad | 0 |
| `dropout` | Partial; stale LiDAR stops and recovery times out | 88.71% | 0 m translation; initial turn commanded | — | 0 |
| `ambiguous` | Partial; circular geometry rejected, no motion commands | 0% | 0 m | — | 0 |

All completed navigation cases preserved the exact static map hash and finished with zero commands. Obstacle survey used 71.95 simulated seconds; its subsequent route used 45.20 s. Room route used 34.95 s; live obstacle route used 35.90 s. These are virtual course times, not Raspberry Pi benchmark timings.

“Interior observed” counts geometric room-interior grid cell centres with sensor evidence, excluding solid box interiors. It does not certify perfect occupancy classification or inaccessible space. The 6 × 6 m map includes exterior/behind-wall cells that correctly remain unknown; its total-map unknown area is therefore nonzero even on a completed room survey. Minimum frontier/gain thresholds and conservative surface pinhole closure apply at the chosen resolution. The raw reports, maps, routes and logs are in `examples/<scenario>/`.

## Regression coverage

- Strict finite configuration types, wheel order/arrays, bounds, speed limits and CRC framing.
- Signed/overflowing encoder counts, elapsed-time/reset checks, odometry interpolation, stale/invalid scans, conversion/masking and invalidation.
- Measured-ray occupancy, unknown preservation, no invented safety bubble, removable noise, clipping, atomic archives and evidence/metadata corruption rejection.
- Continuous-point scan alignment, translation/yaw correction, bad overlap, corridor/circular degeneracy, immutable-map localization and stationary-map bias regression.
- Pose-graph constraint improvement/rebuild and rejection of a damaging false constraint.
- Full rectangular footprint detours, tight straight corridor versus blocked pivot, sampled motion primitives, live obstacles, full measured braking, mixed residual translation/turning and scan acquisition pose.
- Observation targets in known free space, all candidate regions examined, failure/partial distinction, large ring frontier anchors and separation of observed uncertainty from unexplored space.
- Mandatory zero handshake, firmware identity, missing scans, bounded alignment/planning failure, stale post-computation commands, incompatible routes and pre-device validation.
- Native firmware CRC, malformed/overflowing fields, epoch/sequence/echo age, zero handshake, speed bounds, steady wrong-direction fault and bounded reversal coast.

## Remaining deployment checks

1. **Physical commissioning:** actual corrected wiring, x1 counts, polarity, loaded wheel/track geometry, individual driver gains/deadband, D30 stop, motor coasting/braking, traction/slip, range/angle calibration and all four wheel responses. Shipped confirmation flags remain false until measured.
2. **Pi timing:** actual USB scan assembly/backlog, computation time, command watchdog recovery and sensor age on the target Pi. If data cannot remain fresh, the rover stops; reduce course/speed or improve computation rather than relaxing checks blindly.
3. **Browser visual/interaction review:** browser security policy denied opening the local editor. No alternate browser path was attempted. The HTML/JavaScript syntax and server-side route/footprint logic were checked; visual layout, dragging, click/download interaction still need a normal browser check by the user.

The simulations do not model driver electronics, suspension/traction, slipping turns, glass, drop-offs, obstacle heights or electrical braking. A LiDAR-only system cannot infer hazards outside the scan plane. No claim of perfect operation or physical validation is made.
