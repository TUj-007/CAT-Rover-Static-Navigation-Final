# SDD ledger — plan: docs/superpowers/plans/2026-10-11-autonomous-static-navigation.md

Execution authorized by the user's request to finish and deliver the project. Native execution selected as previously recommended.

Ruling: Git-dependent worktree/brief/commit scripts are inapplicable at this non-Git workspace root; work in the requested separate folder, keep this ledger and test logs instead of deleting them — preserves the user's requested location and audit trail — cost if wrong: no Git rollback history for the new files.

Pre-flight: Tasks 1→2→3→4 share Pose/Scan/frame conventions; use one package with relative imports.
Pre-flight: Tasks 3/4→5/6 share occupancy state and map frame; unknown stays blocked and live obstacles remain separate.
Pre-flight: Tasks 2/4/5/6→7 share freshness and accepted localization; the runtime is the sole motion-command owner.
Pre-flight: Tasks 3/5→8 share map identity, footprint and waypoint frame; validate before hardware initialization.
Pre-flight: Task 7→9 shares real runtime APIs; simulator supplies devices/clock, not alternate autonomy logic.

Task 1: complete — absent-module RED observed, 3 contract tests GREEN.
Task 2: complete — absent-device RED observed; full 8-test suite GREEN.
Task 3: complete — mapping/storage RED observed; full 12-test suite GREEN, including atomic persistence and unknown-space preservation.
Task 4: complete — scan matching RED observed; translation/yaw correction, circle/corridor rejection, immutable localization and rejected-scan tests GREEN.
Task 5: complete — planning RED observed; exact-footprint check fixed a cell-centre dilation false blockage. Full 21-test suite GREEN.
Task 6: complete — frontier RED observed; 3 exploration tests GREEN, including partial versus complete distinction.
Task 7: complete — guarded runtime and corrected firmware delivered; zero/freshness/deadline tests GREEN; native parser/reversal vectors pass; enabled and disabled Mega builds pass. No upload or physical motion performed.
Task 8: complete — local route editor, identity/finite/footprint/path validation, CLI pre-device checks and JavaScript syntax verified. Browser visual gate unavailable due to explicit browser-policy denial.
Task 9: complete — production-runtime room, occlusion exploration, live-obstacle detour, dropout and ambiguous-geometry simulations delivered; 46 Python tests pass; compileall, native harness and both AVR builds pass. README, calibration guide, source audit and validation evidence included.

Ruling: Browser visual validation is unavailable because opening localhost was denied by browser security policy; do not circumvent the denial — retain server-side input/path tests and JS syntax checks, document pending manual interaction/visual QA — cost if wrong: editor rendering/drag/download defects could remain until browser verification.

Implementation notes: editor.py/map_editor.html supply the planned web/static interface; test_simulation.py supplies the end-to-end suite. KD-tree correlative scoring replaces raster distance-transform scoring; measured fine-voxel keyframe means prevent cell-centre scan bias. These preserve the selected architecture and frame contracts.

Review findings resolved: measured braking under deceleration; acquisition-pose scan transforms; all frontier candidates and large-ring anchors; bounded alignment recovery; steady reverse-sign detection with coasting allowance; mixed measured/commanded stopping trajectories; post-computation age check; stationary map bias. Regression failures were reproduced before fixes.

Final physical checks pending by nature of this host: actual wiring/encoder/polarity, motor tuning, loaded stopping, LiDAR mounting and Pi runtime timing. Hardware/calibration flags remain false in delivered real-hardware defaults. Synthetic examples are labeled simulation only.
