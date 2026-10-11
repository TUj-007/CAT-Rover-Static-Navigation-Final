# CAT Rover — camera-free autonomous static navigation

This is the separate Raspberry Pi implementation requested for the final Antigravity hardware. It uses the RPLidar A1M8, four wheel encoders, a Raspberry Pi and an Arduino Mega. **Front motors use MD10 shields; rear motors use BTS7960 drivers.** No camera or RC receiver is used.

## What it does

1. On the first survey, it establishes `[0,0,0]` at the rover's launch pose, builds an occupancy map and autonomously visits observation frontiers in known free space. It stops when no useful reachable observation target remains.
2. It saves a versioned `.npz` map, a `.pgm` image and a `.json` description. Interrupted or failed surveys save a **partial** map and a reason.
3. You select a start pose, intermediate points and an end point in the map editor. Point headings and dwell times are explicit. The editor checks paths before downloading a route.
4. On subsequent runs, place the rover at that start pose and heading. LiDAR alignment must succeed before motion. The rover visits the points in order, then stops. The static map stays unchanged; new live obstacles are checked separately.

Use a bounded, level indoor course whose walls/obstacles intersect the LiDAR scan plane. First map with the temporary obstacles removed, as requested; keep perimeter walls/features visible. The survey maps observable reachable space at the configured resolution. It cannot measure through walls, underneath the LiDAR plane, glass reliably, or beyond sensor range. Unknown cells remain blocked. A featureless/symmetric scan that cannot support localization causes a stop.

## 1. Install on Raspberry Pi

Use 64-bit Raspberry Pi OS, Python 3.11 or later (3.13 supported by the source). Copy this entire folder to the Pi:

```bash
cd ~/raspberry_pi_static_nav
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
cp config.example.json config.json
mkdir -p maps
```

If USB access is denied, add your account to `dialout` and log out/in:

```bash
sudo usermod -aG dialout "$USER"
ls -l /dev/serial/by-id/
```

Edit `mega_port` and `lidar_port` in `config.json` to match those devices. Use stable by-id paths. No camera libraries, ROS installation or desktop session are required for driving. The browser editor can run on a laptop with a copy of the map.

## 2. Wire, calibrate and upload the companion firmware

Follow [docs/WIRING_CALIBRATION.md](docs/WIRING_CALIBRATION.md). The old Antigravity firmware is incompatible with this package. Upload `firmware/rover_controller/rover_controller.ino` to **Arduino Mega 2560**. Its identity is protocol 2, x1 encoders, front MD10/rear BTS layout 1.

Delivered firmware and configuration confirmation flags are **false** because actual polarity, encoder counts, motor gains and braking have not been measured here. Set them true only during the documented commissioning process. The reported `600 counts/revolution`, PWM deadband and braking rate are calibration examples.

```bash
python -m cat_rover_nav diagnose --config config.json --seconds 10
```

This sends zero commands and prints firmware identity, raw encoder counts and scan usability. After calibration, confirm `hardware_confirmed` and `calibration_confirmed` in `config.json` and the corresponding firmware constants. The firmware also requires the D30 emergency stop contact.

## 3. First fully autonomous mapping run

Place the rover near the course centre with clear surroundings. Its initial forward direction becomes +x; left becomes +y. Set `map_size_m` and `survey_bounds` to contain the whole course. Default map is 6 × 6 m, 5 cm cells, with motion constrained to ±2.9 m. The rover's complete footprint must fit inside these bounds.

```bash
python -m cat_rover_nav explore --config config.json --output maps/environment.npz \
  --log maps/survey.jsonl --allow-motion
```

The rover explores autonomously. If all useful space is visible from the launch pose, it can finish without driving. Check the final printed status and reason. A `partial` status means unresolved frontiers, localization/sensor failure, a timeout or an interruption. Increase map bounds and run a new survey if scans extend outside the map. Existing output maps are protected from accidental overwrite by the CLI; choose a new filename for a new survey. Ctrl+C/SIGTERM requests zero, saves a partial survey and closes devices. Power loss can recover the latest atomic checkpoint.

## 4. Define start, intermediate and end points

```bash
python -m cat_rover_nav view-map --map maps/environment.npz
```

Open the printed local URL. Click the start; drag its arrow to set the real rover heading. Click the intermediate points in order and finally the end point. You can set heading and dwell before placing each point. Click **Check paths**, then **Download route** and save it as `maps/route.json`.

The editor binds to `127.0.0.1:8765` and cannot control motors. For a headless Pi, either copy `environment.npz` to a laptop and run the viewer there, or use an SSH tunnel:

```bash
# Pi:
python -m cat_rover_nav view-map --map maps/environment.npz --no-browser
# Laptop, in another terminal:
ssh -L 8765:127.0.0.1:8765 YOUR_PI_USER@YOUR_PI_ADDRESS
```

Then open `http://127.0.0.1:8765` on the laptop. Copy the downloaded route JSON back to the Pi. Route headings are radians in the file, degrees in the editor. The map origin is the first survey launch position, **not** the lower-left corner of the image. Routes contain the exact map identity and are rejected if the map changes.

## 5. Navigate the route autonomously

Physically place the rover at the route's selected start and heading, within the local scan search basin (defaults ±15 cm / ±0.20 rad). A known, approximate start is required; this does not perform global localization from an arbitrary unknown pose.

```bash
python -m cat_rover_nav navigate --config config.json --map maps/environment.npz \
  --route maps/route.json --log maps/navigation.jsonl --allow-motion
```

All points and static paths are validated before USB devices are opened. The rover then verifies its initial scan, follows the ordered route without resetting pose between legs, obeys dwell times and stops at the end. A live obstacle can cause replanning or a stopped fault if no confident safe path remains. Output exit code is 0 for a completed mission, 1 for a partial/faulted run.

## Test without hardware

```bash
python -m unittest discover -s tests -v
python -m cat_rover_nav simulate --scenario room --output demo/room
python -m cat_rover_nav simulate --scenario obstacle --output demo/obstacle
python -m cat_rover_nav simulate --scenario live-obstacle --output demo/live-obstacle
python -m cat_rover_nav simulate --scenario dropout --output demo/dropout
python -m cat_rover_nav simulate --scenario ambiguous --output demo/ambiguous
python -m cat_rover_nav view-map --map examples/room/map.npz
```

Simulation runs the same mapping/localization/planning/runtime as hardware; only sensors, encoders and clock are synthetic. Example maps/routes are **simulation artifacts**, not maps of your physical course. Do not use their confirmed simulation config to drive the real rover.

## Files and engineering details

- `cat_rover_nav/`: configuration, hardware workers, strict USB framing, odometry, occupancy mapping, scan matching, SLAM pose graph, footprint planning, exploration, guarded runtime, route editor, simulation and log audit.
- `firmware/rover_controller/`: corrected Mega controller with encoder feedback, CRC/epoch/sequence/time checks, immediate commanded stop, watchdogs and reversal/stall handling.
- `docs/WIRING_CALIBRATION.md`: pin map, measurements and commissioning.
- `docs/ANTIGRAVITY_AUDIT.md`: source review and migration decisions.
- `docs/VALIDATION.md`: executed checks, simulation evidence and remaining physical/browser verification.
- `examples/`: generated simulation maps, route files, logs and reports.

Safety checks use the oriented 0.565 × 0.40 m rectangular body plus margin, including sampled pivot sweeps and measured reaction/braking travel. Survey planning uses known static free space; navigation adds a live obstacle layer. Freshness is measured from scan acquisition, not the time a cached scan is fetched. Computation cannot refresh motion independently; the Mega stops after a 300 ms command timeout. Abrupt PWM zero is a stop request; actual coasting distance must be measured and configured.

This software was verified on the development computer and in simulation. Physical Pi timing, real range data, traction, braking and motor tuning still require the commissioning procedure before autonomous floor runs. See validation evidence for the precise limits.
