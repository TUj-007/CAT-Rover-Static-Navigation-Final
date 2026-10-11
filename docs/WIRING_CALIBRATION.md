# Wiring and calibration

## Physical motor layout

The wheel order in firmware, telemetry and configuration is **FL, RL, FR, RR**. Left-side speed commands drive FL+RL; right-side commands drive FR+RR. Each wheel has its own feedback controller. Corrected assignments require two front MD10 PWM/DIR channels and two rear BTS7960 modules.

| Wheel | Physical driver | PWM / direction pins | Enable pins | Encoder A / B |
|---|---|---|---|---|
| Front left (FL) | MD10 | PWM D3, DIR D4 | — | D18 / D22 |
| Rear left (RL) | BTS7960 | RPWM D5, LPWM D6 | R_EN D26, L_EN D27 | D19 / D23 |
| Front right (FR) | MD10 | PWM D9, DIR D12 | — | D20 / D24 |
| Rear right (RR) | BTS7960 | RPWM D7, LPWM D8 | R_EN D28, L_EN D29 | D21 / D25 |

Use appropriate motor power wiring, common logic ground, and the voltage/logic requirements of your exact shields. D30 uses `INPUT_PULLUP`: a normally closed emergency-stop contact to ground permits operation; opening it stops operation. Provide a motor-power disconnect as well. No RC receiver pins are used. BTS enables and PWM are low at boot and commanded stop.

MD10 shield routing must match these pins; some shield versions need jumpers or re-routing. Confirm the actual model before wiring. Driver direction constants are configurable; pin identities alone do not prove physical forward direction.

## Measure before enabling ground motion

1. **Dimensions and sensor mounting.** Measure loaded wheel diameter, effective track width and the full body/attachments. Example values are 0.10 m, 0.4425 m and 0.565 × 0.40 m. Set both `HardwareConfig.h` and `config.json` wheel dimensions/counts consistently. Set `lidar_x_m`, `lidar_y_m`, `lidar_yaw_deg` and `lidar_angle_sign` so returns ahead/left agree with +x/+y. Standard A1 output is converted clockwise to CCW with default sign -1; verify with actual points.
2. **Encoder scale.** Firmware counts only A-channel RISING edges, with B indicating direction (**x1**). With drive disabled, rotate each wheel forward through several measured complete output revolutions. `diagnose` prints counts; use signed uint32 differences, divide by revolutions and enter FL/RL/FR/RR `counts_per_rev` and `COUNTS_PER_REV`. Do not substitute a manufacturer's x4/shaft figure. The supplied 600 is unverified.
3. **Encoder sign.** Forward manual rotation must increase the corresponding count. Change that wheel's `ENC_SIGN` to -1 if needed. Verify the physical order; wiring changes from the old rear/front assignments must not swap telemetry labels.
4. **Raised-wheel commissioning.** Keep all driven wheels securely clear of the ground. After wiring/count measurements, record provisional per-wheel motor tuning, then enable both confirmation gates for this bench commissioning and upload. Use the bounded pulse tool below to test each side forward/reverse. Adjust `MOTOR_SIGN` until commanded positive means forward. The encoder sign must still match. A persistent wrong-sign velocity, implausible speed or stall latches fault 16; inspect and reset the Mega after fixing the cause.
5. **Motor tuning.** `MIN_PWM`, `KF`, `KP` and `KI` are individual FL/RL/FR/RR values. The shipped 130 PWM is a lead from Antigravity's bench notes, not a measured setting for this corrected driver layout. Adjust feedforward/deadband so sustained measured speeds follow 0.04–0.08 m/s in both directions without oscillation/saturation. Limit PWM and speed during commissioning. Do not use autonomous navigation to discover motor polarity.
6. **Loaded motion and stopping.** On a controlled clear course, measure forward distance, left/right turning and stopping distance with the actual load. Recheck effective track width on the skid-steer chassis. Reduce cruise speed until the encoder/scan timing and slip are acceptable. Set `braking_mps2` to a conservative measured deceleration (smaller value means more clearance); reaction defaults 0.90 s. Confirm all four wheels move and that commanded zero, D30 stop, USB loss and interrupted Pi execution stop within the measured envelope.
7. **Finish calibration.** Save measurements/tuning. Keep confirmation flags false if any measurement fails. Enable ground operation only after commissioning succeeds. Any geometry/sensor-mount change requires a new survey and route.

The controller's reversal pause is 120 ms plus bounded coasting recovery. Stops bypass PWM ramping. PWM zero is not a guarantee of active electrical braking, especially with BTS enables low; measure the resulting coasting.

## Bounded bench pulses (wheels raised only)

This module opens the Mega only, validates firmware identity and fresh telemetry, sends a zero handshake, then a short closed-loop target using the normal protected C protocol. It is separate from autonomy and does not require LiDAR. Speeds are limited to 0.10 m/s and duration to 5 s.

```bash
python -m cat_rover_nav.bench --config config.json --left 0.04 --right 0 \
  --seconds 2 --wheels-raised --allow-motion
python -m cat_rover_nav.bench --config config.json --left 0 --right 0.04 \
  --seconds 2 --wheels-raised --allow-motion
```

Repeat with -0.04 after forward measurements. It prints measured FL/RL/FR/RR m/s. To inspect raw counts without motion, use the regular `diagnose` command with gates false. Neither tool sends the old unchecked `M`/`I` commands.

## Upload

Use Arduino IDE with board **Arduino Mega or Mega 2560**, or Arduino CLI:

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli compile --fqbn arduino:avr:mega firmware/rover_controller
arduino-cli upload --fqbn arduino:avr:mega --port /dev/ttyACM0 firmware/rover_controller
```

Replace the upload port with the actual Mega device. No upload is performed automatically. `diagnose` must report `{"protocol":2,"encoder_edges":1,"motor_layout":1}`. Fault 4 means confirmation gates disabled, fault 1 emergency-stop contact open, fault 8 command timeout requiring a new zero handshake, fault 16 encoder/motion/stall failure. RC recovery is not used.

## USB wire contract

115200 baud, ASCII, newline-terminated, maximum bounded frame. Suffix `*HHHH` is CRC-16/CCITT-FALSE (initial 0xFFFF) over the body. `Q` requests `V,2,1,1`; telemetry retains the legacy field count with neutral RC placeholders for compatibility.

`C,epoch,sequence,echoed_telemetry_ms,left_mm_per_s,right_mm_per_s` requires matching epoch, increasing modular uint32 sequence, telemetry echo no older than 250 ms, magnitude ≤300 mm/s, and first command zero. No CRC/age/sequence bypass exists. Accepted command timeout is 300 ms; AVR execution watchdog is 250 ms. The host stops on reset, uncertainty or stale sensing; after a fault, investigate its reason before another run.
