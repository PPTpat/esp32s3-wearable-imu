# Signal processing (components/dsp)

Pure C++ with no ESP-IDF dependencies, so everything here runs and is tested on a PC.
All algorithms are fed at the sensor rate (104 Hz by default) and use no heap allocation.

| Module | Input | Output |
|---|---|---|
| `Madgwick` | gyroscope (rad/s), accelerometer, dt | orientation as quaternion and Euler angles |
| `StepCounter` | acceleration magnitude (g) | step count |
| `FallDetector` | acceleration magnitude (g) | fall events |

## Orientation: Madgwick filter

Gradient-descent filter for a 6-axis IMU. The gyroscope is integrated, and the estimated gravity
direction is pulled towards the measured accelerometer vector with gain `beta` (default 0.1).

- Frame convention: a sensor lying flat reads (0, 0, +1 g) and gives the identity orientation.
- Roll and pitch are absolute. **Yaw has no reference** (there is no magnetometer), so it drifts with the gyroscope bias.
- A zero accelerometer vector skips the correction and falls back to gyroscope integration.

## Step counter

1. A running mean with a 1 s time constant removes gravity (high-pass).
2. A one-pole low-pass at about 4 Hz removes jitter.
3. A local maximum counts as a step if it is above `max(minThresholdG, adaptiveRatio * recent peak envelope)`
   and at least `minIntervalMs` after the previous step.

Defaults: `minThresholdG` 0.12 g, `adaptiveRatio` 0.5, `minIntervalMs` 300 ms.
The minimum interval rejects the second bump of a single step.

Known limitations:

- A sharp impact looks like a peak and **is counted as a step** (a replay with a synthetic fall showed exactly this).
- Shaking the sensor at a few hertz is counted as steps.
- Thresholds depend on where the sensor is worn and have **not been tuned on real recordings yet**.

## Fall detector

A fall is a sequence: free fall, impact, stillness.

| Stage | Condition (defaults) |
|---|---|
| free fall | magnitude below 0.4 g for at least 60 ms |
| impact | magnitude above 2.5 g within 500 ms after free fall ends |
| settling | 300 ms of ringing after the impact is ignored |
| stillness | magnitude within 1 g +/- 0.25 g for 1 s, within 3.5 s of the end of settling |

An impact without free fall, a free fall without impact, and movement after the impact do not trigger.
One event is reported per fall.

Known limitations:

- At 104 Hz one sample is about 10 ms, so very short impact peaks can be missed.
  The sensor can run faster, and it also has a hardware free-fall detector that is not used yet.
- The thresholds are starting values and **have not been validated on real falls**.
- This is a demonstration algorithm, not a safety or medical device.

## Tests

Host unit tests (Unity) use synthetic signals with known answers:

- `test_madgwick`: static roll and pitch convergence, gyroscope-only yaw, tracking when gyroscope and
  accelerometer agree, correction of a gyroscope bias, normalisation, zero-input and bad-dt handling.
- `test_step_counter`: sinusoidal walking, gentle walking, movement below the threshold,
  double-bump steps, still signal, noise, reset.
- `test_fall_detector`: rest, a full fall, timing of the event, impact only, short free fall,
  movement after the impact, late impact, two falls, reset.

These tests show the algorithms do what they are designed to do. They say nothing about accuracy on real movement.

## Replaying recordings

`replay_csv` runs a CSV from `tools/ble_logger.py` through all three algorithms:

```bash
cmake --build test/host/build --target replay_csv
./test/host/build/replay_csv data/raw/session.csv --events
```

It prints the sample rate, the step count (with times if `--events` is given), fall events and the final orientation.

## Next

Record real sessions (still, a known number of steps, shaking, a soft drop), compare the counts, tune the
thresholds, and only then run the algorithms on the device.