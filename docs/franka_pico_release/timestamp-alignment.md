# Timestamped collection and LeRobot conversion

## Business contract

Collection writes the raw Pico episode to PKL without resampling or destructive
alignment. Every retained stream carries its own timestamp and data together.
The PKL remains the source record, including Pico controls, raw events, and
diagnostic fields that are not needed by training.

PKL-to-LeRobot conversion is the only alignment stage. It creates an equally
spaced 30 Hz grid and independently selects the nearest raw sample for each
required stream. A match is valid only when its distance from the grid point is
no greater than half that stream's sampling period. Offline conversion may use
the nearest sample before or after the grid point.

Required streams are:

- global and wrist camera images;
- measured robot state, including TCP, gripper, and Franka joint state;
- the actual post-guard robot action.

If any required stream has no valid match at any grid point, conversion fails
with an explicit diagnostic. It does not fill values with zeros, a previous
sample, or extrapolation, and it does not modify or delete the PKL. Raw samples
outside the aligned output interval remain in the PKL.

LeRobot output contains training fields only: camera images, robot state,
gripper state, actual action, task, success/end markers, and segment metadata.
Pico controls, Pico timestamps, raw events, and diagnostics remain PKL-only.

## Timestamp semantics

All host timestamps use one monotonic clock domain where possible:

- cameras: host frame-receive time, not an exposure timestamp;
- robot state: host time when the measured state is read;
- action: host time when the final post-guard command is sent;
- Pico: vendor timestamp plus host receive time;
- episode/reset events: host monotonic event time.

The converter must use host-monotonic timestamps for cross-stream matching and
must preserve the original timestamp semantics in the PKL metadata.
