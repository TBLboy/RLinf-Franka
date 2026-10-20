# Collection operation

Only an operator may start a live robot session. Before a run: clear the work
area, make the emergency stop reachable, confirm Franka Desk is ready, connect
both UVC cameras through USB 3.0, and verify the configured Robotiq serial path.

```bash
bash scripts/run_pico_session.sh --check
bash scripts/run_pico_session.sh
```

The collection launcher owns the Ray lifecycle and the Pico publisher lifecycle.

## 30 Hz control-rate experiment

The active Release configuration requests a 30 Hz RLinf environment cadence:
read robot state and cameras, process the newest Pico command, send the next
TCP target, and write one collection sample. It does not change Franky's
separate real-time impedance loop. Collection metadata also declares 30 Hz.

This is an operator-approved experiment, not a hardware-validated setting.
Before collecting usable demonstrations, test unloaded and within a small
workspace; stop immediately for stale motion, vibration, unexpected force,
Desk warnings, or errors. The 10 Hz baseline is tagged locally as
`backup/vr-controls-20261020`.

Both UVC cameras are explicitly requested at 30 FPS. A session rejects a camera
that cannot report the configured resolution and frame rate; do not treat 30 Hz
images as valid until an operator confirms two fresh, stable camera streams on
direct USB 3.0 connections.

## Pico operator controls

The supported single-arm collection controls are:

| Input | Effect |
| --- | --- |
| Right menu, release after holding at least 1 second | Enable VR control. Release the right grip once before driving again. |
| Right menu, release before 1 second | Pause and lock VR control immediately. |
| Right side grip | Hold to drive Cartesian arm motion. |
| Right trigger while grip is held | Set the continuous Robotiq opening target. |
| Left thumbstick click | Start a recording when VR is enabled and no recording is active. |
| Y | End and save the active recording successfully. |
| X | End and discard the active recording, then reset. |
| A + X within 150 ms | Discard the active recording and run the configured reset pose. |
| Right thumbstick click | Recalibrate the Pico frame while arm control is inactive. |

A and X are edge-debounced for 40 ms before their 150 ms combination window.
During reset, VR motion and recording requests are ignored. Completion leaves VR
enabled but requires a fresh right-grip press. A and B otherwise remain reserved
for future DAgger control and have no single-button action.

Every post-startup reset, including reset after Y success and A+X discard/reset,
closes the Robotiq before arm motion and opens it after one bounded trajectory to
`reset_ee_pose`. It does not add position-convergence retries before opening.
The opening is the reset-complete signal. Reset intentionally releases any object
held by the gripper; do not use it when releasing the object is unsafe.
Do not connect another control process to the robot during a session. Keyboard
episode controls are not part of the supported collection workflow. Any
unexpected motion, vibration, force, Desk warning, or robot error requires
releasing grip and stopping immediately.

After a completed run:

```bash
python scripts/check_collection_run.py --run-dir <run-directory>
python scripts/post_process_to_lerobot.py --run-dir <run-directory> --repo-id local/franka-pico
```

The converter is the legacy pickle-to-LeRobot path. It uses
`infos[*].intervene_action`, not the top-level pickle `actions` field.

## Validated Cartesian damping route

Release uses `franky-control==2.0.0` with the public Franky
`libfranka-0.21.2` wheel index. The supported Pico configuration uses Cartesian
translational stiffness `1200`, translational damping `69.282032`, rotational
stiffness `40`, rotational damping `4`, and rotation clips `0.15`. The tracker
passes explicit six-axis damping. The translational value equals
`2 × sqrt(1200)`, preserving the previous critical-damping behavior while making
it independently configurable. Continuous gripper control uses the right trigger only while
side grip is held; A/B are not gripper controls.

This is the validated Cartesian route, not joint-IK. Do not raise damping,
stiffness, target-step guards, collision thresholds, or Desk limits as part of
normal operation. Any Release update still requires operator-led unloaded,
small-range hardware revalidation before being labelled verified on a new host.
