# Collection operation

Only an operator may start a live robot session. Before a run: clear the work
area, make the emergency stop reachable, confirm Franka Desk is ready, connect
both UVC cameras through USB 3.0, and verify the configured Robotiq serial path.

```bash
bash scripts/run_pico_session.sh --check
bash scripts/run_pico_session.sh
```

The collection launcher owns the Ray lifecycle and the Pico publisher lifecycle.
Do not connect another control process to the robot during a session. During
recording, `a` starts or aborts, `b` creates a segment, and `c` completes a
successful episode; hold the Pico grip while operating. Any unexpected motion,
vibration, force, Desk warning, or robot error requires releasing grip and
stopping immediately.

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
