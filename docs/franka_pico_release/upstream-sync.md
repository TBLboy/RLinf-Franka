# RLinf upstream synchronization

## Remote roles

- `upstream` is `https://github.com/RLinf/RLinf.git` and is used only to obtain
  framework updates.
- `origin` is reserved for this project's Release distribution repository. It
  is configured only after a project distribution URL is assigned.

## Supported-version rule

An RLinf revision is not a supported Franka/Pico Release version until the
adaptation layer has passed the checks below. Record the approved branch, tag,
or commit and the resulting verification in Release history or the project
state before distribution.

## Procedure

```bash
git fetch upstream
git merge upstream/<approved-branch-or-tag>
# resolve Release adaptation conflicts
python -m pytest tests/release
bash scripts/run_pico_session.sh --dry-run
```

Then verify that the single-arm configuration, rig extension registration,
Pico bridge, launcher, quality check, and LeRobot converter still match the
upstream interfaces. If a real-hardware check is required, the operator performs
an unloaded, small-range session with a reachable emergency stop. Any failed
compatibility check or unverified hardware behavior prevents release marking.

Never raise Desk limits, collision thresholds, or other safety limits as part of
an upstream synchronization.
