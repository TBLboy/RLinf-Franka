# RLinf Franka/Pico Release

This repository is an **RLinf-based Franka/Pico adaptation release**. RLinf is
the upstream framework; this repository maintains the integration, collection
workflow, configuration, data tools, tests, and operator documentation for one
Franka Research 3 arm controlled through Pico.

The framework may contain training, simulators, agentic workflows, and other
robot integrations. Their presence is not a Release support commitment.

## Supported workflow

1. Install the documented environment and external vendor components.
2. Run `bash scripts/run_pico_session.sh --check` before a live session.
3. Start collection through `bash scripts/run_pico_session.sh`.
4. Check a completed run with `python scripts/check_collection_run.py`.
5. Use `python scripts/post_process_to_lerobot.py` for the legacy pickle-to-LeRobot export.

See `docs/franka_pico_release/` for setup, safety, source provenance, and
upstream synchronization rules.
