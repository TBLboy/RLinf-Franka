# Installation

Supported host: Ubuntu 22.04 with a PREEMPT_RT kernel, Python 3.11, and the
Franka Research 3 network already configured. This repository does not install
or configure Franka Desk, robot firmware, the PC Service, or USB permissions.

```bash
cd Release
uv sync --group test
```

Install the compatible Franky Control/libfranka wheel and the vendor
`xrobotoolkit_sdk` into their documented environments before use. On this
validated host, the Release launcher reuses the known working vendor SDK
environment at:

```text
../pico_software/XRoboToolkit-Teleop-Sample-Python/.venv
```

Set `RLINF_PICO_SDK_VENV=/path/to/compatible/.venv` to use a replacement. Do
not replace the working SDK environment with an empty generic bridge venv. The
root `pyproject.toml` and `uv.lock` are inherited from the approved RLinf
baseline; Release-specific dependency additions are documented in
`environment-pyproject.toml` and `dependencies.md`.

Verify the offline layout before connecting hardware:

```bash
python -m pytest
bash scripts/run_pico_session.sh --dry-run
```

See [dependencies.md](dependencies.md) for requirements that are not vendored.
