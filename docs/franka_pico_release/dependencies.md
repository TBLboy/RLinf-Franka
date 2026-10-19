# External dependency inventory

## Python environment

Create the collection environment from the repository root with `uv sync`. The
Pico bridge reuses the existing verified XRoboToolkit SDK environment by default;
configure `RLINF_PICO_SDK_VENV` only with a vendor-compatible replacement.
The root `pyproject.toml` and `uv.lock` are tracked from the approved RLinf
baseline, so they can describe upstream capabilities beyond this Release's
support scope. Release-specific package requirements are captured in
`environment-pyproject.toml`; they must be reconciled with an upstream update
before marking that update supported.

## Not vendored

| Requirement | Why it is external | Required verification |
| --- | --- | --- |
| Franky Control 2.0.0 / libfranka 0.21.2 | Hardware binding and native wheel | `python -c 'import franky'` within the Release environment |
| Franka Desk and FCI access | Robot firmware and safety system | Operator confirms FCI and Desk status before collection |
| XRoboToolkit PC Service and `xrobotoolkit_sdk` | Vendor runtime and hardware SDK | Pico bridge imports `xrobotoolkit_sdk`; `scripts/pico_probe.py` observes live non-zero data |
| Robotiq USB-RS485 device | Physical device and udev path | Stable `/dev/serial/by-id/` path is present |
| ZED UVC cameras | Physical UVC devices | Preflight finds both configured `/dev/v4l/by-id/` nodes over USB 3.0 |
| Linux input permission | Host security configuration | `RLINF_KEYBOARD_DEVICE` is readable and identifies a real keyboard |

Do not copy proprietary vendor binaries into this repository until their
redistribution terms are documented and approved.
