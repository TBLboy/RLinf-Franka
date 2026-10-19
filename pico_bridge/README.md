# Pico bridge

This directory owns the project-specific bridge that reads `xrobotoolkit_sdk` and
publishes headset and controller state over the configured ZeroMQ IPC socket.

The Release launcher uses the verified external SDK environment at
`../pico_software/XRoboToolkit-Teleop-Sample-Python/.venv` by default. Override
it with `RLINF_PICO_SDK_VENV` only for a vendor-compatible replacement. The
vendor PC Service binary and SDK binding are external dependencies and are not
redistributed here.
