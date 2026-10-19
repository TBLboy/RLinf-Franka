# xrobotoolkit_sdk source and build contract

Archived source: `xrobotoolkit-pybind/` at commit
`c64ccf6acd577a333e03b66fafe8efeeceb511b1` (`xrobotoolkit_sdk` version 1.0.2).

License: MIT; retained at `xrobotoolkit-pybind/LICENSE`.

The resulting Python extension links to `libPXREARobotSDK.so`, supplied by the
installed matching XRoboToolkit PC Service package. Build the binding against
the installed service; do not copy a `.so` from another host or PC Service
version.

The supported host is x86_64 Ubuntu with Python 3.11. Build into a dedicated
Pico SDK environment and point Release at it with `RLINF_PICO_SDK_VENV` when it
is not the default path.
