# Internal deployment resources

This directory archives the exact Pico deployment resources validated for this
internal project. The PC Service installer and Pico APK are vendor binaries for
internal use only: do not publish, mirror, or redistribute them outside the
authorized project context.

Before installing a binary, verify it with its adjacent `SHA256SUMS` file. The
SDK source is MIT-licensed and is built locally against the matching installed
PC Service SDK; do not substitute an arbitrary prebuilt Python module.

Installation order:

1. Install the Ubuntu PC Service package.
2. Build and install `xrobotoolkit_sdk` from `sdk/` into the Pico SDK virtual
   environment.
3. Install the Pico APK on the headset.
4. Use `bash scripts/run_pico_session.sh --check` before a live collection run.
