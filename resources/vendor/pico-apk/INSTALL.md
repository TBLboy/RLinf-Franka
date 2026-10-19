# XRoboToolkit Pico application 1.1.1

**Internal-use vendor binary. Do not redistribute outside this project.**

Verify the archived APK before installing it on the authorized Pico headset:

```bash
cd Release/resources/vendor/pico-apk
sha256sum -c SHA256SUMS
```

Install `XRoboToolkit-PICO-1.1.1.apk` using the approved headset deployment
procedure. In the app, configure the PC Service address with the host's current
hotspot-network IPv4 address (for example `192.168.50.x`), enable Head,
Controller, and Send tracking, disable `Switch w/ A Button`, and set Pico Motion
Tracker mode to None.

The exact host address is printed by `scripts/run_pico_session.sh`; do not copy
an address from a different host or network.
