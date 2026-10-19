# XRoboToolkit PC Service 1.0.0

**Internal-use vendor binary. Do not redistribute outside this project.**

Target: Ubuntu 22.04, amd64.

```bash
cd Release/resources/vendor/pc-service
sha256sum -c SHA256SUMS
sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
```

The package installs to `/opt/apps/roboticsservice`. The Release launcher starts
`/opt/apps/roboticsservice/runService.sh` and expects the Pico headset to reach
the PC Service on TCP port `63901` over the configured hotspot network.

Package identity: `roboticsservice` version `1.0.0.0`, SHA-256 recorded in
`SHA256SUMS`.
