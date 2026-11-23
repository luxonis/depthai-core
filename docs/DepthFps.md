# Depth FPS selection

Depth uses one set of FPS limits per device class, independent of the OS version.
The regular-device neural limits follow the [depth guide](https://docs.luxonis.com/overview/toplevel-features/depth/).
Lite devices have a separate FPS column. These values currently copy the regular-device
limits as placeholders and have not been measured on Lite hardware.

When wiring an RVC4 Depth node, OS versions older than 1.47.0 (or an unknown version)
produce a warning. Version 1.47 is the earliest verified version for the faster neural
models; this is a recommendation, not a compatibility gate. Selection and camera FPS
remain unchanged by the warning. Actual throughput on older software may be lower.
Lite devices also report that their limits are provisional.

AUTO and explicit NEURAL selection, default prediction, and fallback ranking all use
the same device-class limits. Existing resolution/model filters and the 10% selection
tolerance remain in effect. An impossible pinned AUTO request still raises an error.
