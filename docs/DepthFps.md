# Depth FPS selection

Depth uses one set of FPS limits per device class, independent of the OS version.
The neural limits come from a per-model throughput sweep on OS 1.47 with DepthAI 3.12
(live 1280x800 stereo input requested at 60 FPS, depth measured on device). Models that
reached the 60 FPS input rate are listed at 60; their true maximum may be higher.

| Model | FPS limit | Measured FPS (2 runs) |
|---|---|---|
| 1248x780 | 20 | 21.7 / 21.8 |
| 1056x660 | 30 | 31.4 / 31.5 |
| 960x600 | 38 | 38.8 / 38.9 |
| 864x540 | 45 | 46.7 / 46.6 |
| 768x480 and smaller | 60 | 60 (input-capped) |

Lite devices have a separate FPS column. These values currently copy the regular-device
limits as placeholders and have not been measured on Lite hardware. Lite devices
report that their limits are provisional.

When a Depth node resolves to NeuralDepth on OS older than 1.47.0 (or an unknown version),
it logs a warning. Version 1.47 is the earliest verified version for the faster neural
models; this is a recommendation, not a compatibility gate. Selection and camera FPS
stay the same. Actual throughput on older software may be lower.

AUTO and explicit NEURAL selection, default prediction, and fallback ranking all use
the same device-class limits. Existing resolution/model filters and the 10% selection
tolerance remain in effect. An impossible pinned AUTO request still raises an error.

When no backend covers the requested resolution, AUTO falls back to StereoDepth
`FAST_ACCURACY` if the device supports StereoDepth. Otherwise it raises an error that
names the resolution.
