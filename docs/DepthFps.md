# Depth FPS selection

Depth selection uses one FPS limit per neural model on RVC4 for both Lite and OAK4-D, regardless of the reported OS version.
The limits are based on two 30-second runs per model with live 1280x800 stereo input
requested at 60 FPS and on-device depth FPS measurement. Both devices used DepthAI 3.12.1:
OAK4-LITE-TOF R1 (2080684916, OS 1.49.1) and OAK4-D R9 (3514489054, OS 1.47.0).

| Model | FPS limit | Lite mean FPS | OAK4-D mean FPS |
|---|---:|---:|---:|
| 1248x780 | 20 | 21.78 | 21.78 |
| 1056x660 | 30 | 31.50 | 31.48 |
| 960x600 | 38 | 39.04 | 38.82 |
| 864x540 | 45 | 46.81 | 46.47 |
| 768x480 | 60 | 60.00 | 59.98 |
| 576x360 | 60 | 59.99 | 60.07 |
| 480x300 | 60 | 60.00 | 60.02 |
| 384x240 | 60 | 60.00 | 60.00 |
| 288x180 | 60 | 60.01 | 60.01 |
| 192x120 | 60 | 59.99 | 60.01 |

The four largest models were within 0.74% across these short runs. The remaining
models reached the camera's 60 FPS input rate; their true maximum may be higher.
These measurements do not establish sustained performance after thermal soak.

When a Depth node resolves to NeuralDepth on OS older than 1.47.0 (or an unknown version),
it logs a warning. Version 1.47 is the earliest verified version for the faster neural
models; this is a recommendation, not a compatibility gate. Selection and camera FPS
stay the same. Actual throughput on older software may be lower.

AUTO and explicit NEURAL selection, default prediction, and fallback ranking all use
the same FPS limits. Existing resolution/model filters and the 10% selection
tolerance remain in effect. An impossible pinned AUTO request still raises an error.

When no backend covers the requested resolution, AUTO falls back to StereoDepth
`FAST_ACCURACY` if the device supports StereoDepth. Otherwise it raises an error that
names the resolution.
