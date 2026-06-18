# RVC4 ToF Examples (VD55H1)

Python examples for the unified `dai.node.ToF` API on **RVC4** (ST VD55H1 sensor).

On RVC4:
- **IPP runs on device** — `tof.depth` is final IPP depth
- **No host ImageFilters** — use `ToFPreset` instead of `ImageFiltersPresetMode`
- **Auto-camera** — `tof.build()` creates and links `Camera.raw → tof.rawInput` unless you connect `rawInput` yourself first
- **Do not use** `tof.tofBaseNode` in application code — use `tof.initialConfig` and `tof.inputConfig` directly

## Examples in this folder

| Script | Purpose |
|--------|---------|
| [`tof_minimal.py`](tof_minimal.py) | Smallest pipeline — build + depth preview |
| [`tof_params_minimal.py`](tof_params_minimal.py) | Build, `initialConfig`, runtime `inputConfig` |
| [`tof_rvc4_config.py`](tof_rvc4_config.py) | Full CLI for every RVC4 option + live multi-output preview |

## C++ counterparts

See [`examples/cpp/RVC4/ToF/`](../../../cpp/RVC4/ToF/) (`tof_minimal`, `tof_params_minimal`).

## Prerequisites

Rebuild **Python bindings** after API changes (C++ `depthai-core` alone is not enough):

```bash
# from depthai-device repo root
TARGET=depthai ./scripts/build_depthai_core.sh
```

Run examples with local bindings:

```bash
cd external/depthai-core
PYTHONPATH=build/bindings/python \
DEPTHAI_DEVICE_NAME_LIST=<device-ip-or-mxid> \
python examples/python/RVC4/ToF/tof_params_minimal.py
```

No RVC4 FWP rebuild is required for host-side API changes if your gate firmware already supports ToF + `rawInput`.

---

## Quick start

```python
import depthai as dai

with dai.Pipeline() as pipeline:
    tof = pipeline.create(dai.node.ToF)

    tof.build(
        dai.CameraBoardSocket.AUTO,
        fps=10,
        preset=dai.ToFPreset.MID_RANGE,
    )

    depth_q = tof.depth.createOutputQueue()
    pipeline.start()
    frame = depth_q.get()
    print(frame.getWidth(), frame.getHeight())  # 804 x 672 (fixed F3_FULL capture)
```

---

## 1. Build parameters (`tof.build()`)

Called **before** `pipeline.start()`. Requires a connected RVC4 device on the pipeline.

```python
tof.build(
    boardSocket,           # positional: dai.CameraBoardSocket
    fps=10,                # integer value; stored as float on wire
    preset=...,            # RVC4: dai.ToFPreset
)
```

> **Capture mode is fixed to `F3_FULL`** in BETA and is not user-selectable (no `sensorMode`
> parameter). Output is always 804 × 672.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `boardSocket` | `CameraBoardSocket` | `AUTO` | Camera socket. `AUTO` picks first socket with `CameraSensorType.TOF`. |
| `fps` | `int` / `float` | sensor default | Integer frame rate (10–30 recommended on RVC4). Stored as `float` in `ToFProperties` / `CameraProperties` — same wire type as other camera nodes. Non-integer values are rejected. |
| `preset` | `ToFPreset` | *(none)* | IPP processing preset (`LOW_RANGE`/`MID_RANGE`/`HIGH_RANGE`/`FAST_OBJECTS`). Applied only when explicitly passed. Tune `tof.initialConfig` **after** `build()` for further overrides. **RVC4 only.** |

After build:

| Method | Returns | Description |
|--------|---------|-------------|
| `tof.getBoardSocket()` | `CameraBoardSocket` | Resolved socket |
| `tof.getOutputResolution()` | `(width, height)` | **IPP output size** for `depth`, `amplitude`, `confidence`, `intensity` |
| `tof.getRawResolution()` | `(width, height)` | **Raw superframe size** for manual `Camera.build(sensorResolution=...)` |
| `tof.getCamera()` | `Camera` or `None` | Auto-created camera after `pipeline.start()`, or `None` if you linked `rawInput` manually |
| `tof.getSensorResolution()` | `(width, height)` | **Deprecated** — alias of `getOutputResolution()` |

### Two resolutions on RVC4 (do not mix them up)

| What you need | Method | F3_FULL example |
|---------------|--------|-----------------|
| Size of `tof.depth` / `tof.amplitude` frames | `getOutputResolution()` | **804 × 672** |
| Size for `Camera.build(sensorResolution=...)` | `getRawResolution()` | **1344 × 7244** |

Auto-camera uses `getRawResolution()` internally. You only need it when wiring a manual `Camera`.

---

## 2. Resolution (fixed F3_FULL)

Capture is fixed to `F3_FULL` in BETA, so the two relevant sizes are constant:

| What you need | Method | Value |
|---------------|--------|-------|
| Size of `tof.depth` / `tof.amplitude` / `tof.confidence` / `tof.intensity` frames | `tof.getOutputResolution()` | **804 × 672** |
| `Camera.build(sensorResolution=...)` for a manual camera | `tof.getRawResolution()` | **1344 × 7244** |

```python
tof.build(dai.CameraBoardSocket.CAM_D, fps=10)
print(tof.getOutputResolution())  # (804, 672) — depth/amplitude/confidence frame size
print(tof.getRawResolution())     # (1344, 7244) — Camera raw superframe
```

Auto-camera uses `getRawResolution()` internally; you only need it when wiring a manual `Camera`.

---

## 3. IPP presets (`dai.ToFPreset`)

Replaces `ImageFiltersPresetMode` on RVC4. Selects phase-unwrap threshold and filter behavior.

| Preset | Phase unwrap threshold | IPP filters |
|--------|------------------------|-------------|
| `LOW_RANGE` | 50 | IPP defaults (bilateral, TNR, FPC, R2P enabled unless overridden) |
| `MID_RANGE` | 75 | same |
| `HIGH_RANGE` | 130 | same |
| `FAST_OBJECTS` | 75 | TNR **disabled** (reduces motion blur on fast scenes); other filters at IPP defaults |
| `OFF` *(internal)* | 10000 | Bilateral, TNR, flying-pixel, R2P **disabled**. Not part of the public BETA surface. |

```python
tof.build(..., preset=dai.ToFPreset.MID_RANGE)
# or set on initialConfig after build():
tof.initialConfig.setToFPreset(dai.ToFPreset.MID_RANGE)
```

---

## 4. IPP `initialConfig` (device-side)

Set on `tof.initialConfig` **after** `tof.build()` and **before** `pipeline.start()`. These fields are sent to the gate IPP pipeline at start.

| Field | Type | Description |
|-------|------|-------------|
| `phaseUnwrapErrorThreshold` | `int` | Phase unwrap error limit (uint16 on wire). Preset sets this; override manually if needed. |
| `enableBilateralFilter` | `bool` or `None` | `True` = on, `False` = bypass, `None` = IPP default |
| `bilateralStdFactor` | `float` or `None` | Bilateral std factor |
| `bilateralFilterKernelSize` | `int` or `None` | Bilateral kernel size |
| `enableTemporalNoiseReduction` | `bool` or `None` | TNR on/off |
| `tnrMaxGain` | `float` or `None` | TNR max gain |
| `tnrStdFactor` | `float` or `None` | TNR std factor |
| `enableFlyingPixelCorrection` | `bool` or `None` | Flying-pixel correction on/off |
| `fpDepthThreshold` | `float` or `None` | Flying-pixel depth threshold |
| `fpMinDepthOccurrence` | `float` or `None` | Flying-pixel min occurrence |
| `enableRadialToPerp` | `bool` or `None` | Radial-to-perpendicular depth conversion |

Example:

```python
tof.build(..., preset=dai.ToFPreset.MID_RANGE)
cfg = tof.initialConfig
cfg.enableBilateralFilter = True
cfg.enableTemporalNoiseReduction = True
cfg.enableFlyingPixelCorrection = True
cfg.enableRadialToPerp = False
cfg.phaseUnwrapErrorThreshold = 75
```

View type helper (optional):

```python
ipp = dai.ToFIppConfig.fromToFConfig(tof.initialConfig)
ipp.applyPreset(dai.ToFPreset.LOW_RANGE)
ipp.applyTo(tof.initialConfig)
```

---

## 5. Runtime config (`tof.inputConfig`)

Send updated IPP parameters while the pipeline is running:

```python
cfg = tof.initialConfig
cfg.enableRadialToPerp = True
cfg_q = tof.inputConfig.createInputQueue()
cfg_q.send(cfg)
```

---

## 6. Outputs

| Output | RVC4 type | Description |
|--------|-----------|-------------|
| `tof.depth` | `ImgFrame` `RAW16` | **Final IPP depth** in mm (use this) |
| `tof.amplitude` | `ImgFrame` `RAW8` | Per-pixel amplitude, log1p-normalized uint8 |
| `tof.confidence` | `ImgFrame` `RAW8` | IPP confidence, max-entropy log1p uint8 (connect only if needed) |
| `tof.intensity` | `ImgFrame` `RAW8` | Ambient / intensity, log1p-normalized uint8 |
| `tof.rawDepth` | alias of depth | Same IPP output as `depth` on RVC4 — accessing it logs a warning; use `depth` |
| `tof.raw` | sensor passthrough | Preferred path for raw sensor frames on RVC4 (via the node) |
| `tof.phase` | not produced on RVC4 | Deprecated (RVC2 only); accessing on RVC4 logs a warning |

Optional outputs are only produced when connected (queue or link) before start.

```python
depth_q = tof.depth.createOutputQueue()
conf_q = tof.confidence.createOutputQueue()   # opt-in
amb_q = tof.intensity.createOutputQueue()     # opt-in
```

---

## 7. Auto-camera vs manual `rawInput`

**Default (auto-camera):** `tof.build()` → at pipeline build time, host creates `Camera`, sets `CameraSensorType.TOF`, links `camera.raw → tof.rawInput`.

**Override:** connect `rawInput` **before** `pipeline.start()`:

```python
tof.build(dai.CameraBoardSocket.CAM_D, fps=10)

cam = pipeline.create(dai.node.Camera)
cam.setSensorType(dai.CameraSensorType.TOF)
cam.build(tof.getBoardSocket(), sensorResolution=tof.getRawResolution(), sensorFps=10)
cam.raw.link(tof.rawInput)
# tof.getCamera() is None
```

Use `getRawResolution()` for `Camera.build(sensorResolution=...)`, not `getOutputResolution()`.

---

## 8. RVC2-only fields (ignored on RVC4)

Do not rely on these on RVC4 — they apply to the Myriad decoder path only:

- `median`
- `phaseUnwrappingLevel`
- `enablePhaseShuffleTemporalFilter`
- `enableBurstMode`
- `enableDistortionCorrection`
- `enableFPPNCorrection`, `enableOpticalCorrection`, `enableTemperatureCorrection`, `enableWiggleCorrection`, `enablePhaseUnwrapping`

Also unused on RVC4: `tof.imageFiltersNode`, host `ImageFiltersPresetMode` on build (warning logged; use `ToFPreset`).

---

## 9. CLI reference (`tof_rvc4_config.py`)

Print all settings without a device:

```bash
python examples/python/RVC4/ToF/tof_rvc4_config.py --print-only
```

| Flag | Maps to |
|------|---------|
| `--socket` | `boardSocket` |
| `--fps` | `fps` (integer) |
| `--preset` | `ToFPreset` (`LOW_RANGE`/`MID_RANGE`/`HIGH_RANGE`/`FAST_OBJECTS`) |
| `--phase-unwrap` | `phaseUnwrapErrorThreshold` |
| `--no-ipp-filters` | `ToFPreset.OFF` |
| `--bilateral`, `--bilateral-std`, `--bilateral-kernel` | bilateral IPP |
| `--tnr`, `--tnr-max-gain`, `--tnr-std` | TNR IPP |
| `--fpc`, `--fp-depth-th`, `--fp-min-occ` | flying-pixel IPP |
| `--r2p` | `enableRadialToPerp` |
| `--manual-camera` | skip auto-camera |
| `--amplitude`, `--confidence`, `--ambient` | extra output queues |

---

## 10. Troubleshooting

| Issue | Fix |
|-------|-----|
| `AttributeError: ToFPreset` | Rebuild Python bindings: `TARGET=depthai ./scripts/build_depthai_core.sh` |
| `build() got an unexpected keyword argument 'sensorMode'` | `sensorMode` was removed in BETA; capture is fixed to `F3_FULL`. Drop the argument. |
| `rawInput` not connected | Call `tof.build()` before start, or link `cam.raw → tof.rawInput` manually |
| `ToF fps must be an integer value` | Pass whole-number fps (e.g. `10`, not `10.5`) |
| Old script uses `tof.tofBaseNode` | Replace with `tof.initialConfig`, `tof.inputConfig`, `tof.rawInput` |
