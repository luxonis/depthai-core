# OpticalFlow (RVC4)

`OpticalFlow` computes native bidirectional dense EVA motion between consecutive
consumed images. It has no corner detector, descriptors, feature selection or
feature maintenance. It reuses the accelerator implementation used internally by FeatureTracker.
Its output packer creates owning snapshots. Two instantiated nodes compute independently;
this is code reuse, not sharing a computed flow field.

## Contract

- `inputImage`: GRAY8 or RAW8. Width 128..1920 divisible by 128;
  height 128..1056 divisible by 32. Dimensions and format stay fixed.
  Rows must be packed: stride equals width (an implicit stride of zero is also
  accepted), first-plane and memory offsets are zero, and the payload contains at least
  `width * height` bytes. Extra trailing allocation padding is allowed. Padded
  rows, offset planes and truncated or missing payloads are rejected before
  accelerator initialization or execution. Repack such images upstream.
  Default queue: blocking, four entries. A nonblocking queue may drop frames;
  packets always identify the actual consumed pair, including sequence gaps.
- `passthroughInputImage`: each accepted input, including the first frame.
- `outputFlow`: no first-frame output. Thereafter one owning `MessageGroup` per
  consumed pair, with the layout described below.
  No accelerator initialization, execution or packing when outputFlow is unused.
- `flow`: NNData with FP32 HWC `forward` / `backward`, shape `[H,W,2]`, channels
  dx,dy in input pixels per pair. Forward lives on the reference grid and maps
  to target; backward lives on target and maps to reference. U8 `confidence`
  is `[H,W,1]`, one raw score map, not a probability or backward score.
- `reference` / `target`: metadata-only Buffer copies retaining exact sequence
  and all source timestamps. Group and NNData metadata identify target.
- Maps own their storage and are unchanged by subsequent accelerator calls.
  There is no CPU flow, interpolation, validity masking or exposure scaling.
  Current backend vectors are integer-valued, exported as FP32; floating-point
  storage does not imply subpixel estimation.

The native ABI does not explicitly define the confidence coordinate domain.
Existing FeatureTracker samples it on the reference grid; independent matching
checks remain appropriate. Kernel estimation must use actual pair timestamps.
The output costs about 17 bytes per input pixel (4.18 MB at 640x384).

Requires matching host SDK and RVC4 runtime. RVC2 is rejected; no software
fallback is provided. Unsupported input layout/payload or changing geometry/format stops the
node with an error rather than reusing incompatible accelerator buffers.

## Validation

Mirrored C++/Python examples live under `OpticalFlow/`. Host tests validate
packing ownership/metadata and geometry/layout/payload guards.
The CTest target `optical_flow_device_test` (labels `ondevice`, `rvc4`,
`ci_nightly`, `noreplayci`) exercises four supported geometries from 128x128 to
1920x1056, identity and signed 2D translations, pair timestamps and sequence
numbers, passthrough bytes and retained output ownership without downloaded
assets or OpenCV. `tests/hardware/optical_flow.py`
checks identity, signed translations, independently moving box/background,
first-frame behavior, exact pair IDs/timestamps and retained-packet ownership.

`tests/hardware/optical_flow_matrix.py` adds configurable GRAY8/RAW8 resolution
tests, vertical/diagonal motion and multiple moving regions, live throughput,
queue drops, malformed-input rejection with recovery, shutdown/restart and a
FeatureTracker regression. It requires an idle device and never stops apps.
For example, run it with `--device DEVICE --width 1920 --height 1056
--mode synthetic --output results`. Live capture also accepts `--socket CAM_A`;
the selected sensor must support the requested dimensions. Reported live
throughput includes full flow transfer to the host, not just accelerator time.
