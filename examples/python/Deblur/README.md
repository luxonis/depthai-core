# Deblur (RVC4, experimental)

`Deblur` consumes `OpticalFlow.outputFlow` and the corresponding
`OpticalFlow.passthroughInputImage`. It applies spatially varying, quantized
motion kernels to packed GRAY8 images. It does not estimate flow again.
C++ and Python live examples are provided in their respective Deblur directories.
Width must be 128..1920 and divisible by 128; height must be 128..1056 and
divisible by 32. Rows are packed and plane/memory offsets must be zero.

Run `python3 examples/python/Deblur/deblur.py --device DEVICE --show` to display
original and restored images, or omit `--show` for headless statistics. The C++
`deblur` example accepts an optional device ID/address as its first argument.
Neither example requires a recording or writes image files. The supplied live
configuration requests 640x384 from CAM_B; choose a sensor supporting that size.

```python
flow = pipeline.create(dai.node.OpticalFlow)
deblur = pipeline.create(dai.node.Deblur)
camera.requestOutput((640, 384), type=dai.ImgFrame.Type.GRAY8, fps=15).link(flow.inputImage)
flow.passthroughInputImage.link(deblur.inputImage)
flow.outputFlow.link(deblur.inputFlow)
restored = deblur.output.createOutputQueue()
```

Only restored images need to leave the device. `passthroughInputImage` and
`outputMask` are optional comparison/diagnostic outputs.

## Behavior and limits

Images and both flow endpoints must match by sequence number AND device timestamp.
A bounded cache retains reference images and pending targets. Image payloads are
copied on entry so history cannot exhaust the camera's capture pool. A target is emitted
once a matching pair is available, its wait expires, or cache pressure requires
passthrough. Missing reference/flow, first frames, invalid fields, missing exposure
or exposure exceeding the pair interval result in unchanged output. Out-of-order
flow is accepted within the cache window. Late packets cannot alter emitted images
and are evicted from the bounded flow cache. Output
follows consumed image order. Nonblocking inputs may drop frames under load.
Use a cache/timeout appropriate to processing latency; no indefinite wait for flow.
The timeout excludes restoration time and downstream output backpressure.

Backward flow is defined on the current image grid. Its displacement is scaled by
exposure divided by the actual reference/target device timestamp interval. Exposure
comes from target metadata, or the explicit `exposureTimeUs` override. Frame rate
is never substituted for measured interval. Motion is assumed constant over this
interval and the exposure; rolling shutter and acceleration are not modeled.

Forward/backward consistency, smoothed photometric agreement and local texture
validate each pixel. Raw EVA confidence is sampled at the mapped reference point,
following FeatureTracker's convention, not a guaranteed ABI probability. Similar
vectors share a centered line-spread Wiener FIR kernel. This is causal approximate
restoration, not the joint variational method or lossless recovery. Integer native
flow precision, clipped frequencies, occlusions and noise limit recoverable detail.
Filtering uses stored GRAY8 intensities without radiometric linearization.

The GPU performs FIR filtering; correspondence checks and kernel preparation run
on CPU. `useOpenCL=false` selects the CPU reference. GPU failure is explicit and
stops this node without silently switching backend. Input images are never mutated.
Corrections are limited by `maxCorrection`; pixels with zero outputMask are exactly
preserved. The mask stores the effective blend weight, not measured improvement.
Malformed image input stops this node; malformed flow bypasses affected frames.

Only source and generated tests belong to this change. No recordings, captured
fixtures, binaries, local addresses or generated outputs are included. Matching
firmware and OpticalFlow support are required. Hardware/performance validation and
upstream firmware publication remain merge gates until recorded as complete.

## Generated tests

`deblur_test` checks known horizontal/vertical local blur, static-region identity,
correction bounds, exposure scaling, invalid fields, ordered/bounded rendezvous,
and configuration serialization. `deblur_device_test` compares the CPU reference
and GPU at 256x128, 640x384, 1280x736 and 1920x1056, exercises native EVA connected
directly to Deblur, checks idle/backpressured shutdown and invalid-input
isolation, and runs live streams beyond camera-pool/cache capacity at three
resolutions through 1920x1056. Restoration fixtures and supplied flow packets are
generated in memory; live tests capture directly from the camera. No external
fixtures are required. These controlled tests do not establish improvement for
all real scenes.
