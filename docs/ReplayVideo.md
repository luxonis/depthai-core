# ReplayVideo

`dai::node::ReplayVideo` reads a video file, a directory of frames, or an archive and sends `ImgFrame` messages. On RVC4 it runs on the device unless `setRunOnHost(true)` is set. On RVC2, and in a pipeline with no device, it runs on the host. `setRunOnHost(false)` on RVC2 or with no device fails when the pipeline starts.

`ReplayMetadataOnly` (also called `ReplayMessage`) replays one `.mcap` file with the stored messages. It follows the same host and device rule.

Setters record the configuration. The pipeline checks it at start.

## Sources

`setReplayVideoFile(path)` is a host path. `setSource(path, DEVICE)` is an absolute path that is already on the device. A relative device path fails at start.

The host path may be:

- an `.mp4` file with H.264, H.265, or MJPEG
- an `.avi` file with FFV1 or MJPEG
- a directory of `<stream>_<number>.dai`, `.jpg`, or `.png` files
- a `.tar`, `.tar.gz`, or `.tar.xz` archive of one of those

Video must be 8-bit 4:2:0. VP8 and VP9 are not accepted. A `.dai` file that holds an H.264 or H.265 `EncodedFrame` fails at start. `BITSTREAM` fails at start.

`setArchiveEntry(name)` selects one stream inside an archive or a holistic directory. The entry is the record name, for example `CameraCAM_A`. It can name a directory, `<name>.mp4`, `<name>.avi`, or `<name>.mcap`.

`setReplayMetadataFile` attaches an `ImgFrame` `.mcap` sidecar. For a directory, `<stream>.mcap` is used when it is present. A `.dai` stream ignores that sidecar and logs one warning. The message count and the frame size in the sidecar must match the frames.

## Output

The default output type is `YUV420p`. `setOutFrameType` accepts `NV12`, `YUV420p`, `GRAY8`, `RGB888i`, and `BGR888i`. An unset `.dai` stream keeps the type stored in the files, including a type outside that set such as `RAW16`. Setting a type or a size on a `RAW16` stream fails at start.

`setSize` resizes. Both sides must be positive and even. The check runs at start. A size equal to the source is not resized, and the bytes stay equal. The frame transformation is scaled by the same ratio. There is no half-pixel shift.

`getOutFrameType()` returns the configured type. Until `setOutFrameType` is called, that value stays `YUV420p` even when a `.dai` stream is sent in its stored type.

JPEG and MJPEG use full range. Other conversions use BT.601 limited range. PNG color type 3 and 16-bit PNG fail at start.

## Rate, loop, and timestamps

`setFps` sets the send rate. `0` is the same as not set. With a positive fps, including `1000`, the node waits `1/fps` between sends. `fps = 1000` is one frame per millisecond. It does not drop frames when the consumer is slow. The next send waits until the previous one has finished.

Without fps, the node uses the recorded device-timestamp gaps. A gap that is not positive becomes 1/30 s. With no timestamps at all, the rate is 30 fps.

On the first pass the recorded sequence numbers and timestamps are kept. Later passes continue the sequence and shift the timestamps by the span of the recording plus one nominal interval. The nominal interval is `1/fps` when fps is set and is not exactly 1000. When fps is unset or exactly 1000, it is the mean of the recorded device timestamps, or 1/30 s for a single frame.

`ts`, `tsDevice`, and `tsSystem` move by the same amount. A stream with no metadata uses the scheduled send time for `ts` and `tsDevice`, leaves `tsSystem` unset, and numbers sequences from 0.

`setLoop(true)` is the default. The node returns to the first frame after the last one. `setStopPipelineAtEnd` has no effect while looping. With loop off, the node logs `Replay finished` and, by default, stops the pipeline. `setStopPipelineAtEnd(false)` ends the node thread and leaves the pipeline running.

## Cache

Host files are read in place. An archive is extracted into the host cache. A source that is sent to an RVC4 device is uploaded once, in 1 MiB chunks, under its SHA-256. A later run with the same bytes logs `Replay cache hit <hash>` and does not upload again.

The default limit is 32 GiB. `ReplayVideo::setReplayCacheLimit` changes it and does not delete anything by itself. Entries that are not in use are removed when a new entry would pass the limit or the free space. The oldest entry goes first. An entry larger than the limit fails before anything is removed.

`setPersistentUpload(true)` keeps the entries created by that start after the pipeline stops. The default removes them. The flag stays on once it is set. `DEPTHAI_REPLAY_CACHE` selects the host cache directory. `DEPTHAI_REPLAY_PERSISTENT_UPLOAD=1` and `DEPTHAI_REPLAY_DECODE_MODE=PRE_DECODE` apply when the setters were not called.

`ReplayVideo::getReplayCacheInfo` and `clearReplayCache` read and clean the host cache. The same methods on `Device` talk to the RVC4 cache. They throw on any other platform. The device methods need firmware that implements the cache RPC. This repository does not contain that firmware.

`PRE_DECODE` decodes every frame before the pipeline starts and reuses that set on the next run. `STREAM` is the default. It checks the source at start, decodes the first frame, and decodes the rest during the run. A frame that changes or fails during the run stops the pipeline.

## Camera and models

`Camera::setMockIsp` and `Camera::build` link the replay output to `mockIsp`. They set the output type to `NV12` on RVC4 and to `YUV420p` on RVC2, and they copy the size and fps onto the mock ISP. A device path with no `setSize` asks the device for the size. An archive with no `setSize` fails in `setMockIsp` because the size is not known until the recording is opened by holistic replay.

`NeuralNetwork::build(ReplayVideo, model, fps)` sets the type, size, and fps from the model. A model type outside the five supported types fails and the error names the type.

## Holistic replay

`Pipeline::enableHolisticReplay(path)` still builds the replay nodes. On RVC2 it extracts the recording on the host, as before. On RVC4 it does not extract the recording for the run. Each camera gets one `ReplayVideo`, and each other source, including the IMU, gets one `ReplayMetadataOnly`. Every node uses the recording path and `setArchiveEntry` with the record name. The nodes loop, leave fps unset, and use `NV12` unless the legacy flag asks for `YUV420p`.

The whole recording is one cache entry. A directory is packed into one tar so it can be uploaded as that entry. The pack is not the size probe. The probe reads the video, the first and last `.dai` frame, or the `.mcap` so the camera mock ISP size is known before start.

A camera stream is a video plus metadata, or a directory of `.dai` frames, or a `.mcap` of full frames. The metadata count must match the frame count. A recording that breaks that rule fails at start.

## What this repository does not do

RVC4 firmware is not in this repository. The host sends the node properties and the cache RPC. The hardware decoder, the on-device replay thread, and the camera mock-ISP checks (a frame whose size is not the mock ISP size, and a separate frame object per camera output) run in firmware.

There is no GStreamer path. When libav is available it is used for video. Otherwise OpenCV decodes through its video reader and the color conversion is lossy.

A crashed process can leave a cache entry marked in use. Nothing reaps it. `clearReplayCache` skips entries that are in use.
