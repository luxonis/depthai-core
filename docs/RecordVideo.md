# RecordVideo

`dai::node::RecordVideo` writes `ImgFrame` or `EncodedFrame` messages to disk. It runs on an RVC4 device unless `setRunOnHost(true)` is set. On RVC2, and in a pipeline with no device, it runs on the host. `setRunOnHost(false)` on RVC2 or with no device fails when the pipeline starts.

The node does not change the frames it receives. `input` accepts `ImgFrame` and `EncodedFrame`, does not block, and keeps the 15 newest messages. `inputControl` accepts `RecordControl`, does not block, and keeps 4 messages. `status` sends one `RecordStatus` for every state change.

## Formats

`setFormat(RecordFormat::VIDEO)` is the default. It writes a video file and a `.mcap` metadata file in `<output directory>/<recording name>/`. The stream name is the file stem. Segment 0 has no suffix. Later segments insert `_s<segment>` before the extension, for example `frames_s1.mp4`.

- An `EncodedFrame` becomes an `.mp4` file. H.264 and MJPEG are supported. H.265 fails with `Record H265 is not supported in VIDEO format`.
- An `ImgFrame` on the host becomes an `.avi` file. An `ImgFrame` on the device is rejected. Encode it with `VideoEncoder`, or use `RAW_FRAMES`.
- A `BITSTREAM` `ImgFrame` is rejected in both formats.
- `setFps` is stored in the video file. It is not measured from the stream. The default is 30.

`setFormat(RecordFormat::RAW_FRAMES)` writes one `<recording>/<stream>/<stream>_<sequence>.dai` file per frame. The sequence is eight digits when it fits. `sequenceNum` below 0 is an error, and a file that already exists is an error. H.264 and H.265 frames are written and produce one warning that they cannot be replayed.

`setCompressionLevel(CompressionLevel::NONE)` writes an uncompressed `.mcap` file. Every other level uses LZ4.

## Names and directories

`setRecordingName`, `setOutputDirectory`, and `setStreamName` are checked when the pipeline starts. A name may contain letters, digits, `.`, `_`, and `-`. It cannot be empty, `.`, or `..`. A stream name cannot end in `_` plus digits. `cam_` is valid. `cam_12` is not.

The default recording name is `recording_<local time with milliseconds>`. The default output directory is the current directory. The default stream name is `frames`. The first host node that uses a recording name creates `<output directory>/<recording name>`. If that directory already exists, the pipeline does not start.

## Trigger

Without `setWaitForTrigger(true)` the node opens segment 0 and sends `STARTED` before the first frame. With it, the node stays idle until `RecordControl::start()` arrives on `inputControl`.

While idle, the node keeps the newest `setPreTriggerFrames` frames. The default is 60. Zero keeps none. The buffer also stays within `setMaxPreTriggerBytes` (default 256 MiB). If the first idle frame shows that the frame count times its size exceeds that budget, the node sends `ERROR` and stops the pipeline. The message contains both byte counts. When a later frame would pass the byte budget, the oldest frame is dropped and the node warns once.

`RecordControl::start(maxFrames, maxDurationMs, maxSizeKb)` stops the segment when any limit is reached and leaves the node idle until the next start. A value below 0 turns that limit off. A start with 0 in any of the three is ignored. `maxFrames` counts written frames and includes the pre-trigger frames that are written. The newest `maxFrames` buffered frames are kept. `maxDurationMs` is measured from the moment the start is applied, on the steady clock. A frame received at or after that time is not written. `maxSizeKb` counts the payload bytes of written frames, 1024 bytes per unit. The frame that reaches the size is written, and the segment then stops. The newest pre-trigger frames that fit in that budget are kept; one frame is kept when it alone is larger than the budget. Container and metadata bytes are not counted. `RecordControl::stop()` closes the current segment. A start received while recording is ignored.

A segment with no frames still sends `STARTED` and `STOPPED` and writes no media file. Stopping the pipeline while recording closes the segment and sends `STOPPED`. Stopping it while idle discards the pre-trigger buffer and sends no status.

## Status

`RecordStatus` reports `STARTED`, `STOPPED`, or `ERROR`. `segment` starts at 0. `frameCount`, `firstSequenceNum`, `lastSequenceNum`, and `skippedFrames` describe the frames written in that segment, including pre-trigger frames. `path` is `<output directory>/<recording name>` on the host. `error` is empty unless the state is `ERROR`.

`sequenceNum` on the status message counts from 0. Its timestamps are the time of the state change on the steady clock. `getStatus()` returns the last status, including when `status` is not linked. After `ERROR` the node does not also send `STOPPED`.

A gap in written sequence numbers adds `sequence - previous - 1` to `skippedFrames` and logs a warning. A sequence number that does not increase is still written and logs `Record sequence went back`. The first frame of a segment does not count as a skip. If the first written frame implies more than 100 MB/s at the configured fps, the node warns once that frames can be skipped. It does not drop that frame itself.

## Device store

On RVC4 the recording is created in `/data/depthai/recordings/<name>`. `Device::listRecordings`, `downloadRecording`, and `deleteRecording` talk to that store. They throw on any other platform. A name that fails the rules above is rejected before the device is called.

`setTransferToHost(true)` is the default. When the pipeline stops, the host downloads `<output directory>/<name>/` and then deletes the device copy unless `setKeepOnDevice(true)`. A download logs progress every 10 percent. A failed download is logged, left on the device, and does not throw out of `Pipeline::stop`. Nothing is downloaded after the device disconnects. Calling `setTransferToHost` or `setKeepOnDevice` on a node that runs on the host logs that the call was ignored.

`prepareRecordingDownload` stops and joins the device recording nodes before a download. Pipeline stop does this while status transport is still running, including when transfer is disabled, and retains the final status for `getStatus()`. The RVC4 firmware in `depthai-device-kb` implements these calls and uses the matching SDK headers. Its `DEPTHAI_DEVICE_KB_CORE_PATH` must point to that SDK checkout.

The store records creation time and completion in persistent bookkeeping. A recording is complete after every writer has closed its files; an interrupted process leaves it incomplete. Download and delete reject recordings held by a writer or replay reader. Downloads contain payload files, excluding store bookkeeping and unfinished frame files. Device pre-trigger history owns copies of idle frames so it can release camera pool buffers immediately. Encoded MP4 recording uses the shared MP4 backend without requiring OpenCV.

## Legacy files

`setRecordVideoFile` and `setRecordMetadataFile` keep the previous layout: the host writes those exact paths. Later segments insert `_s<segment>` before the extension. `setOutputDirectory` and `setStreamName` are ignored with a warning. `RAW_FRAMES` cannot be combined with these setters.

On RVC4, a legacy node whose input is only `ImgFrame` stays on the host. `setRunOnHost(false)` in that case fails at start. A legacy node fed by `EncodedFrame` runs on the device and the files are copied to the configured host paths when the pipeline stops.

## Holistic record

`RecordConfig.format` is `VIDEO` or `RAW_FRAMES`. `keepOnDevice` is false by default. Both keys are optional in `record_config.json`. An unknown `format` string throws `std::invalid_argument` when the pipeline starts. `streams` stores the fps of each camera stream.

On RVC2, holistic record still uses one host file per stream, sets that stream's fps, and makes the record input blocking. The archive written at stop is `recording_<local time>.tar.gz`.

On RVC4, one device `RecordVideo` is created per camera. The cameras share `holistic_<device id>_<local time>`. The stream name is `Camera` plus the board socket, for example `CameraCAM_A`. `VIDEO` with encoding enabled records the encoded stream on the device. `RAW_FRAMES` records the camera frames and does not add a `VideoEncoder`. `VIDEO` with encoding disabled cannot write an AVI on the device, so the host nodes are used and the pipeline warns `Record holistic: video encoding off, host nodes used`.

At stop the host downloads the shared recording, writes `record_config.json` and `camera_info.json`, and packs `recording_<local time>.tar.gz`. Video entries stay at the archive root (`CameraCAM_A.mp4` and `.mcap`). Raw frames stay in one directory per camera (`CameraCAM_A/CameraCAM_A_00000000.dai`). IMU data stays in `IMU.mcap` on the host. Hidden marker files are not packed. Older `.tar` recordings remain readable. `ReplayVideo` plays `.dai` files. See `docs/ReplayVideo.md`.

Holistic nodes do not wait for a trigger and do not keep a pre-trigger buffer. Sync and `MessageDemux` stay on the device on RVC4. The demux stays on the host for RVC2 and for the host fallback above.

## What changed from earlier releases

- `input` no longer blocks. A full queue drops the oldest message.
- `setFps` is no longer measured from the stream. The default is 30.
- `CompressionLevel::NONE` is uncompressed. It used to be overwritten with LZ4.
- On RVC4, a new-layout node and a legacy node fed by `EncodedFrame` run on the device. Encoded record tests that must stay on the host call `setRunOnHost(true)`.
- The node can run on the host or on an RVC4 device. The same configuration produces the same files, status, and skip counts, except that device `VIDEO` cannot record a raw `ImgFrame` and an RVC4 frame can be larger because of stride padding.
