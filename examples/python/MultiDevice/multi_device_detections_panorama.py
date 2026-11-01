#!/usr/bin/env python3
"""Show detections and a calibrated panorama in separate windows. Keys: 1=Off, 2=NMS, 3=Average, W/S=IoU +/-0.05, Q=quit."""
import argparse
from datetime import timedelta

import cv2
import depthai as dai

FPS = 30
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("calibration", help="JSON saved by the MultiDeviceCalibration example")
parser.add_argument("model", help="Model zoo slug, e.g. yolov6-nano; all cameras must share label IDs")
parser.add_argument("devices", nargs="+", help="At least two device IDs or IP addresses")
parser.add_argument("--projection", choices=["Perspective", "Equirectangular", "Cylindrical"], default="Cylindrical")
parser.add_argument("--average", action="store_true", help="Average duplicate boxes and union their masks instead of NMS")
parser.add_argument("--blend", action=argparse.BooleanOptionalAction, default=True,
                    help="Enable exposure compensation and multiband blending (default: on; --no-blend is faster)")
parser.add_argument("--panorama-scale", type=int, choices=range(1, 5), default=2,
                    help="Stitching resolution multiplier for 640x400 camera views (default: 2)")
parser.add_argument("--sync-threshold-ms", type=int, default=1000 // FPS,
                    help=f"Maximum timestamp spread in milliseconds (default: {1000 // FPS}; does not synchronize capture)")
args = parser.parse_args()
if len(args.devices) < 2:
    parser.error("at least two devices are required")
if not 0 < args.sync_threshold_ms <= 1000 / FPS:
    parser.error(f"sync threshold must be positive and at most one frame period ({1000 / FPS:.1f} ms at {FPS} FPS)")

syncThreshold = timedelta(milliseconds=args.sync_threshold_ms)
with dai.Pipeline(createImplicitDevice=False) as pipeline:
    # Apply the graph before starting: all camera transformations must use a common coordinate system.
    calibration = dai.beta.MultiDeviceCalibrationHandler(args.calibration)
    pipeline.setMultiDeviceCalibration(calibration.getGraph())
    detectionSync = pipeline.create(dai.node.Sync)
    detectionSync.setRunOnHost(True)
    detectionSync.setSyncThreshold(syncThreshold)
    demux = pipeline.create(dai.node.MessageDemux)
    demux.setRunOnHost(True)
    detectionSync.out.link(demux.input)
    merged = pipeline.create(dai.node.ImgDetectionsFilter)
    overlapModes = (dai.ImgDetectionsFilterConfig.OverlapMode.OFF,
                    dai.ImgDetectionsFilterConfig.OverlapMode.NMS,
                    dai.ImgDetectionsFilterConfig.OverlapMode.AVERAGE)
    modeIndex = 2 if args.average else 1
    iouThreshold = 0.4
    merged.initialConfig.setConfidenceRange(0.5)
    merged.initialConfig.overlapMode = overlapModes[modeIndex]
    merged.initialConfig.overlapIouThreshold = iouThreshold
    configQueue = merged.inputConfig.createInputQueue(maxSize=1, blocking=False)
    views = []
    displayQueues = []
    for index, identifier in enumerate(args.devices):
        device = pipeline.addDevice(dai.DeviceInfo(identifier))
        camera = pipeline.create(dai.node.Camera, device).build(dai.CameraBoardSocket.CAM_A, sensorFps=FPS)
        model = dai.NNModelDescription(args.model, platform=device.getPlatformAsString())
        network = pipeline.create(dai.node.DetectionNetwork, device).build(camera, model, fps=FPS)
        key = f"cam{index}"
        network.out.link(detectionSync.inputs[key])
        # MessageDemux accepts Buffer outputs; declare the concrete type for the typed filter inputs.
        demux.outputs[key].setPossibleDatatypes([(dai.DatatypeEnum.ImgDetections, False)])
        demux.outputs[key].link(merged.inputs[key])
        # Share one NV12 transfer between stitching and preview; separate BGR streams saturate Gigabit Ethernet.
        view = camera.requestOutput((640 * args.panorama_scale, 400 * args.panorama_scale),
                                    type=dai.ImgFrame.Type.NV12, fps=FPS, enableUndistortion=True)
        views.append(view)
        # Pair the undistorted view with detections from the same capture.
        cameraDisplay = pipeline.create(dai.node.Sync)
        cameraDisplay.setRunOnHost(True)
        cameraDisplay.setSyncThreshold(timedelta(milliseconds=1))
        view.link(cameraDisplay.inputs["image"])
        network.out.link(cameraDisplay.inputs["detections"])
        displayQueues.append((f"Camera {index}: {identifier}", cameraDisplay.out.createOutputQueue(maxSize=1, blocking=False)))

    stitching = pipeline.create(dai.node.Stitching).build(views)
    stitching.setMode(dai.node.Stitching.Mode.PANORAMA)
    stitching.setUseInputCalibration(True)
    stitching.setSeamFinder(dai.node.Stitching.SeamFinder.GRAPHCUT_COLOR if args.blend else dai.node.Stitching.SeamFinder.NONE)
    stitching.setCameraModel(getattr(dai.CameraModel, args.projection))
    stitching.setMaxPanoramaSize(1600 * args.panorama_scale, 800 * args.panorama_scale)
    stitching.setSyncThreshold(syncThreshold)
    stitching.out.link(merged.inputReference)
    for index in range(len(views)):
        stitching.outSourceMasks[f"input{index}"].link(merged.inputSourceMasks[f"cam{index}"])

    # Match by host timestamp: different devices have independent sequence counters.
    display = pipeline.create(dai.node.Sync)
    display.setRunOnHost(True)
    display.setSyncThreshold(syncThreshold)
    stitching.out.link(display.inputs["image"])
    merged.out.link(display.inputs["detections"])
    panoramaWindow = "Multi-device detections panorama"
    displayQueues.append((panoramaWindow, display.out.createOutputQueue(maxSize=1, blocking=False)))
    previewWidth, previewHeight, panoramaWidth, panoramaHeight, headerHeight = 640, 400, 1280, 800, 60
    for viewName, _ in displayQueues:
        cv2.namedWindow(viewName, cv2.WINDOW_NORMAL)
        size = (panoramaWidth, panoramaHeight + headerHeight) if viewName == panoramaWindow else (previewWidth, previewHeight)
        cv2.resizeWindow(viewName, *size)
    pipeline.start()
    while pipeline.isRunning():
        for viewName, queue in displayQueues:
            group = queue.tryGet()
            if group is None:
                continue
            image = group["image"]
            frame = image.getCvFrame()
            detections = group["detections"]
            isPanorama = viewName == panoramaWindow
            if not isPanorama:
                detections = detections.transformTo(image.getTransformation())
                frame = cv2.resize(frame, (previewWidth, previewHeight), interpolation=cv2.INTER_AREA)
            else:
                scale = min(panoramaWidth / frame.shape[1], panoramaHeight / frame.shape[0])
                frame = cv2.resize(frame, (round(frame.shape[1] * scale), round(frame.shape[0] * scale)), interpolation=cv2.INTER_AREA)
            for detection in detections.detections:
                box = detection.getBoundingBox().denormalize(frame.shape[1], frame.shape[0])
                points = box.getPoints()
                for a, b in zip(points, points[1:] + points[:1]):
                    cv2.line(frame, (round(a.x), round(a.y)), (round(b.x), round(b.y)), (0, 255, 0), 2)
                cv2.putText(frame, f"{detection.labelName or detection.label}: {detection.confidence:.2f}",
                            (round(box.center.x), round(box.center.y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if isPanorama:
                frame = cv2.copyMakeBorder(frame, headerHeight, 0, 0, 0, cv2.BORDER_CONSTANT, value=(0, 0, 0))
                cv2.putText(frame, f"Duplicates: {overlapModes[modeIndex].name} | IoU: {iouThreshold:.2f}",
                            (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(frame, "1: Off | 2: NMS | 3: Average | W/S: IoU +/-0.05 | Q: Quit",
                            (10, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(viewName, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q")):
            break
        if key in (ord("1"), ord("2"), ord("3")):
            modeIndex = key - ord("1")
        elif key in (ord("w"), ord("W"), ord("s"), ord("S")):
            step = 0.05 if key in (ord("w"), ord("W")) else -0.05
            iouThreshold = min(1.0, max(0.0, round(iouThreshold + step, 2)))
        else:
            continue
        # Send a fresh, complete config; queued messages may still be in use by the filter.
        config = dai.ImgDetectionsFilterConfig()
        config.setConfidenceRange(0.5)
        config.overlapMode = overlapModes[modeIndex]
        config.overlapIouThreshold = iouThreshold
        configQueue.send(config)
    cv2.destroyAllWindows()
