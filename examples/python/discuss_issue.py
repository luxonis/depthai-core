import time
import depthai as dai

YOLO_HUB_SLUG = "luxonis/yolov6-nano:r2-coco-512x288"
NN_W, NN_H = 512, 288
RGB_FPS = 15
MONO_FPS = 15
DURATION_SEC = 30
LOG_INTERVAL_SEC = 5.0


def main() -> None:
    print(f"depthai version: {dai.__version__}")
    print(f"NNArchive:       HubAI {YOLO_HUB_SLUG}")
    print(f"NN input:        {NN_W}x{NN_H}")

    desc = dai.NNModelDescription(model=YOLO_HUB_SLUG, platform="RVC2")
    archive = dai.NNArchive(dai.getModelFromZoo(desc))

    p = dai.Pipeline()
    p.__enter__()
    try:
        cam_rgb = p.create(dai.node.Camera).build(
            boardSocket=dai.CameraBoardSocket.CAM_A,
            sensorResolution=(1920, 1080),
            sensorFps=RGB_FPS,
        )
        rgb_prev = cam_rgb.requestOutput(
            size=(NN_W, NN_H),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.STRETCH,
            fps=RGB_FPS,
            enableUndistortion=True,
        )

        left = p.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B, sensorFps=MONO_FPS
        )
        right = p.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C, sensorFps=MONO_FPS
        )
        left_g = left.requestOutput(
            size=(640, 400), type=dai.ImgFrame.Type.GRAY8, fps=MONO_FPS
        )
        right_g = right.requestOutput(
            size=(640, 400), type=dai.ImgFrame.Type.GRAY8, fps=MONO_FPS
        )

        stereo = p.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.FAST_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        # stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        left_g.link(stereo.left)
        right_g.link(stereo.right)
        rgb_prev.link(stereo.inputAlignTo)

        sdn = p.create(dai.node.SpatialDetectionNetwork)
        sdn.setNNArchive(archive)
        sdn.setBoundingBoxScaleFactor(0.5)
        sdn.setDepthLowerThreshold(100)
        sdn.setDepthUpperThreshold(10000)
        rgb_prev.link(sdn.input)
        stereo.depth.link(sdn.inputDepth)

        q_dets = sdn.out.createOutputQueue(maxSize=4, blocking=False)
        q_pass = sdn.passthrough.createOutputQueue(maxSize=4, blocking=False)
        q_depth = stereo.depth.createOutputQueue(maxSize=4, blocking=False)

        p.start()
        print(f"\nPipeline running. Polling for {DURATION_SEC}s...\n")
        print(f"{'t (s)':>6}  {'sdn.out':>9}  {'sdn.passthrough':>17}  {'stereo.depth':>14}")

        n_dets = n_pass = n_depth = 0
        last_log = 0.0
        start = time.time()
        while time.time() - start < DURATION_SEC:
            while q_dets.has():
                _ = q_dets.tryGet()
                n_dets += 1
            while q_pass.has():
                _ = q_pass.tryGet()
                n_pass += 1
            while q_depth.has():
                _ = q_depth.tryGet()
                n_depth += 1

            elapsed = time.time() - start
            if elapsed - last_log >= LOG_INTERVAL_SEC:
                last_log = elapsed
                print(
                    f"{elapsed:>6.0f}  {n_dets:>9d}  {n_pass:>17d}  {n_depth:>14d}",
                    flush=True,
                )
            time.sleep(0.05)

        print("\n=== Summary ===")
        print(f"  depthai version:                  {dai.__version__}")
        print(f"  Duration:                         {DURATION_SEC}s")
        print(f"  sdn.out messages received:        {n_dets}")
        print(f"  sdn.passthrough messages received: {n_pass}")
        print(f"  stereo.depth messages received:   {n_depth}")
        if n_pass > 0 and n_depth > 0 and n_dets == 0:
            print(
                "\nREPRODUCED. Watch the device log above for:\n"
                "  [SpatialLocationCalculator(N)] [critical] ImgDetections "
                "transformationData is not aligned to depth frame transformation"
            )
        elif n_dets > 0:
            print(
                "\nHealthy: sdn.out emitting detections."
            )
        else:
            print(
                "\nUnexpected: passthrough or depth also silent. Check device "
                "connection and calibration."
            )
    finally:
        try:
            p.__exit__(None, None, None)
        except Exception:
            pass


if __name__ == "__main__":
    main()