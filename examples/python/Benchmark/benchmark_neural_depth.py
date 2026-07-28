#!/usr/bin/env python3
"""Benchmark one or more NeuralDepth models running in a single pipeline.

Measures FPS, end-to-end latency and pixel throughput of every NeuralDepth node
with on-device BenchmarkIn nodes, so only the small report messages are sent to
the host and the measurement is not limited by the host link.

Frame routing:
  parallel   - every node receives every stereo pair (models share the NPU)
  sequential - a Script node round-robins stereo pairs between the nodes, so
               only one model has work queued at a time

Examples:
  python benchmark_neural_depth.py --models L
  python benchmark_neural_depth.py --models S,N --mode parallel
  python benchmark_neural_depth.py --models M,S --mode sequential

Requires a device with NeuralDepth support (RVC4, LuxonisOS 1.20.4+).
"""

import argparse
import json
import time

import depthai as dai

MODELS = {
    "XL": (dai.DeviceModelZoo.NEURAL_DEPTH_EXTRA_LARGE, 1248, 780),
    "L": (dai.DeviceModelZoo.NEURAL_DEPTH_LARGE, 768, 480),
    "M": (dai.DeviceModelZoo.NEURAL_DEPTH_MEDIUM, 576, 360),
    "S": (dai.DeviceModelZoo.NEURAL_DEPTH_SMALL, 480, 300),
    "N": (dai.DeviceModelZoo.NEURAL_DEPTH_NANO, 384, 240),
    "N2": (dai.DeviceModelZoo.NEURAL_DEPTH_288X180, 288, 180),
    "N3": (dai.DeviceModelZoo.NEURAL_DEPTH_192X120, 192, 120),
}


def buildDispatcher(pipeline, leftOutput, rightOutput, count):
    """Script node that hands each stereo pair to one node in round-robin order."""
    script = pipeline.create(dai.node.Script)
    leftOutput.link(script.inputs["left"])
    rightOutput.link(script.inputs["right"])
    for name in ("left", "right"):
        script.inputs[name].setBlocking(False)
        script.inputs[name].setMaxSize(2)
    script.setScript(
        f"""
n = {count}
i = 0
while True:
    left = node.inputs['left'].get()
    right = node.inputs['right'].get()
    node.outputs['left' + str(i)].send(left)
    node.outputs['right' + str(i)].send(right)
    i = (i + 1) % n
"""
    )
    return script


def run(models, mode, fps, duration, reportEvery, warmupReports, dropFrames, deviceId):
    device = dai.Device(dai.DeviceInfo(deviceId)) if deviceId else dai.Device()
    with dai.Pipeline(device) as pipeline:
        leftCamera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        rightCamera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        leftOutput = leftCamera.requestFullResolutionOutput(fps=fps)
        rightOutput = rightCamera.requestFullResolutionOutput(fps=fps)

        dispatcher = buildDispatcher(pipeline, leftOutput, rightOutput, len(models)) if mode == "sequential" else None

        nodes = []
        for i, key in enumerate(models):
            model, width, height = MODELS[key]
            neuralDepth = pipeline.create(dai.node.NeuralDepth)
            if dispatcher is not None:
                neuralDepth.build(dispatcher.outputs[f"left{i}"], dispatcher.outputs[f"right{i}"], model)
            else:
                neuralDepth.build(leftOutput, rightOutput, model)
            if dropFrames:
                # a saturated model must not backpressure the camera or the other models
                for inp in (neuralDepth.left, neuralDepth.right):
                    inp.setBlocking(False)
                    inp.setMaxSize(2)

            benchmark = pipeline.create(dai.node.BenchmarkIn)
            benchmark.sendReportEveryNMessages(reportEvery)
            benchmark.logReportsAsWarnings(False)
            neuralDepth.depth.link(benchmark.input)
            nodes.append((f"{key}#{i}", key, width, height, benchmark.report.createOutputQueue(8, False)))

        pipeline.start()

        # models are compiled/loaded on the first frames, measure only once every node reports
        warmedUp = {name: 0 for name, *_ in nodes}
        deadline = time.monotonic() + 120
        while min(warmedUp.values()) < warmupReports and time.monotonic() < deadline:
            for name, _key, _w, _h, queue in nodes:
                while queue.tryGet() is not None:
                    warmedUp[name] += 1
            time.sleep(0.05)
        if min(warmedUp.values()) < warmupReports:
            raise RuntimeError(f"warmup timed out, reports received: {warmedUp}")

        totals = {name: {"messages": 0.0, "time": 0.0, "latency": 0.0, "reports": 0} for name, *_ in nodes}
        end = time.monotonic() + duration
        while time.monotonic() < end:
            for name, _key, _w, _h, queue in nodes:
                while True:
                    report = queue.tryGet()
                    if report is None:
                        break
                    total = totals[name]
                    total["messages"] += report.numMessagesReceived
                    total["time"] += report.timeTotal
                    total["latency"] += report.averageLatency * report.numMessagesReceived
                    total["reports"] += 1
            time.sleep(0.05)
        pipeline.stop()

    perNode = []
    totalPixelsPerSecond = 0.0
    for name, key, width, height, _queue in nodes:
        total = totals[name]
        if total["reports"] == 0 or total["time"] == 0:
            perNode.append({"node": name, "model": key, "error": "no reports in measurement window"})
            continue
        nodeFps = total["messages"] / total["time"]
        pixelsPerSecond = nodeFps * width * height
        totalPixelsPerSecond += pixelsPerSecond
        perNode.append(
            {
                "node": name,
                "model": key,
                "resolution": f"{width}x{height}",
                "fps": round(nodeFps, 2),
                "latencyMs": round(1000.0 * total["latency"] / total["messages"], 1),
                "mpixelsPerSecond": round(pixelsPerSecond / 1e6, 2),
                "reports": total["reports"],
            }
        )
    return {
        "mode": mode,
        "models": list(models),
        "cameraFps": fps,
        "totalMpixelsPerSecond": round(totalPixelsPerSecond / 1e6, 2),
        "totalFps": round(sum(node.get("fps", 0.0) for node in perNode), 2),
        "nodes": perNode,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--models", default="S", help=f"comma separated, repeats allowed, one of {sorted(MODELS)}")
    parser.add_argument("--mode", choices=["parallel", "sequential"], default="parallel")
    parser.add_argument("--fps", type=float, default=60.0, help="camera FPS")
    parser.add_argument("--duration", type=float, default=15.0, help="measurement window in seconds")
    parser.add_argument("--report-every", type=int, default=15, help="messages per BenchmarkIn report")
    parser.add_argument("--warmup-reports", type=int, default=2, help="reports discarded per node before measuring")
    parser.add_argument("--keep-frames", action="store_true", help="use blocking node inputs instead of dropping frames")
    parser.add_argument("--device", help="device id / mxid, defaults to the first device found")
    parser.add_argument("--out", help="append the result as a JSON line to this file")
    args = parser.parse_args()

    models = [model.strip().upper() for model in args.models.split(",") if model.strip()]
    for model in models:
        if model not in MODELS:
            parser.error(f"unknown model {model}, known models: {sorted(MODELS)}")

    result = run(
        models, args.mode, args.fps, args.duration, args.report_every, args.warmup_reports, not args.keep_frames, args.device
    )
    print(json.dumps(result), flush=True)
    if args.out:
        with open(args.out, "a") as handle:
            handle.write(json.dumps(result) + "\n")


if __name__ == "__main__":
    main()
