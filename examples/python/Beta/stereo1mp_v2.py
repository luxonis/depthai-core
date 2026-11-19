#!/usr/bin/env python3
"""Stereo-1MP v2 neural stereo on an OAK4 (RVC4): stereo pair -> rectification -> L/R Sync -> NeuralNetwork -> compose -> depth + confidence.

    python stereo1mp_v2.py [--device 10.11.0.182] [--seconds 60] [--show]           # network shipped in the firmware
    python stereo1mp_v2.py --nna Stereo-1MP-v2_TS10q28c1_1280x800.rvc4.tar.xz ...   # or from an NNArchive

The network runs on the NPU (int8) with the R26 correlation op package (shipped in the firmware next to the executable).
By default the firmware's copy is used (NeuralNetwork backend property "firmware_model" = STEREO1MP_V2_1280x800.dlc), so no
model file is needed; --nna loads the same network from its NNArchive instead (identical DLC, md5 79b395e1).

L/R pairing: Rectification and NeuralNetwork pair their two inputs by arrival order only. The left and right camera counters can
start offset, so without a Sync the network can see L and R 1-2 frames (33-67 ms) apart for the whole run (measured on an OAK4-Lite:
100 % of pairs 33 ms apart; with Sync 0 us). The pipeline therefore pairs the rectified frames by device timestamp (dai.node.Sync +
MessageDemux, as StereoDepth does) before the network. --no-sync restores the old order pairing (for comparison only).

Compose (1/4-res disparity x4 bilinear + clamped 1/2-res residuals, then Z = fx*B/d) runs in an on-device Script node by default
(--compose device): only depth (u16 mm, 1280x800) and confidence (u8, 640x400, conf = value/256) cross the link.
--compose host streams the raw tensors (~2.9 MB / frame) and composes on the host (NumPy + OpenCV).
Recommended: --conf-thr 0.25 (default), performance profile sustained_high_performance (default).
Working range: Z >= fx*B / 192 px (~0.33-0.37 m) to ~15 m (network floor 4.3 px; smaller disparities are invalid).
"""
import argparse
import datetime
import sys
import time

import numpy as np
import depthai as dai

ap = argparse.ArgumentParser()
ap.add_argument("--nna", default="", help="NNArchive (.rvc4.tar.xz); default: the network shipped in the firmware")
ap.add_argument("--device", default="", help="device IP / id (default: first found)")
ap.add_argument("--fps", type=float, default=30.0)
ap.add_argument("--profile", default="sustained_high_performance", help="SNPE perf profile: sustained_high_performance / default / burst")
ap.add_argument("--compose", default="device", choices=["device", "host"])
ap.add_argument("--conf-thr", type=float, default=0.25, help="invalidate depth where conf (bilinear x2) < thr; 0 = off")
ap.add_argument("--sync", action=argparse.BooleanOptionalAction, default=True, help="pair L/R by device timestamp before the network (default on)")
ap.add_argument("--sync-ms", type=float, default=10.0, help="Sync threshold in ms")
ap.add_argument("--sync-queue", type=int, default=4, help="Sync input queue size (non-blocking); 1 halves the rate on some firmwares")
ap.add_argument("--seconds", type=float, default=0.0, help="stop after N s (0 = until q / Ctrl-C)")
ap.add_argument("--show", action="store_true", help="OpenCV window with the depth colour map")
a = ap.parse_args()

W, H = 1280, 800          # network canvas: rectified GRAY8 pair
DMIN_PX = 4.3             # far floor of the network (~15 m on the training rig): smaller disparities are invalid
DMAX_PX = 192.0           # 48 bins x 4 px: nearest measurable Z = fx*B / 192
FIRMWARE_MODEL = "STEREO1MP_V2_1280x800.dlc"
UDO = "libUdoR26PackageReg.so"
OUTPUTS = "disp_low,head9,conf"

COMPOSE = r"""
import numpy as np
import cv2
FXB, DMIN, THR = %f, %f, %f
W, H = 1280, 800
b = node.inputs["fxb"].get()          # fx*B of the rectified pair, sent once by the host
FXB = float(bytes(b.getData()).decode())
node.warn("compose fx*B %%.0f px*mm" %% FXB)
def tensor(m, name, dt):
    ti = m.getTensorInfo(name)
    n = 1
    for d in ti.dims:
        n *= d
    buf = np.asarray(m.getData(), dtype=np.uint8)
    return np.frombuffer(buf[ti.offset:ti.offset + n * np.dtype(dt).itemsize].tobytes(), dt), ti
while True:
    m = node.inputs["nn"].get()
    dl, ti = tensor(m, "disp_low", np.uint16)
    dl = ((dl.astype(np.float32) - np.float32(ti.qpZp)) * np.float32(ti.qpScale)).reshape(H // 4, W // 4)
    h9, tj = tensor(m, "head9", np.uint16)
    r = (h9.reshape(H // 2, W // 2, 5)[..., 1:5].astype(np.float32) - np.float32(tj.qpZp)) * np.float32(tj.qpScale)
    np.clip(r, -4.0, 4.0, out=r)
    d = cv2.resize(dl, (W, H), interpolation=cv2.INTER_LINEAR)
    d += r.reshape(H // 2, W // 2, 2, 2).transpose(0, 2, 1, 3).reshape(H, W)
    cf, _ = tensor(m, "conf", np.uint8)
    v = d >= np.float32(DMIN)
    if THR > 0:
        v &= cv2.resize(cf.reshape(H // 2, W // 2).astype(np.float32) * np.float32(1.0 / 256), (W, H), interpolation=cv2.INTER_LINEAR) >= np.float32(THR)
    z = np.zeros_like(d)
    np.divide(np.float32(FXB), d, out=z, where=v)
    z += 0.5
    depth = z.astype(np.uint16)
    img = ImgFrame(); img.setType(ImgFrame.Type.RAW16); img.setWidth(W); img.setHeight(H)
    img.setData(depth.view(np.uint8).reshape(-1)); img.setTimestamp(m.getTimestamp()); img.setSequenceNum(m.getSequenceNum())
    node.io["depth"].send(img)
    c = ImgFrame(); c.setType(ImgFrame.Type.RAW8); c.setWidth(W // 2); c.setHeight(H // 2)
    c.setData(cf); c.setTimestamp(m.getTimestamp()); c.setSequenceNum(m.getSequenceNum())
    node.io["conf"].send(c)
"""


def host_compose(nnd, fxb):
    """Same maths as the Script compose, on the host (raw tensors dequantised with their TensorInfo scale / zero point)."""
    import cv2
    out = {}
    for n in ("disp_low", "head9", "conf"):
        ti = [t for t in nnd.getAllLayers() if t.name == n][0]
        raw = np.asarray(nnd.getTensor(n, dequantize=False)).astype(np.float32)
        out[n] = (raw - np.float32(ti.qpZp)) * np.float32(ti.qpScale) if ti.quantization else raw
    dl = out["disp_low"].reshape(H // 4, W // 4)
    r = np.clip(out["head9"].reshape(H // 2, W // 2, 5)[..., 1:5], -4.0, 4.0)
    d = cv2.resize(dl, (W, H), interpolation=cv2.INTER_LINEAR) + r.reshape(H // 2, W // 2, 2, 2).transpose(0, 2, 1, 3).reshape(H, W)
    conf = cv2.resize(out["conf"].reshape(H // 2, W // 2), (W, H), interpolation=cv2.INTER_LINEAR)
    v = d >= np.float32(DMIN_PX)
    if a.conf_thr > 0:
        v &= conf >= np.float32(a.conf_thr)
    z = np.zeros_like(d)
    np.divide(np.float32(fxb), d, out=z, where=v)
    return (z + 0.5).astype(np.uint16), conf


device = dai.Device(dai.DeviceInfo(a.device)) if a.device else dai.Device()
with dai.Pipeline(device) as p:
    calib = device.readCalibration()
    fx = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, W, H)[0][0]
    base_mm = abs(calib.getBaselineDistance(dai.CameraBoardSocket.CAM_C, dai.CameraBoardSocket.CAM_B, False)) * 10.0
    print(f"device {device.getDeviceName()}, B {base_mm:.2f} mm, network {'NNArchive ' + a.nna if a.nna else 'firmware ' + FIRMWARE_MODEL}, "
          f"L/R sync {'ON' if a.sync else 'OFF'}")

    camL = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=a.fps)
    camR = p.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=a.fps)
    oL = camL.requestOutput((W, H), dai.ImgFrame.Type.GRAY8, dai.ImgResizeMode.STRETCH, a.fps, False)
    oR = camR.requestOutput((W, H), dai.ImgFrame.Type.GRAY8, dai.ImgResizeMode.STRETCH, a.fps, False)
    rect = p.create(dai.node.Rectification)
    rect.setBackend(dai.node.Rectification.Backend.HW)
    rect.setAlphaScaling(0.0)
    rect.setOutputSize(W, H)
    oL.link(rect.input1); oR.link(rect.input2)

    tap = p.create(dai.node.Script)      # forwards one rectified left frame: its transformation holds the rectified fx
    tap.setScript("f = node.inputs['l'].get()\nnode.io['out'].send(f)\nimport time\nwhile True:\n    time.sleep(1)\n")
    rect.output1.link(tap.inputs["l"]); tap.inputs["l"].setBlocking(False); tap.inputs["l"].setMaxSize(1)
    q_rect = tap.outputs["out"].createOutputQueue(maxSize=1, blocking=False)

    srcL, srcR = rect.output1, rect.output2
    if a.sync:
        sy = p.create(dai.node.Sync)
        sy.setRunOnHost(False)
        sy.setSyncThreshold(datetime.timedelta(milliseconds=a.sync_ms))
        sy.setTimestampSource(dai.node.Sync.TimestampSource.DEVICE)
        srcL.link(sy.inputs["left"]); srcR.link(sy.inputs["right"])
        for name in ("left", "right"):            # like StereoDepth: never block the shared rectification producer
            sy.inputs[name].setBlocking(False); sy.inputs[name].setMaxSize(a.sync_queue)
        dm = p.create(dai.node.MessageDemux)
        dm.setRunOnHost(False)
        sy.out.link(dm.input)
        srcL, srcR = dm.outputs["left"], dm.outputs["right"]

    nn = p.create(dai.node.NeuralNetwork)
    props = {"udo_package": UDO, "output_tensors": OUTPUTS, "performance_profile": a.profile}
    if a.nna:
        nn.setNNArchive(dai.NNArchive(a.nna))
    else:
        props["firmware_model"] = FIRMWARE_MODEL
    nn.setBackendProperties(props)
    for name, src in (("left", srcL), ("right", srcR)):
        src.link(nn.inputs[name]); nn.inputs[name].setBlocking(True); nn.inputs[name].setMaxSize(2)

    if a.compose == "device":
        sc = p.create(dai.node.Script)
        sc.setScript(COMPOSE % (fx * base_mm, DMIN_PX, a.conf_thr))
        q_fxb = sc.inputs["fxb"].createInputQueue()
        nn.out.link(sc.inputs["nn"]); sc.inputs["nn"].setBlocking(False); sc.inputs["nn"].setMaxSize(2)
        q_depth = sc.outputs["depth"].createOutputQueue(maxSize=4, blocking=False)
        q_conf = sc.outputs["conf"].createOutputQueue(maxSize=4, blocking=False)
    else:
        q_nn = nn.out.createOutputQueue(maxSize=4, blocking=False)

    device.setLogLevel(dai.LogLevel.WARN); device.setLogOutputLevel(dai.LogLevel.WARN)
    p.start()
    rf = q_rect.get()
    fx_r = float(rf.getTransformation().getIntrinsicMatrix()[0][0])   # rectified fx (not the raw calibration fx)
    fxb = fx_r * base_mm
    print(f"rectified fx {fx_r:.2f} px -> fx*B {fxb:.0f} px*mm, range {fxb / DMAX_PX / 1000:.2f} .. {fxb / DMIN_PX / 1000:.1f} m")
    if a.compose == "device":
        msg = dai.Buffer(); msg.setData(list(f"{fxb:.3f}".encode())); q_fxb.send(msg)
    t0 = time.time(); n = 0; lat = []
    while p.isRunning() and (a.seconds <= 0 or time.time() - t0 < a.seconds):
        if a.compose == "device":
            fr = q_depth.get()
            depth = fr.getFrame()                                    # u16 mm, 0 = invalid
            q_conf.tryGet()
        else:
            fr = q_nn.get()
            depth, _ = host_compose(fr, fxb)
        n += 1
        lat.append((dai.Clock.now() - fr.getTimestamp()).total_seconds() * 1000)
        if n % 150 == 0:
            el = time.time() - t0
            v = depth[depth > 0]
            print(f"{el:6.1f} s  {n / el:5.1f} fps  latency p50 {np.median(lat[-150:]):5.1f} ms  "
                  f"valid {100 * v.size / depth.size:4.1f} %  median Z {np.median(v) if v.size else 0:.0f} mm", flush=True)
        if a.show:
            import cv2
            vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=255.0 / 5000.0), cv2.COLORMAP_TURBO)
            vis[depth == 0] = 0
            cv2.imshow("Stereo-1MP v2 depth (0-5 m)", vis)
            if cv2.waitKey(1) == ord("q"):
                break
    el = time.time() - t0
    print(f"done: {n} frames in {el:.0f} s = {n / max(el, 1e-6):.2f} fps, latency p50 {np.median(lat) if lat else 0:.1f} ms")
    sys.exit(0 if n > 0 else 1)
