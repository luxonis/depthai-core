#!/usr/bin/env python3
"""RVC4 ToF example exercising the newly exposed VD55H1 IPP post-processing settings.

The following ToFConfig fields are only honored on RVC4 (VD55H1) devices and map
directly onto STMicroelectronics IPP controls. Each is optional -- leaving one unset
keeps the IPP/wrapper default:

  Bilateral spatial filter:
    enableBilateralFilter, bilateralStdFactor, bilateralKernelSize
  Temporal noise reduction (TNR):
    enableTemporalNoiseReduction, tnrMaxGain, tnrStdFactor
  Flying-pixel filter:
    enableFlyingPixelFilter, flyingPixelDepthThreshold, flyingPixelMinDepthOccurrence
  Illumination pipe type:
    pipeType (AUTO / FLOOD / DOT)

This script builds a ToF pipeline, applies the exposed settings via the initial
config, streams the depth, amplitude, intensity and confidence outputs, and
(in --test mode) verifies that all four streams produce data with the settings
applied -- confirming the firmware accepts them.

Requires a firmware package that implements these controls. Point to it with either
the --fwp flag or the DEPTHAI_DEVICE_RVC4_FWP environment variable, e.g.:

    export DEPTHAI_DEVICE_RVC4_FWP=/path/to/depthai-device-rvc4-fwp.tar.xz
    python tof_rvc4_exposed_settings.py --ip 10.11.0.86

Usage:
    python tof_rvc4_exposed_settings.py --ip 10.11.0.86              # live display
    python tof_rvc4_exposed_settings.py --ip 10.11.0.86 --test      # headless verify
"""

import argparse
import sys

import cv2
import numpy as np

import depthai as dai


def colorizeDepth(frame: np.ndarray, minDepth: float, maxDepth: float) -> np.ndarray:
    invalidMask = frame == 0
    try:
        logDepth = np.log(frame.astype(np.float32) + 1e-6)
        logDepth[invalidMask] = 0.0
        logMin, logMax = np.log(minDepth + 1e-6), np.log(maxDepth + 1e-6)
        logDepth = np.clip(logDepth, logMin, logMax)
        # Map from the FIXED depth range (not the per-frame min/max) so a given depth
        # always maps to the same color -- otherwise the mapping shifts every frame and
        # the image flickers.
        colored = np.interp(logDepth, (logMin, logMax), (0, 255))
        colored = colored.astype(np.uint8)
        colored = cv2.applyColorMap(colored, cv2.COLORMAP_JET)
        colored[invalidMask] = 0
    except (IndexError, ValueError):
        colored = np.zeros((*frame.shape, 3), dtype=np.uint8)
    return colored


def normalizeFrame(frame: np.ndarray) -> np.ndarray:
    return cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)


# Live-tuning trackbar spec. Each tuple: (slider label, config attribute, max slider
# value, scale). The config value sent to the device is `slider_position * scale`
# (cast back to int for integer-typed fields).
#
# Ranges/defaults below follow the VD55H1 IPP filter reference (depthai-device-kb,
# TOF_IPP_FILTERS.md), which documents the underlying wrapper controls these fields
# map onto. depthai-core forwards each field to the firmware unchanged (no unit
# conversion), so the values here are exactly the IPP control values.
TUNE_PARAMS = [
    ("bilat: enable",         "enableBilateralFilter",           1,   1.0),
    # NR stdFactor: photometric sigma multiplier. IPP default 4.0; higher -> more
    # smoothing. Slider covers 0.1..8.0 (step 0.1) so the default sits mid-range.
    ("bilat: stdFactor x10",  "bilateralStdFactor",              80,  0.1),
    # NR kernelSize: IPP valid range {3,15} in ODD steps only (step 2); default 5.
    # Even/zero values are invalid, so this slider is snapped to odd via ODD_ATTRS.
    ("bilat: kernelSize",     "bilateralKernelSize",             15,  1.0),
    ("tnr: enable",           "enableTemporalNoiseReduction",    1,   1.0),
    # TNR maxGain: max temporal integration (frames blended). IPP default 16; a gain
    # of 0 is meaningless so the slider is clamped to >= 1 (see MIN_SLIDER).
    ("tnr: maxGain",          "tnrMaxGain",                      16,  1.0),
    # TNR stdFactor: motion-detection sensitivity. IPP default 1.4 (mode variants 1.2).
    # Slider covers 0.05..5.0 (step 0.05).
    ("tnr: stdFactor x20",    "tnrStdFactor",                    100, 0.05),
    ("fly: enable",           "enableFlyingPixelFilter",         1,   1.0),
    # FPC depthTh: neighbour "same-surface" depth threshold in MILLIMETRES. IPP
    # default 100 mm (per-mode overrides span ~30..230 mm). Slider covers 0..300 mm.
    ("fly: depthThr (mm)",    "flyingPixelDepthThreshold",       300, 1.0),
    # FPC minDepthOccurence: min neighbours within depthTh (out of the 5x5=25 window)
    # required to keep a pixel. IPP default 23; fewer -> more aggressive removal.
    ("fly: minOccurrence",    "flyingPixelMinDepthOccurrence",   25,  1.0),
    # phaseUnwrapErrorThreshold is a plain uint16 (not std::optional), default 100,
    # and applyConfig writes it unconditionally -- so every runtime config we send
    # overwrites it. Exposing it as a slider keeps that value explicit and tunable.
    # (IPP treats a very large value, ~10000, as effectively disabled.)
    ("phaseUnwrapErrThr",     "phaseUnwrapErrorThreshold",       500, 1.0),
    # pipeType is a 3-value enum, rendered as a cycling button (see ENUM_ATTRS) rather
    # than a slider; maxval here is just len(choices) - 1.
    ("pipe: type",            "pipeType",                        2,   1.0),
]

# Config attributes that are integer-typed and must not be assigned a float.
INT_ATTRS = {"bilateralKernelSize", "tnrMaxGain", "phaseUnwrapErrorThreshold"}
BOOL_ATTRS = {"enableBilateralFilter", "enableTemporalNoiseReduction", "enableFlyingPixelFilter"}
# Enum-typed attributes: attr -> ordered tuple of dai.ToFConfig.<Enum> member names.
# The slider position is the index into this tuple.
ENUM_ATTRS = {"pipeType": ("AUTO", "FLOOD", "DOT")}
# Attributes whose IPP control only accepts ODD values (rounded up to the next odd).
ODD_ATTRS = {"bilateralKernelSize"}
# Minimum allowed slider position per attribute:
#   bilateralStdFactor -> IPP rejects a zero std factor (1 -> 0.1 after scaling).
#   bilateralKernelSize -> IPP valid range starts at 3 (odd).
#   tnrMaxGain -> a gain of 0 is meaningless.
MIN_SLIDER = {"bilateralStdFactor": 1, "bilateralKernelSize": 3, "tnrMaxGain": 1}


def _snap(attr: str, pos: int, maxval: int) -> int:
    """Clamp a slider position to [MIN_SLIDER, maxval] and enforce odd-only attrs."""
    pos = max(MIN_SLIDER.get(attr, 0), min(maxval, pos))
    if attr in ODD_ATTRS and pos % 2 == 0:
        pos = min(maxval, pos + 1)  # round up to the next odd value
    return pos


# Custom slider panel drawn entirely with cv2 primitives. OpenCV's native
# createTrackbar labels do not render on every highgui backend (notably Qt6, and
# inconsistently on others), so instead of fighting that we draw our own sliders --
# label, track and handle -- as an image and drive them with a mouse callback. This
# renders identically on any backend.
PANEL_W = 660
ROW_H = 42
HEADER_H = 40
LABEL_W = 250
TRACK_X0 = LABEL_W + 10
TRACK_X1 = PANEL_W - 95
BTN_W = 90       # toggle button width (bool rows)
BTN_H = 26       # toggle button height


def panel_height() -> int:
    return HEADER_H + ROW_H * len(TUNE_PARAMS)


def _button_rect(y: int) -> tuple:
    """Toggle-button rectangle (x0, y0, x1, y1) centered vertically on row `y`."""
    return TRACK_X0, y - BTN_H // 2, TRACK_X0 + BTN_W, y + BTN_H // 2


def _pos_to_x(pos: int, maxval: int) -> int:
    return int(TRACK_X0 + (pos / maxval) * (TRACK_X1 - TRACK_X0))


def _x_to_pos(x: int, maxval: int, minpos: int) -> int:
    frac = (x - TRACK_X0) / (TRACK_X1 - TRACK_X0)
    frac = min(1.0, max(0.0, frac))
    return max(minpos, min(maxval, int(round(frac * maxval))))


def _row_center_y(row: int) -> int:
    return HEADER_H + ROW_H * row + ROW_H // 2


def initial_positions(cfg: dai.ToFConfig) -> list:
    """Slider positions (ints) initialized from a ToFConfig."""
    positions = []
    for label, attr, maxval, scale in TUNE_PARAMS:
        init = getattr(cfg, attr)
        if attr in ENUM_ATTRS:
            choices = ENUM_ATTRS[attr]
            pos = choices.index(init.name) if init is not None else 0
        else:
            pos = int(round((bool(init) if attr in BOOL_ATTRS else (init or 0)) / scale))
        positions.append(_snap(attr, pos, maxval))
    return positions


def panel_mouse(event, x, y, flags, state) -> None:
    """Mouse callback: bool/enum rows toggle or cycle on click; slider rows follow drag.

    The row is latched in state["drag_row"] on mouse-down and reused for every
    subsequent move until mouse-up, instead of being recomputed from the current y
    each call -- otherwise a fast drag that drifts vertically hands control to
    whichever row the cursor happens to pass over.
    """
    if event == cv2.EVENT_LBUTTONUP:
        state["drag_row"] = None
        return

    if event == cv2.EVENT_LBUTTONDOWN:
        row = (y - HEADER_H) // ROW_H
        if not (0 <= row < len(TUNE_PARAMS)):
            return
        state["drag_row"] = row
    elif event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON:
        row = state.get("drag_row")
        if row is None:
            return
    else:
        return

    _label, attr, maxval, _scale = TUNE_PARAMS[row]

    if attr in BOOL_ATTRS:
        # Toggle only on the initial click (not on drag), so it doesn't flip repeatedly.
        if event == cv2.EVENT_LBUTTONDOWN:
            bx0, _by0, bx1, _by1 = _button_rect(_row_center_y(row))
            if bx0 <= x <= bx1:
                state["pos"][row] = 0 if state["pos"][row] else 1
        return

    if attr in ENUM_ATTRS:
        # Cycle through the enum's choices only on the initial click.
        if event == cv2.EVENT_LBUTTONDOWN:
            bx0, _by0, bx1, _by1 = _button_rect(_row_center_y(row))
            if bx0 <= x <= bx1:
                state["pos"][row] = (state["pos"][row] + 1) % len(ENUM_ATTRS[attr])
        return

    state["pos"][row] = _snap(attr, _x_to_pos(x, maxval, MIN_SLIDER.get(attr, 0)), maxval)


def config_from_positions(positions: list) -> dai.ToFConfig:
    """Build a ToFConfig from the current slider positions."""
    cfg = dai.ToFConfig()
    for i, (label, attr, _maxval, scale) in enumerate(TUNE_PARAMS):
        if attr in ENUM_ATTRS:
            value = getattr(dai.ToFConfig.PipeType, ENUM_ATTRS[attr][positions[i]])
        else:
            raw = positions[i] * scale
            if attr in BOOL_ATTRS:
                value = bool(raw)
            elif attr in INT_ATTRS:
                value = int(round(raw))
            else:
                value = float(raw)
        setattr(cfg, attr, value)
    return cfg


def render_panel(positions: list) -> np.ndarray:
    """Draw the slider panel: each row shows label, track, handle and live value."""
    img = np.full((panel_height(), PANEL_W, 3), 40, dtype=np.uint8)
    cv2.putText(img, "Live tuning -- drag sliders, click toggles", (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    for i, (label, attr, maxval, scale) in enumerate(TUNE_PARAMS):
        y = _row_center_y(i)
        cv2.putText(img, label, (10, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (220, 220, 220), 1, cv2.LINE_AA)

        if attr in BOOL_ATTRS:
            # Render a toggle button: green "ON" / grey "OFF".
            on = bool(positions[i])
            bx0, by0, bx1, by1 = _button_rect(y)
            cv2.rectangle(img, (bx0, by0), (bx1, by1),
                          (70, 160, 70) if on else (70, 70, 70), -1)
            cv2.rectangle(img, (bx0, by0), (bx1, by1), (200, 200, 200), 1)
            text = "ON" if on else "OFF"
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.putText(img, text, (bx0 + (BTN_W - tw) // 2, y + th // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
            continue

        if attr in ENUM_ATTRS:
            # Render a cycling button showing the current enum choice; click advances it.
            text = ENUM_ATTRS[attr][positions[i]]
            bx0, by0, bx1, by1 = _button_rect(y)
            cv2.rectangle(img, (bx0, by0), (bx1, by1), (170, 110, 60), -1)  # BGR: blue
            cv2.rectangle(img, (bx0, by0), (bx1, by1), (200, 200, 200), 1)
            (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.putText(img, text, (bx0 + (BTN_W - tw) // 2, y + th // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            continue

        cv2.line(img, (TRACK_X0, y), (TRACK_X1, y), (90, 90, 90), 2, cv2.LINE_AA)
        hx = _pos_to_x(positions[i], maxval)
        cv2.circle(img, (hx, y), 7, (120, 220, 120), -1, cv2.LINE_AA)
        raw = positions[i] * scale
        shown = int(round(raw)) if attr in INT_ATTRS else f"{raw:.3g}"
        cv2.putText(img, str(shown), (TRACK_X1 + 8, y + 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (120, 220, 120), 1, cv2.LINE_AA)
    return img


def draw_depth_hover(display: np.ndarray, depth_raw: np.ndarray, mouse: dict) -> None:
    """Overlay the depth value (mm) under the mouse cursor onto the depth display."""
    x, y = mouse["x"], mouse["y"]
    h, w = depth_raw.shape[:2]
    if not (0 <= x < w and 0 <= y < h):
        return
    d = int(depth_raw[y, x])
    text = f"({x},{y})  {d} mm" if d > 0 else f"({x},{y})  no depth"
    cv2.drawMarker(display, (x, y), (255, 255, 255), cv2.MARKER_CROSS, 12, 1, cv2.LINE_AA)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
    cv2.rectangle(display, (5, 5), (15 + tw, 15 + th), (0, 0, 0), -1)
    cv2.putText(display, text, (10, 10 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)


def parse_args():
    p = argparse.ArgumentParser(description="RVC4 ToF exposed IPP settings example/test")
    p.add_argument("--ip", default=None, help="Device IP address (e.g. 10.11.0.86)")
    p.add_argument("--socket", default="AUTO", help="ToF board socket (default: AUTO)")
    p.add_argument("--test", action="store_true",
                   help="Headless verification: capture N frames, check all streams produce data, then exit")
    p.add_argument("--frames", type=int, default=20,
                   help="Frames to check in --test mode (default: 20)")
    return p.parse_args()


def build_exposed_config() -> dai.ToFConfig:
    """A ToFConfig with all newly exposed RVC4 IPP post-processing controls set."""
    cfg = dai.ToFConfig()

    # Values below are the VD55H1 IPP defaults (see TOF_IPP_FILTERS.md); tweak to taste.

    # Bilateral spatial filter -- stdFactor default 4.0, kernelSize odd in {3..15}.
    cfg.enableBilateralFilter = True
    cfg.bilateralStdFactor = 4.0
    cfg.bilateralKernelSize = 5

    # Temporal noise reduction (TNR) -- maxGain default 16, stdFactor default 1.4.
    cfg.enableTemporalNoiseReduction = True
    cfg.tnrMaxGain = 16
    cfg.tnrStdFactor = 1.4

    # Flying-pixel filter -- depthThreshold is in MILLIMETRES (IPP default 100 mm,
    # per-mode ~30..230 mm); minDepthOccurrence default 23 (of the 5x5=25 window).
    cfg.enableFlyingPixelFilter = True
    cfg.flyingPixelDepthThreshold = 100.0
    cfg.flyingPixelMinDepthOccurrence = 23.0

    # Phase-unwrap error threshold. Plain uint16 (default 100), always applied -- set
    # it explicitly so runtime configs don't silently reset it.
    cfg.phaseUnwrapErrorThreshold = 100

    # Illumination pipe type. Optional -- unset keeps the IPP default (AUTO); set
    # explicitly here for demonstration.
    cfg.pipeType = dai.ToFConfig.PipeType.AUTO

    return cfg


def describe(cfg: dai.ToFConfig) -> str:
    return (
        "  bilateral:   enable={} stdFactor={} kernelSize={}\n"
        "  TNR:         enable={} maxGain={} stdFactor={}\n"
        "  flyingPixel: enable={} depthThreshold={} minDepthOccurrence={}\n"
        "  phaseUnwrapErrorThreshold={}\n"
        "  pipeType={}".format(
            cfg.enableBilateralFilter, cfg.bilateralStdFactor, cfg.bilateralKernelSize,
            cfg.enableTemporalNoiseReduction, cfg.tnrMaxGain, cfg.tnrStdFactor,
            cfg.enableFlyingPixelFilter, cfg.flyingPixelDepthThreshold,
            cfg.flyingPixelMinDepthOccurrence, cfg.phaseUnwrapErrorThreshold,
            cfg.pipeType,
        )
    )


def main() -> int:
    args = parse_args()

    minDepth, maxDepth = 100, 7000
    socket = getattr(dai.CameraBoardSocket, args.socket)
    device = dai.Device(dai.DeviceInfo(args.ip)) if args.ip else None

    pipeline = dai.Pipeline(device) if device is not None else dai.Pipeline()

    tof = pipeline.create(dai.node.ToF)
    # The mid-range profile enum moved across depthai versions; support both.
    if hasattr(dai.ToFConfig, "Profile"):
        tof.build(boardSocket=socket, profile=dai.ToFConfig.Profile.MID_RANGE)
    else:
        tof.build(boardSocket=socket, presetMode=dai.ImageFiltersPresetMode.TOF_MID_RANGE)

    cfg = build_exposed_config()
    tof.setInitialConfig(cfg)
    print("Applying exposed RVC4 ToF settings via initial config:")
    print(describe(cfg))

    inputConfigQueue = tof.tofBaseInputConfig.createInputQueue()
    outputQueues = {
        "depth": tof.depth.createOutputQueue(maxSize=4, blocking=False),
        "amplitude": tof.amplitude.createOutputQueue(maxSize=4, blocking=False),
        "intensity": tof.intensity.createOutputQueue(maxSize=4, blocking=False),
        "confidence": tof.confidence.createOutputQueue(maxSize=4, blocking=False),
    }

    with pipeline as p:
        dev = p.getDefaultDevice()
        if dev.getPlatform() != dai.Platform.RVC4:
            print(f"ERROR: this example targets RVC4 devices, got {dev.getPlatform()}", file=sys.stderr)
            return 2

        p.start()
        # Also exercise the runtime path: re-send the same config after start.
        inputConfigQueue.send(cfg)

        if args.test:
            # Count frames per stream in which at least one non-zero pixel was produced.
            got = {name: 0 for name in outputQueues}
            for i in range(args.frames):
                line = [f"frame {i:2d}:"]
                for name, queue in outputQueues.items():
                    frame = queue.get().getCvFrame()
                    nonzero = int(np.count_nonzero(frame))
                    if nonzero > 0:
                        got[name] += 1
                    pct = 100.0 * nonzero / frame.size if frame.size else 0.0
                    line.append(f"{name}={pct:5.1f}%")
                print("  ".join(line))

            print()
            all_ok = True
            for name, count in got.items():
                ok = count >= max(1, args.frames // 2)
                all_ok = all_ok and ok
                print(f"  {name:10s}: {count}/{args.frames} frames with data  [{'OK' if ok else 'FAIL'}]")
            print("\nRESULT: PASS -- exposed settings accepted, all streams produced." if all_ok
                  else "\nRESULT: FAIL -- one or more streams had too little data.")
            return 0 if all_ok else 1

        # Live tuning: trackbars live in their own window so their labels are readable
        # and the image windows stay undistorted. Moving any slider re-sends an updated
        # ToFConfig to the running node via tofBaseInputConfig.
        # Live tuning via a self-drawn slider panel (see render_panel/panel_mouse),
        # so the labels render on any OpenCV backend. Dragging a slider re-sends an
        # updated ToFConfig to the running node via tofBaseInputConfig.
        tune_window = "tuning"
        cv2.namedWindow(tune_window, cv2.WINDOW_AUTOSIZE)
        panel_state = {"pos": initial_positions(cfg), "drag_row": None}
        cv2.setMouseCallback(tune_window, panel_mouse, panel_state)
        last_sent = None

        # Track the cursor over the depth window so we can show the depth value there.
        cv2.namedWindow("depth")
        mouse = {"x": -1, "y": -1}
        cv2.setMouseCallback("depth", lambda e, x, y, f, m: m.update(x=x, y=y), mouse)

        latest = {}  # name -> most recent raw cv frame

        print("Streaming depth, amplitude, intensity, confidence.")
        print("In the 'tuning' window: drag sliders and click the ON/OFF buttons to tune live.")
        print("Hover over the 'depth' window to read the depth (mm) under the cursor. "
              "Press 'q' to quit.")
        while p.isRunning():
            # Push a new config only when the sliders actually changed.
            current = config_from_positions(panel_state["pos"])
            key = tuple(getattr(current, attr) for _, attr, _, _ in TUNE_PARAMS)
            if key != last_sent:
                inputConfigQueue.send(current)
                last_sent = key
                print("Applied:", " ".join(f"{a}={getattr(current, a)}" for _, a, _, _ in TUNE_PARAMS))
            cv2.imshow(tune_window, render_panel(panel_state["pos"]))

            for name, queue in outputQueues.items():
                frame = queue.tryGet()
                if frame is not None:
                    latest[name] = frame.getCvFrame()

            # Redraw depth every iteration (even without a new frame) so the hover
            # readout follows the cursor smoothly.
            if "depth" in latest:
                depth_display = colorizeDepth(latest["depth"], minDepth, maxDepth)
                draw_depth_hover(depth_display, latest["depth"], mouse)
                cv2.imshow("depth", depth_display)
            for name in ("amplitude", "intensity", "confidence"):
                if name in latest:
                    cv2.imshow(name, normalizeFrame(latest[name]))

            if cv2.waitKey(1) == ord("q"):
                break

    return 0


if __name__ == "__main__":
    sys.exit(main())
