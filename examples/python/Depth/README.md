# Focused depth examples (RVC4)

Use Python bindings built from this branch. Run from the repository root with
`depthai` and `numpy` available. For our local checkout:

```sh
cd /home/vincek/dev/depthai-core/.focused-depth-worktree
export PYTHONPATH="$PWD/build-focused-current/bindings/python"
source ../.venv/bin/activate
export DEPTHAI_DEVICE_NAME_LIST=192.168.88.61
```

Change the device address as needed, or omit that export to use device discovery.
Run one example at a time. Open the printed Visualizer URL; stop with **Ctrl+C**.

## Three examples

- `focused_depth_roi.py`: one fixed rectangle in rectified-left coordinates.
- `focused_depth_detection.py`: the largest detected bounding box, transformed into
  rectified-left coordinates.
- `focused_depth_budget.py`: N available regions per frame, largest first, using one
  selected neural model, or automatic budgeting across two or three models. Overlapping padded regions merge before selection, so N
  counts separate inference regions, not necessarily individual detections.

All three call `focused_depth.py`. Its `--mode roi|detector` chooses the ROI source;
`--depth-mode roi|hold|hybrid` independently chooses the depth behavior:

- `roi`: depth inside selected boxes; zero elsewhere. Missing detections give empty depth.
- `hold`: preserve the previous selection through X empty detection messages and compute
  fresh depth there. It does not track motion or handle a stalled detector.
- `hybrid`: full-image EVA depth, overwritten by neural depth inside selected boxes.
  Missing detections leave EVA depth; hybrid does not preserve previous ROIs.

Crops retain disparity padding, then resize to the chosen neural model. Output is
full-size rectified-left depth, not RGB-aligned depth. The color/detection preview
uses its own camera coordinates.

## Common commands

Fixed ROI, Medium model, 30 FPS requested:

```sh
python examples/python/Depth/focused_depth_roi.py --roi 0.35,0.30,0.65,0.70 --depth-model 576X360 --fps 30
```

Largest detected object, Medium model:

```sh
python examples/python/Depth/focused_depth_detection.py --depth-model 576X360 --confidence 0.25 --fps 30
```

Keep the largest object's ROI through two missed detection frames:

```sh
python examples/python/Depth/focused_depth_detection.py --depth-mode hold --hold-frames 2 --depth-model 576X360 --confidence 0.25 --fps 30
```

Full EVA depth at 640×480, enhanced inside the largest object's ROI:

```sh
python examples/python/Depth/focused_depth_detection.py --depth-mode hybrid --stereo-size 640 480 --depth-model 576X360 --confidence 0.25 --fps 30
```

Two S regions per frame, or four Nano regions per frame:

```sh
python examples/python/Depth/focused_depth_budget.py --depth-model 480X300 --rois-per-frame 2 --fps 30
python examples/python/Depth/focused_depth_budget.py --depth-model 384X240 --rois-per-frame 4 --fps 30
```

For automatic Nano + Medium selection, use `--depth-models` instead of the single-model
and ROI-count arguments:

```sh
python examples/python/Depth/focused_depth_budget.py --depth-models 384X240 576X360 --confidence 0.25 --fps 25 --debug
```

Two or three distinct models are accepted and sorted smallest-first. For each merged
crop, largest-first, the scheduler selects the smallest model that fits its dimensions
and downgrades if needed to fit the remaining throughput budget. It does not assign
models by detection rank or guarantee that every configured model is used.
Nano + Medium cost about 39.9 ms in the estimate, fitting a 25 FPS budget but not 30 FPS.
Both single-model and mixed-model runs overlap three frames. Model switching
and detector contention can still reduce achieved FPS. `--debug` shows the
model selected for each crop. `--rois-per-frame` cannot be combined with `--depth-models`.

For single-model runs, the count is independent of model and camera FPS. Fewer available regions produce
fewer crops. The scheduler does not reduce the requested count to maintain FPS.
The default budget configuration is **two Medium regions**, not two S regions;
select `480X300` explicitly for S.

To measure without visualization, append `--headless --seconds 30`. Add `--debug`
for dispatch details. For the browser ports used in our sessions, append
`--webSocketPort 8767 --httpPort 8084`.

## Arguments

- `--mode roi|detector`: ROI source; ROI wrapper defaults to `roi`, others to `detector`.
- `--depth-mode roi|hold|hybrid`: depth behavior, default `roi`.
- `--roi xmin,ymin,xmax,ymax`: normalized fixed rectangle, default `0.35,0.30,0.65,0.70`;
  used only with `--mode roi`.
- `--depth-model`: `192X120`, `288X180`, `384X240` (Nano), `480X300` (S),
  `576X360` (M, **default**). Every admitted crop uses this model in single-model runs.
- `--depth-models SIZE SIZE [SIZE]`: budget wrapper only; two or three distinct sizes
  from the same choices, with automatic per-crop selection. Mutually exclusive with
  `--depth-model` and `--rois-per-frame`.
- `--rois-per-frame N`: single-model budget only; integer 1–8, default 2.
  Replaces the former `--crop-fps` example argument.
- `--model`: detection model, default `yolov6-nano`.
- `--confidence`: detection threshold in [0, 1], default 0.5; we often use 0.25.
- `--hold-frames X`: consecutive empty messages to bridge in hold mode, default 2;
  0 disables preservation.
- `--stereo-size WIDTH HEIGHT`: EVA input size in hybrid mode, default `384 240`.
  Width must be divisible by 128; positive dimensions, at most `1280 800`.
- `--fps`: requested camera rate, default 30; not guaranteed output FPS.
- `--headless`: disable the browser visualizer.
- `--seconds N`: stop N seconds after pipeline startup, including warm-up; default 0 (unlimited).
- `--frames N`: stop after N received depth frames, including empty frames; default 0 (unlimited).
- `--debug`: print crop counts, model sizes and collection timings.
- `--webSocketPort` / `--httpPort`: visualizer ports, default 8765 / 8082.

## Measured performance

The console reports output FPS and nonempty-frame counts after five seconds of
warm-up. All focused-depth dispatch modes overlap three frames, adding two frames of buffering.

On the local RVC4 at `192.168.88.61`, over 20-second measurement windows: two fixed
S regions reached **30.00 FPS**, or **29.92 FPS** with YOLO also running. Four fixed
Nano regions reached **27.45 FPS**. The actual detection example reached **30.00 FPS**
with one region in the scene. These measurements are specific to that setup;
ROI count, input resolution, model and host/device traffic affect performance.

With Nano + Medium configured at 25 requested FPS, enabling mixed-model pipelining
improved the detector scene (one merged Medium crop) from **12.50 to 25.00 FPS**.
A controlled two-region run, dispatching one crop to each model, improved from
**12.50 to 24.90 FPS**. The budget controls admission, not achieved FPS.
