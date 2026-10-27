# Utilities

This folder contains DepthAI utility tools.

## Burro PoE stress test

For a fresh machine, follow [the complete setup and run guide](BURRO_STRESS_TEST.md),
including the pinned SDK install and firmware download/checksum commands.

From the repository root, with the existing `.venvV3` environment active:

```sh
python -u utilities/stress_test.py --burro --headless --duration 3600 --report burro-stress.json > burro-stress.log 2>&1
```

This selects Burro at `10.12.204.228` and loads the tested `poe-perf` firmware
(`dcc78c03f6df6b5efc9ad4a9dda73beb0716d50c`) into RAM. The preset looks for the
downloaded `.cmd` in `../capture/poe-perf-20261027/` relative to the repository.
Use `--firmware /path/to/firmware.cmd` to choose another file, or set
`DEPTHAI_DEVICE_BINARY`. No firmware is flashed.

The RVC2 defaults exercise all three cameras at 20 FPS, one H.264 encoder per
camera, stereo depth with left/right checking and subpixel enabled, one YOLOv6
nano network, and system logging. The model zoo may download the model on first
use. All camera outputs use the same FPS to avoid the RVC2 mixed-rate issue.

`--duration` counts seconds after pipeline startup; `0` runs until Ctrl+C.
Omit `--headless` to show previews. The headless mode drains camera, encoded,
depth, and detection outputs without decoding or rendering them. Statistics
include delivered host FPS, maximum packet gaps, peak temperature, and peak DDR
usage. A silent output (including system logging) fails the test after 20 seconds;
adjust this with `--stall-timeout`. A pass means the workload completed without
exceptions or stalled streams; it does not assert that requested FPS was reached.
Nonblocking host queues may drop packets under load.

Inspect progress from another terminal:

```sh
tail -f burro-stress.log
```

The process exits with status `0` on completion, `1` on a workload failure, and
`130` when interrupted. Runtime results are saved in the JSON report, including
on a detected stall. Connection/build/startup failures appear in the log and exit
nonzero before a runtime report is available. For a heavier camera/encoder load,
add `--camera-fps 30 --encoder-fps 30`; for a camera/encoder/stereo-only test,
add `--no-nnet`.

## Device Manager

![Device Manager](https://user-images.githubusercontent.com/18037362/171629704-0f78f31a-1778-4338-8ac0-bdfb0d2d593f.png)

``device_manager.py`` helps interfacing with the device [Bootloader](https://docs.luxonis.com/projects/api/en/latest/components/bootloader) and bootloader configuration. See [Device Manager Usage](https://docs.luxonis.com/projects/api/en/latest/components/bootloader/#device-manager-usage) to see how to use this utility.

## Standalone executable

Requirements:
```
# Linux/macOS
python3 -m pip install pyinstaller
# Windows
python -m pip install pyinstaller
```

To build standalone executable issue the following command:
```sh
pyinstaller --onefile -w --icon=assets/icon.ico --add-data="assets/icon.ico;assets" --add-data="assets/icon.png;assets" device_manager.py
```

Optionally, append `--runtime-tmpdir [path or .]` to modify where the temporary directory should be created when launched.


## Cam Test
Run:
```sh
python3 cam_test.py
```
To start cam test with GUI.  
Run cam_test.py with args to start cam test without GUI:  

### Bundled executable
Requirements:
```
# Linux/macOS
python3 -m pip install pyinstaller
# Windows
python -m pip install pyinstaller
```

To build a bundled executable issue the following command:
```sh
pyinstaller -w cam_test.py --hidden-import PyQt5.sip
```

The executable will be located in `dist/cam_test` folder.
