# Utilities

This folder contains DepthAI utility tools.

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

The executable will be located in `dist/cam_test`.

---

# DepthAI Manual Calibration Instructions

## Manual Camera Calibration with `calibrate.py`

DepthAI devices can be (re-)calibrated using the `calibrate.py` utility from the **depthai-core** repository. This tool captures **Charuco board** images and computes intrinsic and extrinsic calibration parameters for your camera configuration.

> ⚠️ Most OAK devices are factory calibrated. Manual calibration is mainly required for **Modular cameras** (e.g. OAK-FFC series), custom lenses, or non-standard mechanical mounts.

---

## Prerequisites

Before running calibration, install all required Python dependencies listed in `calib_requirements.txt`.

```bash
# Clone depthai-core (if not already cloned)
git clone https://github.com/luxonis/depthai-core.git
cd depthai-core
git submodule update --init --recursive

# Install calibration dependencies
python3 -m pip install -r utilities/calib_requirements.txt
```

---

## Running the Calibration Script

### 1. Prepare the Charuco Board

- Print or display a Charuco board on a **flat surface or screen**
- Measure the **square size in centimeters** accurately
- Know the number of squares in X and Y direction

These values are required by the calibration script.

---

### 2. Capture Calibration Images

```bash
python3 utilities/calibrate.py \
    --board <BOARD_NAME_OR_JSON_PATH> \
    --squareSizeCm <SQUARE_SIZE_IN_CM> \
    --squaresX <NX> \
    --squaresY <NY>
```

- `<BOARD_NAME_OR_JSON_PATH>` may be:
  - A predefined board name, or
  - A custom JSON board configuration file
- The script runs interactively and guides you through image capture

---

### 3. Process Previously Captured Images (Optional)

If images were already captured, run calibration only:

```bash
python3 utilities/calibrate.py \
    --board <BOARD_NAME_OR_JSON_PATH> \
    --squareSizeCm <SQUARE_SIZE_IN_CM> \
    --squaresX <NX> \
    --squaresY <NY> \
    -m process
```

---

## Helpful Tips

- Use `python3 utilities/calibrate.py --help` to see all available options
- Move the camera through **varied angles and distances**
- Ensure **even lighting** and avoid glare or reflections
- Cover the **entire field of view**, including image corners

---

## Calibration Output

After a successful run, the calibration tool will:

- Save calibration files to the `resources/` directory
- Optionally **flash calibration data to the device EEPROM**
- Output reprojection and epipolar error statistics for quality assessment

If calibration fails:
- Verify Charuco board dimensions
- Double-check `--squareSizeCm`
- Improve lighting and marker visibility

---

For official documentation, see:
https://docs.luxonis.com/hardware/platform/depth/manual-calibration/
