# 🔧 DepthAI Dynamic Calibration Interactive script

An interactive calibration and diagnostics tool for OAK cameras using [DepthAI](https://github.com/luxonis/depthai-core). This script provides a real-time visual interface to:

- Trigger dynamic stereo calibration
- View calibration quality metrics (rotation error, depth improvements)
- Visualize depth disparity
- Overlay health bar indicators and coverage information
- Flash and apply calibrations directly on the device

---

## 📸 Features

- ✅ Real-time disparity visualization (`cv2.COLORMAP_JET`)
- ✅ Calibration result summaries printed to screen
- ✅ Health bar overlays for depth accuracy
- ✅ Live keyboard control (see key commands)
- ✅ Coverage map overlay per frame
- ✅ Full DepthAI pipeline using left/right mono cams, stereo, and dynamic calibration

---

## 💻 Key Commands

These can be pressed anytime in the GUI window:

```
[c] → Calibration quality check  
[r] → Recalibrate  
[a] → Force calibration check  
[d] → Force recalibrate  
[n] → Apply new calibration  
[o] → Apply old calibration  
[l] → Flash new calibration  
[k] → Flash old calibration  
[q] → Quit  
```

These are also shown live on the left camera frame overlay.

---

## 🚀 Setup & Usage

### 1. Install Dependencies

You need:

- Python 3.7+
- DepthAI > 3.0
- OpenCV
- NumPy


```bash
pip install depthai opencv-python numpy
```

### 2. Run the Code

```bash
python dynamic_calibration_interactive.py
```

---

## 🧠 Internals

- The viewer listens to the `DynamicCalibration` node for calibration results.
- Results are parsed, filtered, and visualized using OpenCV.
- A health bar shows how calibration changes affect depth error at 1m, 2m, 5m, and 10m.
- Calibrations can be flashed to EEPROM or applied temporarily.

---


## 📷 Example Output

```
<<< -----------------------------|Final Results -- Calibration check|------------------------------------>>>
Rotation change[°]: 0.017 0.043 0.010
Improvements if new calibration is applied:
1m->0.56%,
2m->0.27%,
5m->0.07%,
10m->0.05%
To continue with recalibration, press 'r'.
<<< -----------------------------|Finished|------------------------------------>>>
```

---
