# HFR (High Frame Rate) examples on IMX586 on the RVC4 platform
This directory contains examples demonstrating high frame rate (HFR) capabilities on the IMX586 sensor using the RVC4 platform.
The examples showcase an early preview of the capabilities.


## HFR resolutions
The HFR mode introduces two new resolutions that can run at a high frame-rate.
At the current time, the resolutions cannot be scaled arbitrarily nor can the FPS be varied arbitrarily — it must be one of the two supported values.
We plan to add more flexibility in the future and it's possible to use `ImageManip` in the meantime.

The current supported resolutions are:
* 1920x1080 @ 240 FPS
* 1280x720 @ 480 FPS


## Setup
To set up the HFR mode both a custom version of this (DepthAI) repository and a custom version of the LuxonisOS are required.

### Installation of LuxonisOS:
First download the HFR version of the OS [here](https://luxonisos-rvc4.fra1.digitaloceanspaces.com/OTA/full_update_luxonis_ext4-1.17.0-imx+16bc91752168bda919ce87196bf57d18fb9c43ec.zip)
```
wget https://luxonisos-rvc4.fra1.digitaloceanspaces.com/OTA/full_update_luxonis_ext4-1.17.0-imx+16bc91752168bda919ce87196bf57d18fb9c43ec.zip
```

then copy the OS to the device

```
scp full_update_luxonis_ext4-1.17.0-imx+16bc91752168bda919ce87196bf57d18fb9c43ec.zip root@<device_ip>:/data/hfr_os.zip
```

ssh into the device and update the OS

```
ssh root@<device_ip>
recovery --update_package=/data/hfr_os.zip --wipe_cache && reboot
```

### DepthAI installation
Clone this repository and checkout the `feature/imx586_hfr` branch

```
git clone https://github.com/luxonis/depthai-core.git
cd depthai-core
git checkout feature/imx586_hfr
```

and install requirements

```
python examples/python/install_requirements.py
```

The examples can then be found in the `examples/python/RVC4/HFR` directory.

## Example descriptions
### Object Detection
The object detection [example](hfr_nn.py) demonstrates how to use the HFR mode with a YOLOv6 model for real-time object detection at **480 FPS**.

### Small live preview
The small preview [example](hfr_small_preview.py) demonstrates how to use the HFR mode for a small live preview at **240 FPS** or **480 FPS**.

### Video encoding
The video encoding [example](hfr_save_encoded.py) demonstrates how to use the HFR mode for video encoding at **240 FPS** or **480 FPS**.

All three examples have a `BenchmarkIn` node included that prints the framerate and the latency. This is the expected output:
```
[2025-08-14 23:31:49.487] [ThreadedNode] [warning] FPS: 474.3766
[2025-08-14 23:31:49.487] [ThreadedNode] [warning] Messages took 1.0118543 s
[2025-08-14 23:31:49.487] [ThreadedNode] [warning] Average latency: 0.05904912 s
```