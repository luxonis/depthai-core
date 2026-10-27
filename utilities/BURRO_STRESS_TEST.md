# Run the Burro PoE stress test from scratch

These instructions run the test **on a Linux host connected to Burro**, targeting
the camera at `10.12.204.228` (MXID `19443010E175078C00`). The host runs Python;
the camera runs the pipeline. Use one device client at a time: stop your own
`cam_test`, viewer, or earlier stress test before connecting.

## 1. Install prerequisites and clone the branch

For Ubuntu with Python 3.12:

```bash
sudo apt-get update
sudo apt-get install -y git curl xz-utils python3-venv libusb-1.0-0 libgl1 libglib2.0-0
git clone --branch custom/burro-poe-stress --single-branch https://github.com/luxonis/depthai-core.git
cd depthai-core
python3 -m venv .venv-burro
source .venv-burro/bin/activate
python -m pip install --upgrade pip
python -m pip install 'depthai==3.11.0' 'opencv-python==4.14.0.94' 'numpy==2.5.4'
python utilities/stress_test.py --help
```

No C++ build or submodule checkout is required. This uses the prebuilt Python SDK,
not a source build of the branch. Installation and CLI import were checked in a
clean Python 3.12 environment with these package versions. The earlier hardware
smoke test used the existing SDK `3.11.0.dev0+b5e5203a25750e4b6db91bd5a034dfe8d6c9fa14`.

## 2. Download the tested firmware

Pin the firmware to `poe-perf` commit
`dcc78c03f6df6b5efc9ad4a9dda73beb0716d50c`. Run from the repository root:

```bash
mkdir -p .firmware/poe-perf
curl --fail --location --retry 3 \
  'https://artifacts.luxonis.com/artifactory/luxonis-myriad-snapshot-local/depthai-device-side/dcc78c03f6df6b5efc9ad4a9dda73beb0716d50c/depthai-device-fwp-dcc78c03f6df6b5efc9ad4a9dda73beb0716d50c.tar.xz' \
  --output .firmware/poe-perf/firmware.tar.xz
echo '4ffe50c7a6479f3946e4a4b3a3b6e17e462b0125319ecd7e964f67cf0c06a1ee  .firmware/poe-perf/firmware.tar.xz' | sha256sum --check
tar -xJf .firmware/poe-perf/firmware.tar.xz -C .firmware/poe-perf
```

The checksum command must print `OK`; do not continue after a download or checksum
failure. The resulting `depthai-device-openvino-universal-*.cmd` is the complete
firmware binary. The `.patch` files are not needed. Access to the Luxonis artifact
host is required; if access fails, obtain this exact package through your usual
internal access route.

## 3. Check connectivity and run a short test

The host needs a route to `10.12.204.228`. From the repository root, with the
virtual environment active:

```bash
ping -c 3 10.12.204.228
python -u utilities/stress_test.py \
  --burro \
  --firmware .firmware/poe-perf/depthai-device-openvino-universal-dcc78c03f6df6b5efc9ad4a9dda73beb0716d50c.cmd \
  --headless --duration 60 --report burro-smoke.json
echo "Exit status: $?"
```

The first neural-network run may download or prepare the model. The run duration
starts after pipeline startup. Firmware is loaded into RAM on each connection;
the test does not flash firmware, calibration, or network configuration. The
branch firmware makes RGB upright without an application rotation override.

## 4. Run for an hour

```bash
python -u utilities/stress_test.py \
  --burro \
  --firmware .firmware/poe-perf/depthai-device-openvino-universal-dcc78c03f6df6b5efc9ad4a9dda73beb0716d50c.cmd \
  --headless --duration 3600 --report burro-stress.json \
  > burro-stress.log 2>&1
echo "Exit status: $?"
```

This runs in the foreground; keep the terminal open. Inspect progress in another
terminal with `tail -f burro-stress.log`. For an overnight run, use
`--duration 28800` (eight hours). Use Ctrl+C to stop early. Reactivate the same
environment with `source .venv-burro/bin/activate` in a new terminal.

The default RVC2 workload runs all three sensors at 20 FPS, previews, three H.264
encoders, stereo depth with left/right checking and subpixel, YOLOv6 nano
inference, and a 1 Hz system logger. Host queues are nonblocking, so delivered FPS
can be below requested FPS. The previous 35-second hardware run completed with
all eight outputs active, about 12–13 FPS delivered, and peak reported average
chip temperature of 45.3 degrees C. This short run does not establish long-term
stability.

## 5. Read the result and adjust the load

```bash
python -m json.tool burro-stress.json
```

The JSON contains `outcome`, `error`, per-stream packet counts, delivered host FPS,
maximum packet gaps, peak reported average chip temperature, peak DDR usage, and
the firmware path. A stream or system logger silent for 20 seconds fails the test
(`--stall-timeout` changes this limit). A pass checks continued output, not a
minimum throughput or image correctness. Exit codes: `0` completed, `1` failed,
`130` interrupted. Connection, model preparation, and pipeline-start failures
exit nonzero and appear in the log before a runtime JSON report is available.
Use a new report filename for each run so a previous report cannot be mistaken
for the latest result.

Append options to either run command as needed:

- `--camera-fps 30 --encoder-fps 30`: increase camera and encoder load.
- `--no-nnet`: exercise cameras, encoding, and stereo without model inference.
- `--no-stereo`: remove the depth workload.
- Omit `--headless`: display previews to check RGB orientation on a desktop host.

All requested outputs of a given camera use the same FPS. Avoid independently
changing the camera output rates: mixed rates previously caused severe RVC2
slowdowns. Increasing load may lower throughput or exhaust device resources.

The tested firmware lacks the SDK's `getEthernetLinkSpeed` RPC and may print
`Could not determine Ethernet link speed`. This warning alone did not prevent
the hardware test from streaming successfully.
