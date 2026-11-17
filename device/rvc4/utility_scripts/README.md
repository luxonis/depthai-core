# Utility scripts

Helper scripts for building, deploying and debugging the RVC4 firmware, and for generating the firmware package
license bundle and SBOMs.

## Development

| Script | Where to run | What it does |
| --- | --- | --- |
| `build.sh` | host | Incremental build of `depthai-device-rvc4` (RelWithDebInfo) in the running development container |
| `deploy.sh <device-ip>` | host | Copies the firmware, `env_set.sh`, the Hexagon skeleton libraries and `device_based/` to `/data` on the device |
| `ex.sh "<command>"` | host | Runs a command in the running development container |
| `bash_ex.sh` | host | Opens an interactive shell in the running development container |
| `container_based/build_rvc4.sh` | container | The build command that `build.sh` runs |
| `container_based/debug.sh` | container | Connects the SDK gdb to the gdbserver on the device (`DEVICE_IP=<device-ip>`) |
| `container_based/run_clangd.sh` | container | Starts a clangd instance on TCP port 6010 that editors on the host can use |
| `device_based/run.sh` | device | Stops `depthai_gate` and runs the deployed firmware |
| `device_based/debug.sh` | device | Stops `depthai_gate` and runs the deployed firmware under `gdbserver :2000` |

## Core dump parsing

| Script | Where to run | What it does |
| --- | --- | --- |
| `parse_core_dump.sh <core_dump.tar.gz>` | host | Unpacks a core dump archive from the device and prints its backtrace (non-interactive) |
| `container_based/debug_core_dump.sh <core>` | container | Opens an already unpacked core dump in an interactive gdb session |

## License bundle and SBOM

| Script | What it does |
| --- | --- |
| `generate_license.py` | Writes the firmware package license bundle (`depthai-device-rvc4-fwp-LICENSE`) and the SPDX 2.3 / CycloneDX 1.6 SBOMs. CMake runs it after the firmware package is built. The bundle opens with `LICENSE.fwp` (the firmware package license), not with the Apache-2.0 source license in `LICENSE`. |
| `generate_third_party_licenses.py` | The collector: lists the vcpkg packages of the build, the vendored submodules of this repository and the third-party parts of the private `depthai-device-rvc4-vendor` module (found through `--private-module-root`, `DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR` or the sibling directory). |
| `fwp_sbom.py` | SBOM generation used by `generate_license.py` (vcpkg dependency graph, files of the fwp archive and their runtime libraries). |

## Usage

The scripts in this folder run on the host. The scripts in `container_based/` run inside the development container,
which `./start_dev_shell.sh rvc4` starts:

```bash
./utility_scripts/bash_ex.sh
cd /work/device/rvc4/utility_scripts/container_based
DEVICE_IP=<device-ip> ./debug.sh
```

The scripts in `device_based/` run on the device after `deploy.sh`:

```bash
ssh root@<device-ip>
/data/device_based/debug.sh
```

## Debugging a stuck production firmware

On the device, freeze the `depthai_gate` process so that its watchdog does not kill `depthai-device-rvc4`:

```bash
kill -STOP $(systemctl show --property MainPID --value depthai_gate.service)
```

In a second shell on the device, attach `gdbserver` to the running firmware:

```bash
gdbserver :2000 --attach $(pgrep -f depthai-device-rvc4)
```

In a third shell, connect from the development container:

```bash
./utility_scripts/bash_ex.sh
DEVICE_IP=<device-ip> /work/device/rvc4/utility_scripts/container_based/debug.sh
```

When you are done, unfreeze `depthai_gate`:

```bash
kill -CONT $(systemctl show --property MainPID --value depthai_gate.service)
```
