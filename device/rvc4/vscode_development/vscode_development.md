# VS Code development

The dev container in `.devcontainer/` gives you a VS Code workspace inside the Luxonis OS RVC4 SDK image, with
code completion from clangd, CMake integration and one-click remote debugging of the firmware on a device.

## Requirements

* `docker` and `docker compose`
* VS Code with the Dev Containers extension (the other extensions are installed in the container, see
  `.devcontainer/rvc4-container/devcontainer.json`)
* Access to the SDK image. Today this is the private Luxonis OS SDK image; the public image
  `ghcr.io/luxonis/depthai-device-sdk` will replace it once it is published. Set `DEPTHAI_DEVICE_SDK_IMAGE` in your
  environment to use another image.
* The vendor module: by default CMake downloads its prebuilt archive. `DEPTHAI_DEVICE_RVC4_VENDOR=<archive>` in your
  environment takes a local archive below your home folder. `DEPTHAI_DEVICE_RVC4_VENDOR_SOURCE_DIR=<path>` takes a checkout of its sources, the
  private repository `depthai-device-vendor-fw` (Luxonis licensees); it is mounted at `/depthai-device-rvc4-vendor`.

`.devcontainer/createEnv.sh` runs when the container is created and writes these values into
`.devcontainer/.env` (gitignored).

## Getting started

1. Clone the repository with its submodules (see `README.md`).
2. Open the repository in VS Code, press `Ctrl + Shift + P` and run **Dev Containers: Open Workspace in Container**.
3. Enter the IP address of the device when prompted (task `Get IP`; it writes `BOARD_IP=<device-ip>` to
   `docker/boardIP`, which is gitignored). You can change it later by running the task again.
4. Select the `rvc4` kit when CMake Tools asks for a kit, and configure a `RelWithDebInfo` or `Debug` build.
5. Build with the **Build** button in the status bar. The build directory is `build_docker_arm64_rvc4`, the same
   one that `docker/build.sh` uses, so a build from the terminal and a build from VS Code share their objects.

Code completion and navigation work as soon as the first configure has produced `compile_commands.json`.

## Debugging

1. In the **Run and Debug** view select **Remote debugging (CMake launch target)** and press play (`F5`).
2. The pre-launch task runs `remote_build_and_run_rvc4.sh`: it builds the firmware, stops `depthai_gate` on the
   device, copies the binary to `/home/root/debug_fw/` and starts it under `gdbserver` on port 1234. gdb connects
   with the split debug symbols (`depthai-device-rvc4.debug`) and the SDK sysroot.
3. When the terminal shows `Created XLink server and waiting for connection...`, run any `depthai-core` example
   on the host; it connects to the firmware under the debugger. Set `DEPTHAI_WATCHDOG=0` for the example so the
   watchdog does not end the session.
4. Start `depthai_gate` again when you are done: `ssh root@<device-ip> systemctl start depthai_gate`.

**Debugging a core dump RVC4** opens a core dump from the device with the same symbols and sysroot; it asks for the
path of the core file.
