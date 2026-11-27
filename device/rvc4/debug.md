# Debugging the firmware on an RVC4 device

The firmware runs under `gdbserver` on the device; gdb runs either in the SDK container (recommended, the SDK gdb
knows the target and its sysroot) or on the host with `gdb-multiarch` and VS Code.

## Build, deploy, and run under gdbserver

```bash
./start_dev_shell.sh rvc4                                   # terminal 1: SDK container (keep it open)
./utility_scripts/build.sh                                  # terminal 2: incremental build in that container
DEVICE_IP=<device-ip> ./utility_scripts/deploy.sh           # copies the firmware and helper scripts to /data
ssh root@<device-ip> /data/device_based/debug.sh            # stops depthai_gate, runs the firmware under gdbserver :2000
```

Then connect from the container (`./utility_scripts/bash_ex.sh`, then
`DEVICE_IP=<device-ip> /work/device/rvc4/utility_scripts/container_based/debug.sh`) and start a `depthai-core` example on the
host with `DEPTHAI_WATCHDOG=0`, so the watchdog does not end the session.

`depthai_gate` must be stopped while you debug: it owns the discovery port and restarts the firmware otherwise.
Start it again when you are done: `ssh root@<device-ip> systemctl start depthai_gate`.

## Split debug symbols

The build strips `depthai-device-rvc4` and keeps the symbols in `depthai-device-rvc4.debug` in the same
directory (`.gnu_debuglink`). gdb finds the `.debug` file automatically when it lies next to the binary that you
pass to gdb; VS Code passes it explicitly with `-symbols <binary>.debug`. Set the sysroot so that the shared
libraries of the device resolve:

```
set sysroot /usr/local/oe-sdk-hardcoded-buildpath/sysroots/aarch64-oe-linux
```

(inside the SDK container; `$OECORE_TARGET_SYSROOT` after `source /usr/local/oe-sdk-hardcoded-buildpath/environment-setup-aarch64-oe-linux`).

## From the host with gdb-multiarch and VS Code

1. Install gdb: `sudo apt install gdb-multiarch` (gdb 12.1 or newer; older 12.0 packages have a bug with remote
   targets).
2. Copy the firmware to the device and start it under `gdbserver` (any free port, matched in `launch.json`):

   ```bash
   scp build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4 root@<device-ip>:/home/root/
   ssh root@<device-ip> "systemctl stop depthai_gate && gdbserver :<port> /home/root/depthai-device-rvc4"
   ```

3. Add this configuration to `.vscode/launch.json` and start it (`F5`):

   ```json
   {
       "name": "Remote RVC4 debugging (gdb-multiarch)",
       "type": "cppdbg",
       "request": "launch",
       "program": "${workspaceFolder}/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4",
       "miDebuggerArgs": "-symbols ${workspaceFolder}/build_docker_arm64_rvc4/RelWithDebInfo/depthai-device-rvc4.debug",
       "args": [],
       "stopAtEntry": false,
       "cwd": "${workspaceFolder}",
       "environment": [],
       "externalConsole": true,
       "MIMode": "gdb",
       "miDebuggerServerAddress": "<device-ip>:<port>",
       "miDebuggerPath": "/usr/bin/gdb-multiarch",
       "setupCommands": [
           {
               "description": "Enable pretty-printing for gdb",
               "text": "-enable-pretty-printing",
               "ignoreFailures": false
           }
       ]
   }
   ```

   Without the SDK sysroot on the host, gdb resolves only the symbols of the firmware itself; for full backtraces
   through the system libraries use the SDK container (the dev container in `.devcontainer/` does this with one
   click, see [vscode_development](vscode_development/vscode_development.md)).

## Attaching to a stuck production firmware

See [utility_scripts/README.md](utility_scripts/README.md): freeze `depthai_gate` with `SIGSTOP` so its watchdog
does not kill the firmware, attach `gdbserver --attach` to the running `depthai-device-rvc4`, connect from the
container, and unfreeze the gate with `SIGCONT` afterwards.

## Core dumps

`./utility_scripts/parse_core_dump.sh <core_dump.tar.gz>` unpacks a core dump archive from the device and prints
the backtrace with the SDK gdb; `utility_scripts/container_based/debug_core_dump.sh <core>` opens it
interactively. In VS Code, the **Debugging a core dump RVC4** configuration does the same.
