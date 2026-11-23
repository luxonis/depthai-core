# NNArchive runtime dependencies

An NNArchive for RVC4 can bundle the SNPE User Defined Operation (UDO)
libraries that its DLC needs. Load the archive with the usual
`NeuralNetwork.setNNArchive()` or `setModelPath()`. The SDK sends the declared
libraries to the device with the DLC, and the device registers them with SNPE
before it builds the network. You do not need a custom OS image.

This feature needs DepthAI 3.13.0 or later with its bundled RVC4 firmware.
Older releases load these archives but ignore the libraries, so SNPE fails to
build the network. It was tested on Luxonis OS 1.45.0 with SNPE 2.41.

Only use archives from sources you trust, such as your own builds or official
Luxonis Zoo models. A UDO library runs native code on the device with the
privileges of the DepthAI process. Archive validation does not sandbox it.

## Archive format

The schema is the one from
[LuxonisML PR #536](https://github.com/luxonis/luxonis-ml/pull/536), released in
LuxonisML 0.10.3. Declare the libraries in `config.json` and add them to the
archive as regular files:

```json
{
  "config_version": "1.1",
  "model": {
    "metadata": {"name": "custom-model", "path": "model.dlc"},
    "inputs": [],
    "outputs": [],
    "heads": [],
    "runtime_dependencies": {
      "snpe_udo": [{
        "registration": "runtime/libUdoCustomPackageReg.so",
        "implementations": [
          {"runtime": "dsp", "path": "runtime/libUdoCustomPackageImplDsp.so"}
        ]
      }]
    }
  }
}
```

Replace the empty `inputs`, `outputs` and `heads` with the model's real
definitions.

- Paths follow the LuxonisML rules: normalized, relative POSIX paths inside
  the archive, without backslashes, NUL bytes or a Windows drive, and not
  `config.json`.
- The SDK rejects missing, empty, duplicate and linked library entries.
- The SDK sends only the registration and DSP implementation libraries to the
  device. It does not transfer or load files that `runtime_dependencies` does
  not declare.
- An archive without `runtime_dependencies`, or with `null`, works as before.

## Supported runtimes

Only SNPE DLC models on RVC4 are supported. Each package needs an AArch64
registration library and a Hexagon DSP implementation (`"runtime": "dsp"`, in
any letter case). The network must run on the DSP runtime, which is the
default.

A DLC that was not prepared for the HTP in advance (it has no
`backend.metadata0` record, see `snpe-dlc-graph-prepare`) is prepared on the
device CPU. SNPE then also needs the AArch64 build of each DSP implementation,
named `<DSP library>_AltPrep.so`, for example
`libUdoSoftmaxUdoPackageImplDsp_AltPrep.so` next to
`libUdoSoftmaxUdoPackageImplDsp.so`. Declare it as another `"dsp"`
implementation of the same package. Without it, the network build fails with
"No backend could validate Op". An AArch64 DSP implementation with any other
name is rejected. Selecting the CPU, GPU or another non-DSP SNPE runtime with
`setBackendProperties({"runtime": ...})` fails when the pipeline starts; a
runtime name that SNPE does not know runs on the DSP. A package without a DSP
implementation fails in `setNNArchive()`. The SDK does not send
implementations for other runtimes, so an archive can still include them for
other consumers.

The libraries must match the device's SNPE SDK version. Qualcomm requires DSP
V68 and later UDOs to use the same SNPE release they were compiled against; see
[Qualcomm's UDO compatibility documentation](https://docs.qualcomm.com/nav/home/udo_overview.html?product=924033590759186372).
The device checks that the registration and `_AltPrep` libraries are AArch64
ELF shared objects and that the other DSP implementations are Hexagon ones.
SNPE reports other incompatibilities when it registers the package or builds
the network.

## Library names and lifetime

SNPE finds a DSP implementation library by the file name that the registration
library contains, and its `_AltPrep` build by that name with the `_AltPrep`
suffix. Keep the archive file names equal to these names; the device ignores
the directory part of the archive path.

Give each library a file name that is unique to its package, for example by
prefixing the model name. Different libraries with the same file name fail:

- in one archive, `setNNArchive()` fails with "Conflicting NNArchive UDO
  libraries";
- in two archives used in one session, the second network fails with
  "Conflicting NNArchive UDO library name";
- if the device already has a different library with that name on its DSP
  search path, such as a firmware, SNPE or LENS library, or on
  `LD_LIBRARY_PATH` for an `_AltPrep` library, the network fails with
  "conflicts with the device library". Identical copies are allowed.

The UDO package name declared inside the registration library must also be
unique. SNPE cannot register two different registration libraries that declare
the same package name in one session, for example an OS LENS package and a
modified copy of it. The second network fails with "Could not register UDO
package".

SNPE registrations belong to the device process and cannot be removed:

- A registered package stays available until the device session ends, even if
  a later network build fails.
- Networks in the same session share identical libraries and registrations.
- To replace a library, start a new device session.

The device stages the libraries in a private directory inside its firmware
directory, and `_AltPrep` libraries in a directory that the device's session
manager adds to `LD_LIBRARY_PATH` when it starts the firmware. Both are removed
when the session ends, and files that a crashed session left behind are removed
when the next session starts.

## Testing

`neural_network_udo_test` runs on RVC4. By default it runs the error cases,
including archives whose placeholder libraries, with and without an `_AltPrep`
library, go through `setNNArchive()` to SNPE registration. To also compare a
UDO archive against a reference archive, set `DEPTHAI_TEST_UDO_ARCHIVE` and
`DEPTHAI_TEST_UDO_REFERENCE_ARCHIVE` to two archives of the same model, one
with UDOs and one without, and run `neural_network_udo_test "[udo]"` from a
build with OpenCV support. The UDO outputs must be identical across networks
and four device sessions, and each output must correlate with the reference
(Pearson correlation of at least 0.98), which allows a reference of another
precision. Select the device with the usual `DEPTHAI_DEVICE_NAME_LIST` or
`DEPTHAI_DEVICE_ID_LIST`.

### Tested

On an OAK 4 with Luxonis OS 1.45.0 and SNPE 2.41, from a Linux host:

- The YOLOv8n-pose archive with a Softmax UDO package (its DLC is prepared on
  the device, so it needs the `_AltPrep` library) gives the same outputs
  through `NeuralNetwork` as `snpe-net-run` with the same libraries: identical,
  or within one float32 rounding step for two keypoint outputs. Through
  `DetectionNetwork` it finds the same 76 people in 30 video frames as the INT8
  Zoo model `luxonis/yolov8-nano-pose-estimation:coco-512x288`, with a mean
  box IoU of 0.96 and a mean keypoint distance of 4 px at 512x288. The
  differences come from the precision of the two models.
- A LENS UDO model packed into an archive gives bit-identical outputs to the
  same model loaded from the OS, in repeated sessions.
- The staged libraries are removed when a session ends. After the firmware is
  killed, they are removed by the time the next session starts, and that
  session runs the archive again.

Not tested: hosts other than Linux, and UDO packages for runtimes other than
the DSP.
