# RVC2/RVC4 stereo parity HIL

The opt-in HIL tests capture rectified frames on a physical RVC2, replay the
same pixels and metadata on both devices, and require byte-identical disparity
and depth. The main test covers every RVC2 preset at D64 and D96 plus isolated
default-preset filter changes. The transition test keeps one node alive while
switching between extended and full-resolution execution paths.

Configure a dedicated build with both endpoints:

```console
cmake -S . -B build-hil \
  -DDEPTHAI_BUILD_TESTS=ON \
  -DDEPTHAI_BUILD_PYTHON=ON \
  -DDEPTHAI_TEST_STEREO_RVC2_RVC4_HIL=ON \
  -DDEPTHAI_TEST_STEREO_RVC2_DEVICE=<RVC2-MXID> \
  -DDEPTHAI_TEST_STEREO_RVC4_DEVICE=<RVC4-IP> \
  -DDEPTHAI_TEST_STEREO_PARITY_CORPUS=<durable-corpus-directory>
cmake --build build-hil --parallel
```

Run both serialized hardware tests against the exact firmware archive under
test:

```console
DEPTHAI_DEVICE_RVC4_FWP=<depthai-device-rvc4-fwp.tar.xz> \
  ctest --test-dir build-hil --output-on-failure -V \
  -R 'stereo_rvc2_(rvc4_parity|runtime_transition)_hil'
```

`--min-rvc4-fps 3.0` in the registered parity test is only a failure/liveness
floor. Release qualification must separately compare sustained host FPS from
the candidate and its baseline artifact.
