---
name: depthai-debugger
description: Investigate DepthAI SDK or device failures, pipeline stalls, frame corruption, and performance regressions using a reproducible case, matching build artifacts, and device evidence. Use for diagnosis and regression isolation, not routine C++ review alone.
---

# DepthAI debugging

Find the first failing boundary, demonstrate the cause, and retain a small reproduction
or regression test. Read the repository's AGENTS.md before choosing checks or builds.

## Establish the version actually running

- Record host commit and worktree changes, but also print `depthai.__file__` and
  `depthai.__version__` for Python. A checkout and its existing extension can differ.
- For native binaries, inspect the build configuration and linked library paths.
  Record platform, transport, sensor, board variant, and relevant runtime overrides.
- Check the firmware pin and any `DEPTHAI_DEVICE_BINARY` / `DEPTHAI_DEVICE_RVC4_FWP`
  override. A firmware banner or dump's version identifies what ran; a source checkout
  alone does not. Symbolize only with a matching ELF and appropriate platform debugger.
- Preserve unrelated edits. An isolated worktree is useful when a firmware checkout
  already contains another investigation's changes.

## Choose the failing boundary

Use the supplied task description, comments, linked reproduction, and complete log
before changing code. Treat an inaccessible attachment or incomplete Slack history
as a specific evidence gap.

| Symptom | Evidence that separates likely causes |
| --- | --- |
| Queue get/send never returns | Producer/consumer state, queue occupancy, message type, Python GIL ownership, and shutdown stack. |
| Frames stop or arrive late | Source FPS, sequence numbers, timestamp domain, processing/send times, pool pressure, and transport throughput. |
| Wrong pixels or tensor results | Byte count, stride, element size, planar/interleaved layout, decoded input bytes, and transformations. |
| XLink errors after a control change | The first device error, message-size change, allocation failure, and per-transport recovery. Later protocol errors may be consequences. |
| Failure only during destruction | All thread stacks, callback ownership, last shared owner, cancellation, and joins. |
| Calibration changes are ignored | Which node cached which calibration, generation/update signal, and per-frame transformations. |

For stalls and latency, read [PipelineDebugging.md](../../../PipelineDebugging.md).
Capture `pipeline.getPipelineState().nodes().detailed()` at more than one instant;
a single waiting state can be normal. Inspect source-to-sink dependencies, including
host bridges and every input of a merging node. Group multi-device nodes by their
schema `deviceId`, and compare timestamps only in a shared clock domain.

## Reproduce and isolate

Keep the same pipeline, data, device, transport, and initial configuration while
changing one variable. Derive runtime controls from the intended initial config;
creating an empty replacement can also change unrelated filters or frame sizes.
Compare payload bytes when investigating codecs or architecture-dependent inference.

Use finite queue timeouts and an outer process timeout for a hanging reproduction.
After an induced disconnect, establish device readiness before another run. Verify
that a callback or test assertion ran; a process that only started is not a pass.

For a native crash or deadlock, a useful starting point is:

```text
gdb --args <existing-binary> <reproduction-arguments>
run
thread apply all bt
```

For Python, run the actual interpreter under the native debugger when C++ frames
matter. IDE breakpoints in a C++-created Python thread may need
`debugpy.debug_this_thread()` or PyCharm's
`pydevd.settrace(suspend=False, trace_only_current_thread=True)` inside that thread.
That debugger attachment is distinct from a pipeline synchronization fix.

## Verify the correction

Add a test at the failing boundary. A datatype change also needs the repository's
serialization coverage on RVC2 and RVC4. Review changes with
[depthai-cpp-review](../depthai-cpp-review/SKILL.md) when applicable.

Distinguish results from the original binary, changed source, and rebuilt artifact.
When builds are left to the user, finish source checks and prepare exact test targets;
do not report an old binary's pass as validation of the fix. Record firmware/build
dependencies and any hardware that was unavailable.

Keep the final investigation useful: task link, versions, reproduction, observed
failure, causal evidence, change, regression coverage, test results, and remaining
uncertainty. Keep private firmware excerpts and deployment details out of public
commits and PR text.
