# Agent Context For `depthai-core`

DepthAI is a library for interfacing with Luxonis OAK cameras and building custom vision pipelines that run across host and device components.

This repository is the DepthAI core library: C++ sources, Python bindings, examples, tests, build scripts, and generated artifacts.

Use this file for baseline repo behavior only. For task-specific workflows, check `.agent/skills/`.

## Default Working Rules

- Prefer focused changes. Avoid unrelated refactors, renames, or formatting churn.
- Follow nearby code style and existing API patterns.
- Treat hardware-dependent tests as conditional. Do not assume RVC2 or RVC4 devices are available.
- This is a stable library, meaning we should **minimize behavioral risk** in all changes. If a change is risky, call that out clearly and consider whether it should be split into smaller steps or require more validation. Any breaking changes to the end user should be avoided. If they are absolutely necessary, they should be carefully considered and outlined.
- If public API or user-facing behavior changes, update tests and examples as needed.

## Quick Orientation

- Main library code: `src/`, `include/`
- Python bindings: `bindings/python/`
- Examples: `examples/cpp/`, `examples/python/`
- Tests: `tests/`
- Build and CI scripts: `ci/`, `scripts/`
- Protobuf definitions: `proto/`

## Iteration

The fastest local loop is to run tests without running anything on device:

```bash
cmake -S . -B build
cmake --build build --parallel 8
cd build
ctest -L onhost --output-on-failure --no-tests=error
```

Use "hardware in the loop" tests that require a device for validation. You can check for connected devices with:
`pip install depthai` and then `python -c "import depthai; print(depthai.Device.getAllAvailableDevices())"`.
If the needed device (RVC2 or RVC4) is not available, request the user to connect one.

For more in depth options and workflows, see `.agent/skills/build-and-test/`.


## Branch Context

- `develop` is the main development branch. Use it for all non-hotfix work.
- `main` is the stable release branch. Use it only for hotfixes or backports.
- Use `v2_stable` and `v2_develop` for v2-targeted changes.

## Deeper Guidance

- Contributor workflow: `CONTRIBUTING.md`
- Task-focused agent workflows:
  - `.agent/skills/pr-review/`

Load those skills when the task clearly matches.
