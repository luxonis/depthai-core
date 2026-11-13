DepthAI Core is the C++ SDK for Luxonis cameras, with Python bindings
through pybind11. It builds on Linux, macOS, and Windows (MSVC). The
firmware for the cameras (RVC2: OAK 1 and OAK 2, RVC4: OAK 4) is a
prebuilt download that CMake embeds in the library.

## Layout

- `include/depthai/` — public headers. `pipeline/datatype/`, `properties/`
  and `common/` are shared with the firmware: they are the wire format
  between host and device.
- `src/` — implementation. Mirrors `include/depthai/`.
- `bindings/python/` — pybind11 bindings; a new public API needs a binding.
  `bindings/python/src/` mirrors `include/depthai/`.
- `examples/cpp/`, `examples/python/` — examples. Each example exists in
  both languages with the same name.
- `tests/src/onhost_tests/` — tests that run without a device.
- `tests/src/ondevice_tests/` — tests that need an RVC2 or RVC4 device.
- `tests/CMakeLists.txt` — `dai_add_test(...)` registers a test and its
  `ctest` labels (`onhost`, `rvc2`, `rvc4`, `usb`, `poe`, ...).
- `cmake/Depthai/` — firmware pins: `DepthaiDeviceSideConfig.cmake` (RVC2),
  `DepthaiDeviceRVC4Config.cmake` (RVC4).
- `cmake/depthaiDependencies.cmake` — XLink and other dependencies.
- `protos/` — protobuf schemas; run `ci/check_protobuf_consistency.sh`
  after a change.
- `ci/` — CI check scripts. `.github/workflows/` — CI workflows.
- `shared/`, `3rdparty/`, `bindings/python/external/` — submodules and
  vendored code; do not edit.

## Build

```bash
git submodule update --init --recursive
cmake -S . -B build -D DEPTHAI_BUILD_TESTS=ON -D DEPTHAI_BUILD_EXAMPLES=ON -D DEPTHAI_BUILD_PYTHON=ON
cmake --build build --parallel 10
```

The first configure builds the vcpkg dependencies and takes a long time.
To run your own firmware, set `DEPTHAI_DEVICE_BINARY` (RVC2) or
`DEPTHAI_DEVICE_RVC4_FWP` (RVC4).

## Test

```bash
cd build
ctest -L onhost          # no device needed
ctest -L rvc2            # needs an RVC2 device
ctest -L rvc4            # needs an RVC4 device
```

Every new node, message, feature and bug fix needs a test that exercises
the real behaviour. A change to a type that derives from `Buffer` needs a
serialization roundtrip test that runs on RVC2 and RVC4.

## Style and CI checks

CI pins clang-format-18 and clang-tidy-18. Use only these versions.

```bash
cmake -S . -B build -DDEPTHAI_CLANG_FORMAT=ON
cmake --build build --target clangformat
git --no-pager diff          # any diff here is a CI failure

cmake -S . -B build -DDEPTHAI_CLANG_TIDY=ON -DCLANG_TIDY_BIN=/usr/bin/clang-tidy-18

bash ci/check_datatype_enum_consistency.sh
bash ci/check_protobuf_consistency.sh
```

## Rules that are easy to break

- C++17 only.
- Keep the public API backward compatible. Rename or remove a symbol
  through `[[deprecated("Use <replacement> instead")]]`.
- Shared types (`datatype/`, `properties/`, `common/`): append a new field
  at the end of the struct and of the `DEPTHAI_SERIALIZE*` list, with a
  default value. Append a new `DatatypeEnum` value before `COUNT`.
- A shared-type change also needs a firmware build with the same
  definitions, then a firmware pin bump in `cmake/Depthai/`.
- The Doxygen comment of a public function is its Python docstring.

The full rule set is in the `depthai-cpp-review` skill
(`.agents/skills/depthai-cpp-review/SKILL.md`). Run it on each C++ change
before you open a PR.

## Pull requests

- Branch from `develop` and open the PR against `develop`. `main` gets only
  release merges. Name the branch `fix/...`, `feat/...`, `ci/...`,
  `docs/...` or `test/...`.
- Write the description for a person who did not see your session: what
  was wrong, how you proved it (log, test, reproduction), what the fix
  does, how you tested it. Say if the PR is ready to merge, or what it
  waits for (a firmware build, more test runs, another PR).
- Tests on devices, Windows and macOS run only when the `testable` label
  is **added**. A push to a PR that already has the label runs nothing. To
  test a new push, remove the label and add it again.
- Give each PR exactly one impact label: `No impact`, `Low impact`,
  `Medium impact` or `High Impact`. When you are not sure between two
  labels, use the higher one.

### Impact labels

Ask: how many features use the code that I change? If the diff changes
code that the bug does not touch, explain why in the description, or
use a higher label.

**`No impact`** — the change does not change library behaviour, or it is
clearly correct.
- Examples: CI fix after a runner image change, a missing docstring or
  Python binding, a fix in a beta node, a new or better test.
- Before merge: nothing more.

**`Low impact`** — a bug fix that stays inside the code it changes. This
is the most common label.
- Examples: the root-cause fix for a failing weekly test, a fix in a node
  that few users use, a bug that the `develop` CI can catch.
- Before merge: a clear description. Add `testable` if you are not sure
  that it passes on Windows or macOS. A review must take only a few
  minutes.

**`Medium impact`** — the change can affect code outside the diff.
- Examples: a change in a common utility (matrix math, transformations),
  a change in a popular node, a bug fix with more than one possible
  solution.
- Before merge: the description proves the bug, explains the fix and
  gives the test results. New tests. `testable` passes for the related
  tests. A person who knows the area does a quick check (default:
  Aljaž Konec, GitHub `aljazkonec1`).

**`High Impact`** — a change to core functionality, or to code that many
features depend on.
- Examples: a new node, XLink, the Camera node, calibration,
  ImgTransformations, depth algorithms, RVC2 MDK register changes.
- Before you start: tell Aljaž Konec the issue, how it fits the larger
  picture, and the plan. A new node needs a spec, a behaviour description
  and docs first.
- Before merge: test coverage, many runs on a real device, and a
  discussion for a large change (in person, Slack or a call). `testable`
  is not sufficient: run the nightly, and for wide impact also the weekly
  or BOM workflow.

```bash
gh workflow run hil_testing_nightly.yml --ref <branch>
gh workflow run hil_testing_weekly.yml --ref <branch>
```

## Public repository

This repository is public. Keep private firmware code, internal hostnames,
IP addresses and customer names out of commits, PR text and comments.
