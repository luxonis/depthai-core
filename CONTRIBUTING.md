# Contributing to DepthAI Core

Thanks for taking the time to improve DepthAI Core. This repository contains the C++ DepthAI library, Python bindings, examples, tests, and shared data used by Luxonis devices. Keep changes focused, test the behavior you touch, and prefer the existing repository patterns over new conventions.

The `develop` branch is the active branch for DepthAI v3 work. For v2 changes, use the `v2_develop` branch.

## Before Opening an Issue

For bugs:

- Search existing GitHub issues and the Luxonis forum first.
- Check the troubleshooting notes in the documentation and in [README.md](./README.md).
- Include a minimal reproducible example.
- Include device model, connection type, platform, operating system, DepthAI version or commit, and relevant logs.
- For pipeline problems, include a pipeline graph when possible.

For feature requests:

- Start with the use case and why it matters.
- Describe the behavior you want, not only a possible implementation.
- Mention affected device platforms, such as RVC2 or RVC4, if known.

## Development Setup

Required tools:

- CMake 3.20 or newer.
- A C++17 compiler.
- Git with submodule support.
- Python 3 for scripts, tests, and bindings.
- `libudev-dev` on Debian-based Linux systems.

Initialize submodules before building:

```bash
git submodule update --init --recursive
```

The normal local C++ build is:

```bash
cmake -S . -B build
cmake --build build --parallel
```

For tests and examples:

```bash
cmake -S . -B build \
    -DDEPTHAI_BUILD_TESTS=ON \
    -DDEPTHAI_BUILD_EXAMPLES=ON \
    -DDEPTHAI_TEST_EXAMPLES=ON
cmake --build build --parallel
```

On Windows, pass `--config Release` or the configuration you built when running CMake build and test commands.

## CMake Options

Keep optional feature changes explicit. Common options include:

- `DEPTHAI_BUILD_TESTS=ON` to build C++ tests.
- `DEPTHAI_BUILD_EXAMPLES=ON` to build examples.
- `DEPTHAI_BUILD_PYTHON=ON` to build Python bindings.
- `DEPTHAI_BUILD_DOCS=ON` to build Doxygen documentation.
- `DEPTHAI_OPENCV_SUPPORT=ON|OFF` for OpenCV-dependent functionality.
- `DEPTHAI_PCL_SUPPORT=ON|OFF` for point cloud functionality.
- `DEPTHAI_BASALT_SUPPORT=ON|OFF` and `DEPTHAI_RTABMAP_SUPPORT=ON|OFF` for VSLAM-related functionality.
- `DEPTHAI_CLANG_TIDY=ON` to run clang-tidy during compilation.
- `DEPTHAI_SANITIZE=ON` to enable address and undefined behavior sanitizers.

See [cmake/depthaiOptions.cmake](./cmake/depthaiOptions.cmake) for the complete option list.

## Code Style

### C++

- Use C++17.
- Follow the existing API style and naming in nearby files.
- Use camelCase for all functions, variables, and members both in Python and C++.
- Keep public headers self-contained and avoid unnecessary includes.
- Prefer RAII and standard library types over manual resource management.
- Do not add broad abstractions unless they reduce real duplication or clarify an existing ownership boundary.
- Avoid unrelated formatting, renames, and refactors in feature or bug-fix pull requests.

C++ formatting is enforced with clang-format. The repository uses [.clang-format](./.clang-format), currently based on Google style with local overrides. Prefer clang-format 18:

```bash
python -m pip install "clang-format~=18.3"
cmake -S . -B build -DDEPTHAI_CLANG_FORMAT=ON
cmake --build build --target clangformat
```

CI checks formatting with:

```bash
bash ci/check_format.sh build
```
and uses clang-format version that ships with Ubuntu 24.04.

clang-tidy is available but not part of every local workflow:

```bash
cmake -S . -B build -DDEPTHAI_CLANG_TIDY=ON
cmake --build build --parallel 8
```

### Python

- Keep examples simple and directly runnable.
- Prefer explicit pipeline setup over clever helper code in examples.
- Use APIs as users are expected to use them.
- Preserve Python compatibility expected by the bindings CI unless the change intentionally updates it.
- If testing against a local Python binding build, set `PYTHONPATH` to the built module directory:

```bash
cmake -S . -B build -DDEPTHAI_BUILD_PYTHON=ON
cmake --build build --parallel 8 --target depthai
export PYTHONPATH="$PWD/build/bindings/python"
python3 -c "import depthai as dai; print(dai.__file__)"
```

## Tests

Run the smallest useful test set while iterating, then run the broader set that matches your change before opening a pull request.

Host-only and hardware tests are separated by CTest labels. Common labels include:

- `onhost` for host-only tests.
- `rvc2` for RVC2 device tests.
- `rvc4` for RVC4 device tests.
- `usb` for USB-specific tests.
- `poe` for PoE/TCP tests.

Examples:

```bash
cd build
ctest -L onhost --output-on-failure --no-tests=error
ctest -L rvc2 --output-on-failure --no-tests=error
ctest -L rvc4 --output-on-failure --no-tests=error
```

The CI-oriented test wrapper is:

```bash
python3 tests/run_tests.py --test_dir build --rvc2
python3 tests/run_tests.py --test_dir build --rvc4
python3 tests/run_tests.py --test_dir build --rvc4usb
python3 tests/run_tests.py --test_dir build --rvc4rgb
```

Hardware tests require matching Luxonis devices and may use environment variables such as `DEPTHAI_PLATFORM` and `DEPTHAI_PROTOCOL`. If you cannot run hardware tests locally, say so in the pull request and describe what you did run.

Python binding tests can be built and run with:

```bash
cmake -S . -B build \
    -DDEPTHAI_BUILD_PYTHON=ON \
    -DDEPTHAI_PYTHON_ENABLE_TESTS=ON \
    -DDEPTHAI_PYTHON_ENABLE_EXAMPLES=ON \
    -DDEPTHAI_PYTHON_TEST_EXAMPLES=ON
cmake --build build --parallel
cd build
ctest --output-on-failure --no-tests=error
```

Some Python CI jobs also build the `pytest` CMake target:

```bash
cmake --build build --target pytest --config Release
```

## Generated and Consistency-Checked Files

Some files have consistency checks or generation scripts. If you touch these areas, run the matching checks:

```bash
bash ci/check_datatype_enum_consistency.sh
bash ci/check_protobuf_consistency.sh
```


## Examples

Examples are part of the user-facing API surface. When adding or changing examples:

- Put C++ examples under [examples/cpp](./examples/cpp) and Python examples under [examples/python](./examples/python).
- Prefer clear names and minimal setup.
- Avoid hidden local file dependencies unless the example explicitly demonstrates record/replay or file input.
- Keep C++ and Python versions aligned when both exist for the same feature.

Build C++ examples with:

```bash
cmake -S . -B build -DDEPTHAI_BUILD_EXAMPLES=ON
cmake --build build --parallel
```

Install Python example requirements with:

```bash
python3 examples/python/install_requirements.py
```

## Pull Request Guidelines

Before opening a pull request:
- Target branch should be `develop` for v3 work or `v2_develop` for v2 work.
- Rebase or merge the current target branch.
- Keep the pull request focused on one feature, fix, or cleanup.
- Include tests for behavior changes, especially public API behavior, serialization, device communication, and pipeline graph changes.
- Run formatting and the relevant build/test commands.
- Mention any tests you could not run and why.
- Include screenshots, logs, or pipeline graphs when they make review easier.
- Update examples and documentation when user-visible behavior changes.

Pull request descriptions should include:

- What changed.
- Why it changed.
- How it was tested.
- Any compatibility concerns, migrations, or platform limitations.

## Compatibility Rules

DepthAI Core is consumed from C++, Python and other downstream projects. Treat public API changes carefully:

- Avoid breaking C++ public headers unless the migration is intentional.
- Keep Python binding behavior aligned with C++ behavior.
- Preserve serialization and replay compatibility unless the change explicitly updates the format.
- Consider RVC2 and RVC4 separately; support for one platform does not imply support for the other.
- Do not change default environment-variable behavior without documenting the impact.
- Do not silently change units, coordinate systems, timestamp behavior, or stream names.

When a breaking change is necessary, document the old behavior, new behavior, migration path, and affected platforms.

## Commit Hygiene

- Use clear commit messages that describe the change.
- Do not include build directories, local caches, temporary recordings, device dumps, or generated artifacts that are not meant to be checked in.
