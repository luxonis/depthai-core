# DepthAI Python Library

[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![PyPI](https://img.shields.io/pypi/v/depthai.svg)](https://pypi.org/project/depthai/)
[![Python Wheel CI](https://github.com/luxonis/depthai-core/actions/workflows/python-main.yml/badge.svg)](https://github.com/luxonis/depthai-core/actions/workflows/python-main.yml)

Python bindings for the [DepthAI C++ SDK](https://github.com/luxonis/depthai-core),
for building vision pipelines with Luxonis OAK cameras.

This README describes **DepthAI v3**. For existing v2 applications, use
`depthai<3` and the [v2 documentation](https://docs.luxonis.com/software/depthai/).
See the [v2 to v3 porting guide](https://github.com/luxonis/depthai-core/blob/develop/V2V3PortinGuide.md)
when migrating.

## Documentation and examples

- [DepthAI v3 documentation](https://docs.luxonis.com/software-v3/depthai/)
- [Python examples](https://github.com/luxonis/depthai-core/tree/develop/examples/python)
- [Running the examples](https://github.com/luxonis/depthai-core/blob/develop/examples/README.md)
- [Support forum](https://discuss.luxonis.com/)

## Installation

Install the released package from PyPI in a Python 3.9 or newer environment:

```sh
python3 -m pip install --upgrade pip
python3 -m pip install --upgrade depthai
```

Verify the installed version and discover connected devices:

```sh
python3 -c "import depthai as dai; print(dai.__version__); print(dai.Device.getAllAvailableDevices())"
```

For a development snapshot, select a version from the
[Luxonis snapshot repository](https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/depthai/)
and install that exact version:

```sh
python3 -m pip install --extra-index-url https://artifacts.luxonis.com/artifactory/luxonis-python-snapshot-local/ "depthai==<snapshot-version>"
```

## Building from source

Use CMake 3.20 or newer, a C++17 compiler, Python 3.9 or newer with its development
headers, and the [SDK dependencies](https://github.com/luxonis/depthai-core#dependencies).
The pip build installs its Python build dependencies from `pyproject.toml`.
The first build downloads and builds native dependencies through vcpkg.

From a checkout of the desired v3 branch or tag:

```sh
git submodule update --init --recursive
cd bindings/python
python3 -m pip install .
```

To produce a wheel instead:

```sh
python3 -m pip wheel . --wheel-dir wheelhouse --no-deps
```

For development, build the extension directly from the repository root.
Before configuring, install NumPy for the Python interpreter selected by CMake.
The example below assumes that interpreter is `python3`:

```sh
python3 -m pip install numpy
cmake -S . -B build -DDEPTHAI_BUILD_PYTHON=ON
cmake --build build --parallel 2
```

On Windows with a multi-configuration generator, add `--config Release` to the
build command. To select a different Python interpreter, pass
`-DPYTHON_EXECUTABLE=/full/path/to/python` when configuring and use that interpreter
in place of `python3` in the commands.

The build and test examples use `--parallel 2` to limit compilation to two jobs.
Increase the count if you have sufficient CPU and memory, or use `--parallel 1`
on machines with limited resources.

Point `PYTHONPATH` at the directory containing the built extension. On Linux/macOS:

```sh
export PYTHONPATH="$PWD/build/bindings/python"
python3 -c "import depthai as dai; print(dai.__file__)"
```

Look for `depthai*.pyd` on Windows or `depthai*.so` on Linux/macOS.
For multi-configuration generators, include the configuration subdirectory
(such as `Release`) in `PYTHONPATH`. Printing `dai.__file__` confirms that Python
loaded the intended build.

## Tests and contributing

### Checking new Python bindings without a build

The Python workflow's precheck checks new public C++ declarations only on PR and
PR-review events, before any requested wheel builds. It compares the PR base with
the checkout.

Run the same check locally from the repository root (including uncommitted files):

```sh
python -m pip install -r ci/requirements-binding-check.txt
python ci/check_python_bindings.py --base "$(git merge-base origin/develop HEAD)"
python -m unittest discover -s ci/tests -p 'test_python_bindings.py'
```

The check parses headers under `include/depthai` and binding sources listed in
`bindings/python/CMakeLists.txt`. New public classes, enums, enum values, fields,
constructors and functions in `dai` need a corresponding pybind11 registration.
Private/protected members, anonymous namespaces, `detail`/`internal`/`impl` namespaces
and nested types (such as `PointCloud::Impl`), `DEPTHAI_SERIALIZE*` member-list macros,
and `internal`/`test` header directories are excluded. Existing omissions are not
reported. Adding an overload or changing parameter types requires a change to
that function's binding registration, even if its name is already bound.
Each new overload needs a distinct registration: parameter types from
`py::overload_cast`, `py::init`, and explicit function-pointer `static_cast`s are
matched against the declaration. Disambiguating an old overload does not cover a
new one. Unresolved explicit casts need an exception; wrappers with unknown
signatures count once. Python aliases of the same callable do not count as
additional overloads. An intentional shared dispatcher needs exact exceptions
for the additional overloads it covers.

For an intentional C++-only API or a binding the checker cannot recognize, add the
exact key from the diagnostic to `ci/python_binding_exceptions.json`, with a
nonempty reason, for example:

```json
{
  "dai::Example::nativeHandle()const": "C++ integration only; exposes a native handle."
}
```

Exceptions cover only that declaration, not its whole class or future overloads.
Keys are literal, including `*` in pointer parameter types; wildcard matching is
never performed. Keep exceptions narrow and review them alongside the API change.

The exception list covers deprecated ColorCamera/MonoCamera APIs and properties,
internal pipeline schemas and event aggregation, and CameraControl strobe fields
and nested types pending RVC4 validation. Existing deprecated bindings remain
available. New overloads still require a binding or a reviewed exact exception.

`tests/api_binding_device_test.py` is retained as a TODO for validation on both
RVC2 and RVC4 and HIL integration. It is excluded from the compiled pytest target
and unconditionally skipped during direct pytest runs until that work is done.

This is a source-level guard, not proof of runtime coverage. It recognizes
`py::class_`, `py::enum_`, `ADD_NODE*`, direct member bindings and common lambda
wrappers that access the typed bound instance directly. Enum values are matched
by their C++ target within the owning enum, including unqualified and
namespace-qualified unscoped enumerators, regardless of their Python name.
Signature matching is textual; type aliases and wrapper conversions may need
exceptions. The check
does not expand arbitrary macros, resolve template instantiations, check
aliases/operators, verify callstack registration, or prove runtime overload
coverage. Header access specifiers are tracked separately in each conditional
branch; declarations public in any parsed branch are checked. Newly encountered
syntax that produces parser errors fails the precheck. Build and runtime tests
remain necessary.

### Running the compiled tests

From the repository root, install the Python test dependencies used by CI, then
configure, build, and run the suite:

```sh
python3 -m pip install pytest numpy opencv-python jinja2 websockets
cmake -S . -B build -DDEPTHAI_BUILD_PYTHON=ON -DDEPTHAI_PYTHON_ENABLE_TESTS=ON
cmake --build build --parallel 2 --config Release
cmake --build build --target pytest --parallel 2 --config Release
```

Some tests need an OAK device. For SDK test labels and contribution rules, see
[AGENTS.md](https://github.com/luxonis/depthai-core/blob/develop/AGENTS.md).

For build failures, check that submodules are initialized, the Python headers
match the selected interpreter, and the
[SDK dependencies](https://github.com/luxonis/depthai-core#dependencies) are installed.
