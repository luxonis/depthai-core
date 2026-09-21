DepthAI Core is the C++ SDK for Luxonis cameras, with Python bindings
through pybind11. It builds on Linux, macOS, and Windows (MSVC).

## Build & Test & Style
Follow instructions in the main README.md

## Layout
- `include/depthai/` — public headers
- `src/` — implementation
- `bindings/python/` — pybind11 bindings; a new public API needs a binding
- `examples/` — C++ and Python examples
- `shared/`, `3rdparty/` — submodules; do not edit
- `protos/` — schemas; run `ci/check_protobuf_consistency.sh` after a change

## Skills
A depthai specific review skill is avaialble at `.agents/skills/depthai-cpp-review/SKILL.md`