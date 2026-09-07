---
name: depthai-cpp-review
description: Reviews C++ changes against this project's rules plus modern C++ Core Guidelines — memory safety, concurrency, security, performance, and idiom. Use for any C++ code review, before committing or merging C++ changes, and whenever the user asks to check, review, or audit .cpp/.hpp/.cc/.h files. Also use proactively after writing or refactoring C++ in this repo.
context: fork
agent: general-purpose
argument-hint: "[optional: path, commit range, or PR number]"
---

# C++ Review

Review C++ changes and report issues by severity. Project rules below take
precedence over the general guidelines when the two conflict.

This skill covers two repositories that are one product:

| Repo                 | Role                                       | Remote                                                 | Integration branch |
| -------------------- | ------------------------------------------ | ------------------------------------------------------ | ------------------ |
| `depthai-core`       | Host SDK. Public C++/Python API.           | `github.com/luxonis/depthai-core`                      | `develop`          |
| `depthai-device-kb`  | Device firmware for RVC4.                  |                                                        | `develop`          |

`depthai-device-kb` holds `depthai-core` as a submodule at
`external/depthai-core`. Both repos compile the same public headers. Find
out which repo you are in before you apply a rule as some rules apply to one
repo only.

The SDK is the primary way to use Luxonis RVC2 (OAK 1 and OAK 2) and RVC4
(OAK 4) hardware. Tests must use real hardware when the feature needs it.

## Scope

If `$ARGUMENTS` names a path, commit range, or PR, review that. Otherwise:

```bash
git diff --stat -- '*.cpp' '*.hpp' '*.cc' '*.hh' '*.cxx' '*.h'
git diff -- '*.cpp' '*.hpp' '*.cc' '*.hh' '*.cxx' '*.h'
```

Review only modified files and their immediate call sites. Do not audit the
whole repo. Never review vendored or submodule trees:
`3rdparty/`, `shared/`, `include/3rdparty/`, `bindings/python/external/`,
`external/` (device-kb), `vcpkg/`, and any `build*/` directory.

## Run static analysis first

Run whichever are available; skip silently if not installed. Both repos share
an identical `.clang-format`. **Use clang-format-18 and clang-tidy-18
only** as CI pins those versions.

```bash
cmake -S . -B build -DDEPTHAI_CLANG_FORMAT=ON
cmake --build build --target clangformat
git --no-pager diff          # any diff here is a CI failure

# tidy (depthai-core)
cmake -S . -B build -DDEPTHAI_CLANG_TIDY=ON -DCLANG_TIDY_BIN=/usr/bin/clang-tidy-18

# consistency gates (depthai-core only  - these are hard CI gates)
bash ci/check_datatype_enum_consistency.sh
bash ci/check_protobuf_consistency.sh
```

Report analyser findings alongside your own. Do not simply relay the tool
output and flag which findings are real and which are noise in this
codebase. The "Known exceptions" list at the end of Part 1 tells you what is
noise here.

---

# PART 1 — PROJECT RULES (highest priority)

## A. Cross-repo contract

- **Wire-format and ABI compatibility across host and device.**
  `DEPTHAI_SERIALIZE` / `DEPTHAI_SERIALIZE_EXT` field lists the
  `DatatypeEnum` order, and every `Properties` struct are the serialised
  contract between the host SDK and device firmware. Both sides compile the
  same header, but a host and a device of different versions talk to each
  other in the field.

  Flag any of these in a shared type
  (`include/depthai/pipeline/datatype/`, `include/depthai/properties/`,
  `include/depthai/common/`):
  - a field reordered, renamed, retyped, or removed from a
    `DEPTHAI_SERIALIZE*` list;
  - a `DatatypeEnum` enumerator inserted in the middle, reordered, or
    removed. New values append at the end, before `COUNT`;
  - a new field with no default value.

  Correct pattern: append the field at the end of the struct **and** at the
  end of the `DEPTHAI_SERIALIZE*` list, with a default initialiser.
  Severity: CRITICAL

## B. Backward compatibility

- **Every change must be backward compatible.**
  A new parameter needs a new overload, an optional, or a default value.
  Severity: CRITICAL

- **Removal and renaming go through deprecation.**
  Keep the old symbol and mark it
  `[[deprecated("Use <replacement> instead")]]`. The message must name the
  replacement — that is the house pattern
  (`XLinkConnection::getMxId`, `ColorCamera::setCamId`, `PointCloudData::isSparse`).
  Deleting a public symbol outright is a break.
  Severity: CRITICAL

## C. Build system

- **C++17 is the hard target.**
  Flag all features that use C++20 or later.
  Severity: HIGH

- **Formatting is enforced by CI, not by opinion.**
  `ci/check_format.sh <builddir>` fails on any diff after
  `--target clangformat`. Do not hand-format; do not fight the tool.
  Severity: MEDIUM

## D. Adding a datatype (depthai-core)

Two CI scripts encode this as a hard gate. A new or changed message must
satisfy all of it. Use the scripts as the checklist:

`ci/check_datatype_enum_consistency.sh` requires:
1. The class lives in `include/depthai/pipeline/datatype/<Name>.hpp` and
   inherits `Buffer`.
2. `getDatatype()` returns `DatatypeEnum::<Name>`, and `<Name>` exists in
   `include/depthai/pipeline/datatype/DatatypeEnum.hpp` - appended, never
   inserted.
3. `<Name>` appears as a key **and** as a child entry in the hierarchy map in
   `src/pipeline/datatype/DatatypeEnum.cpp`.
4. `src/pipeline/datatype/StreamMessageParser.cpp` has a
   `case DatatypeEnum::<Name>: return parseDatatype<Name>(...)`.
5. `serialize(std::vector<std::uint8_t>&, DatatypeEnum&) const override` is
   implemented, and the class ends with
   `DEPTHAI_SERIALIZE(<Name>, Buffer::sequenceNum, Buffer::ts, Buffer::tsDevice, Buffer::tsSystem, <fields...>);`

`ci/check_protobuf_consistency.sh` requires, if the message is protobuf
serialisable:

6. A `.proto` in `protos/`, registered in `protos/CMakeLists.txt`.
7. `serializeProto` / `deserializeProto` / `serializeSchema` overrides inside
   `#ifdef DEPTHAI_ENABLE_PROTOBUF`, and the class also inherits
   `ProtoSerializable`.
8. `getProtoMessage(const <Name>*)` declared in `src/utility/ProtoSerialize.hpp`
   **and** defined in `src/utility/ProtoSerialize.cpp` — the two sets must
   match exactly.
9. If deserialisation is supported: `setProtoMessage(<Name>&)` in both files,
   a `schemaNameToDatatype` mapping, and the enum listed in
   `deserializationSupported`.
Severity: HIGH

## E. Tests

- **Every new node, message, feature, and bugfix needs a test.**
  The test must exercise the real behaviour, not just construct the object.
  Severity: HIGH

- **Serialization roundtrip**
  Any change to a type deriving from `include/depthai/pipeline/datatype/Buffer.hpp` 
  needs a serialization roundtrip test that runs on both RVC2 and RVC4.**
  Severity: HIGH

## F. Python bindings (depthai-core only)

- **All public C++ API must be exposed to Python.** `depthai-device-kb` has
  no bindings, so this rule does not apply there.

  A new public class needs
  `bindings/python/src/<mirrored path>/<Name>Bindings.cpp` with:
  - `void bind_<name>(pybind11::module& m, void* pCallstack)`,
  - all `py::class_` / `py::enum_` declarations first, then the
    callstack push/pop block, then the actual `.def(...)` bindings — this
    two-phase order is what makes forward references work;
  - `DOC(dai, <Name>)` on every class and method;
  - `PYBIND11_MAKE_OPAQUE(std::vector<dai::T>)` plus
    `py::bind_vector` and `py::implicitly_convertible<py::list, std::vector<T>>()`
    for any new vector type crossing the boundary.

  Register the file in `bindings/python/CMakeLists.txt` and push its `bind`
  into the callstack.
  Severity: HIGH

- **Doxygen comments are the Python docstrings.**
  `DOC(dai, X)` is generated from the header comment by `pybind11_mkdoc`. A
  missing or stale Doxygen comment silently ships an empty or wrong Python
  docstring. Every public function needs one:
  ```cpp
  /**
   * Set edges connections between keypoints.
   * @param edges Vector edges connections represented as pairs of keypoint indices.
   * @note This is only applicable if keypoints decoding is enabled.
   */
  void setKeypointEdges(const std::vector<dai::Edge>& edges);
  ```
  Severity: MEDIUM - raise to HIGH when the symbol is bound to Python.

## G. Examples (depthai-core only)

- **C++ and Python examples are mirrored.**
  `examples/cpp/<Category>/<snake_case>.cpp` must have a matching
  `examples/python/<Category>/<snake_case>.py`. A new C++ example without its
  Python twin is a finding, and the same the other way round. Register the
  C++ one with `dai_add_example`.
  Severity: MEDIUM

## H. Node structure

- **Host-side node (`depthai-core`).**
  ```cpp
  class ImageAlign : public DeviceNodeCRTP<DeviceNode, ImageAlign, ImageAlignProperties>
     public:
      constexpr static const char* NAME = "ImageAlign";
      using DeviceNodeCRTP::DeviceNodeCRTP;
     protected:
      Properties& getProperties() override;
  ```
  `Input` and `Output` members are declared inline with their queue config
  and accepted datatypes:
  `Input input{*this, {"input", DEFAULT_GROUP, false, 4, {{DatatypeEnum::ImgFrame, false}}}};`
  A pure host node derives from `CustomThreadedNode<T>` / `ThreadedHostNode`.

- **Device-side node (`depthai-device-kb`).**
  Lives in `namespace dai { namespace gate {`, derives the core
  `dai::node::<X>`, and overrides `run()`, `buildStage1()`, and
  `runOnHost()`. Its `NAME` aliases the core node's `NAME`. A device node
  must also appear in `src/pipeline/PipelineBuilder.cpp`, otherwise the
  pipeline cannot instantiate it.

- **Run loops use `while(mainLoop())`, not `while(isRunning())`.**
  `mainLoop()` also drives pipeline debugging and state reporting.
  Wrap a blocking input read in `auto blockEvent = this->inputBlockEvent();`
  and a blocking send in `outputBlockEvent()`, so the queue-blocking stats in
  `PipelineDebugging.md` stay correct.
  Severity: MEDIUM

- **Setters return the node by reference for chaining**
  (`ImageAlign& setNumShaves(int numShaves);`) and write into `properties`.
  Severity: LOW

## I. Reuse existing code

New features and nodes must reuse what exists. Search before you add a
helper — `include/depthai/utility/` already holds `matrixOps.hpp`,
`span.hpp`, `Serialization.hpp`, `ImageManipImpl.hpp`, `LockingQueue.hpp`,
`CircularBuffer.hpp`, `Pimpl.hpp`, `RecordReplay.hpp`, `Clock.hpp`,
`Memory.hpp`, and more; `include/depthai/common/` holds the shared value
types. Cite the existing equivalent if one exists.

For a genuinely new helper, judge where it belongs: node-agnostic logic, or
logic with a second caller in sight, belongs in `utility/` or `common/`;
anything tied to a node's internals stays local to that node. Do not promote
speculatively. Near-duplicate blocks across nodes are candidates for a common
version.
Severity: MEDIUM

## J. Naming and style

House style, confirmed across both repos. This overrides the general
guidelines in Part 2 where the two disagree.

| Kind                                   | Convention    | Example                                |
| -------------------------------------- | ------------- | -------------------------------------- |
| Class, struct, enum, and its file name | `PascalCase`  | `ImgFrame`, `ImageAlignProperties.hpp` |
| Function, method, variable, member     | `camelCase`   | `setNumShaves`, `alignWidth`           |
| Enumerator                             | `ALL_CAPS`    | `CameraBoardSocket::CAM_A`             |
| Compile-time constant                  | `ALL_CAPS`    | `DEFAULT_QUEUE_SIZE`                   |
| Namespace                              | lowercase     | `dai`, `dai::node`, `dai::gate`        |
Check every modified file, examples included.
Severity: MEDIUM

# PART 2 — GENERAL C++ STANDARDS

Derived from the C++ Core Guidelines. Rule IDs (R.11, ES.20, …) refer to
<https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines> — cite them in
findings so the author can look up the rationale.

## CRITICAL — Memory safety

- Raw `new`/`delete` instead of `unique_ptr`/`shared_ptr` or RAII (R.11, R.20)
- `malloc`/`free` in C++ code (R.10)
- Buffer overflows: C arrays, `strcpy`, `sprintf` without bounds (SL.con.1)
- Use-after-free: dangling pointers, invalidated iterators
- Uninitialized variables read before assignment (ES.20)
- Missing null check before pointer dereference
- Returning a pointer or reference to a local (F.43)
- Resources not tied to object lifetime (P.8, E.6)

## CRITICAL — Security

- Command injection via unvalidated input to `system()` / `popen()`
- Format string attacks: user input as a `printf` or `fmt` format
- Unchecked integer arithmetic on untrusted input (ES.46, ES.100)
- Hardcoded secrets, API keys, passwords in source
- `reinterpret_cast` without documented justification (ES.48)
- Casting away `const` (ES.50)

## HIGH — Concurrency

- Data races: shared mutable state without synchronisation (CP.2, CP.3)
- Deadlocks: multiple mutexes taken in inconsistent order — use
  `std::scoped_lock` (CP.21)
- Manual `lock()`/`unlock()` instead of RAII guards (CP.20)
- Unnamed lock guards — `std::lock_guard<std::mutex>(m);` destroys immediately
  (CP.44)
- Detached threads without lifetime management (CP.26)
- Waiting on a condition variable without a predicate (CP.42)
- Calling unknown code (callbacks) while holding a lock (CP.22)
- `volatile` used for synchronisation (CP.8)
- Node-specific: blocking on an input queue outside a `mainLoop()` iteration,
  or holding a lock across a queue `get()` — the node cannot then be stopped.

## HIGH — Class design and code quality

- Rule of Five violated: some special members defined, others missing (C.21)
- Special members defined where Rule of Zero would do (C.20)
- Base class destructor neither public-virtual nor protected-non-virtual (C.35)
- Single-argument constructor not `explicit` (C.46)
- Virtual functions without exactly one of `virtual`/`override`/`final` (C.128)
- Virtual calls in constructors or destructors (C.82)
- `memset`/`memcpy` on non-trivial types (C.90)
- Missing `const` correctness on methods, parameters, references (Con.2, Con.3)
- Functions over ~50 lines or doing more than one thing (F.2, F.3)
- Nesting deeper than 4 levels
- C-style code: `typedef` over `using` (T.43), C arrays, C-style casts (ES.48)
- Non-const globals (I.2)
- Ownership transferred by raw pointer or reference (I.11, R.3)

## MEDIUM — Performance

- Large objects passed by value where `const&` is right (F.16)
- Missing `std::move` on sink parameters
- Missing `reserve()` on known-size containers
- String concatenation in loops
- Compile-time-computable work left at runtime (Per.11)
- Pointer-chasing layouts where contiguous storage would do (Per.19)
- Optimisation claims without measurement (Per.6) — flag both unmeasured
  optimisation and premature complexity
- Copying an `ImgFrame` payload where a `span` or shared `Memory` would do

## MEDIUM — Idiom and hygiene

- Not `const`/`constexpr` by default (Con.1, ES.25)
- `0`/`NULL` instead of `nullptr` (ES.47)
- Narrowing or signed/unsigned mixed arithmetic (ES.46, ES.100)
- `using namespace` at global scope in a header (SF.7)
- Headers not self-contained (SF.8)
- `std::endl` instead of `'\n'` (SL.io.50)
- Output parameters where a returned struct is clearer (F.20, F.21)
- Exceptions: built-in types thrown, caught by value, empty catch blocks,
  exceptions used for flow control (E.14, E.15, E.3)

---

# Output format

````
# C++ Review

## Scope
<repo, files reviewed, commit range or PR>

## Static analysis
clang-format-18:      <clean | n files would change>
clang-tidy-18:        <n> findings (<n> actionable)
datatype consistency: <PASS | FAIL | n/a>
protobuf consistency: <PASS | FAIL | n/a>
build:                <clean | n warnings>

## Findings

### [CRITICAL] <one-line title>  (PROJECT: <rule name> | <Core Guideline ID>)
`path/to/file.cpp:45`

<one or two sentences: what's wrong and what goes wrong because of it>

Current:
```cpp
<minimal excerpt>
```

Suggested:
```cpp
<the fix>
```

### [HIGH] ...
### [MEDIUM] ...
### [INFO] ...

## Checked and clean
<one line each: what you verified and found correct>

## Summary
CRITICAL: n   HIGH: n   MEDIUM: n   INFO: n
Verdict: APPROVE | WARN | BLOCK
````

## Verdict rules

| Verdict     | Condition                                    |
| ----------- | -------------------------------------------- |
| **APPROVE** | No CRITICAL or HIGH findings                 |
| **WARN**    | MEDIUM findings only — merge with judgement  |
| **BLOCK**   | Any CRITICAL or HIGH finding                 |

Any violation of a Part 1 project rule marked CRITICAL blocks on its own,
regardless of what Part 2 turns up.

## Reviewing discipline

- Every finding needs a file and line. No findings without a location.
- One finding per problem. Don't restate the same issue per call site — cite
  the first and note the count.
- Suggest a fix, don't just name the rule.
- Say what you checked and found clean, briefly. A review that only lists
  problems hides its own coverage gaps.
- If a change is correct but the surrounding design makes it fragile, say so
  under INFO rather than inflating the severity.
- Don't flag anything listed under "Known exceptions".
- State which repo you reviewed. A rule from the wrong repo is a false
  positive.
