# Regressions, evidence and skill validation

## Verify the diagnosis

Use the smallest reproduction that retains the failure. Record the hypothesis,
single changed variable, expected observation and result. Retest the original
case with ordinary logging/watchdogs afterward. For intermittent failures,
report run counts/durations and failure rates; one passing run is weak evidence.

Prefer existing tests under `tests/src/onhost_tests` and
`tests/src/ondevice_tests`. Discover built tests without configuring a new build:

```bash
ctest --test-dir /path/to/existing/build -N
ctest --test-dir /path/to/existing/build --print-labels
```

Select exact names/labels from that build. Device tests need the intended target
and can exercise hardware, crash it or write configuration; inspect the selected
test first. Existing artifacts may be stale relative to sources.

For a source fix follow repository behavioral-test requirements, shared-type
roundtrips, C++ review and consistency checks as applicable. If builds are reserved
for the user, give targets/configuration and required artifacts instead of
building. Do not trigger HIL, workflows, PR updates or external messages solely
because a runbook mentions them.

For regression isolation compare one compatible host/firmware/OS/model revision
at a time. Use isolated checkouts and preserved artifacts, never discard user
changes. A source bisect is useful only if the tested binary follows each step.
For deterministic behavior compare payloads; for quality changes use ground truth
and include invalid outputs; for performance keep device/workload/window fixed.

## Evidence and handoff

Keep artifacts outside the repository, in a new session directory. Preserve
original dumps, logs, context, reproduction, schema, timestamps, counters and
symbol identities. Distinguish device uptime, host monotonic time and wall time.
Record unavailable/failed collection separately from successful empty output.

A useful findings report contains:

- Failure and exact reproduction, including target and actual artifact identities.
- First relevant error and pointers to supporting logs/state/stacks/payloads.
- Experiments, changed variables, results and whether they reproduced the original.
- Cause or ranked hypothesis with evidence and remaining uncertainty.
- Verification completed, untested conditions and required next artifact/action.
- Cleanup/restoration performed and anything intentionally left running.

Review shareable copies for credentials, device addresses, customer imagery,
model data and private source paths. Raw cores contain process memory; redacting
text logs does not sanitize the core.

## Validate this skill without builds or devices

The standard-library tests exercise helper behavior with synthetic artifacts and
short local subprocesses:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tests/skills/depthai_debugger -p 'test_*.py'
```

If the skill-creator validator is installed, run its `quick_validate.py` on this
skill directory. Check relative Markdown links and metadata too. These checks do
not establish hardware procedure correctness.

For behavioral evaluation, use clean agent sessions with a realistic request and
minimum sanitized artifacts. Supply the skill and inputs, keeping the expected
diagnosis out of the evaluating agent's prompt. Use an isolated output directory
and explicitly prohibit builds/device access unless that evaluation authorizes
them. Use independent agents only when delegation is permitted.

| Evaluation input | Expected diagnostic distinction |
| --- | --- |
| Old wheel shadows edited checkout | Verify actual import before firmware speculation |
| Native loader mismatch | Inspect loaded modules/ABI before rebuilding |
| Undrained fan-out output | Trace backpressure to responsible consumer |
| Pools exhausted but queues short | Check held references and other branches |
| Missing RVC2 loop timings | Recognize unsupported data instead of zero activity |
| Multi-device instrumentation rejected | Choose logs/counters or smaller reproductions |
| High internal FPS, low arrival FPS | Separate producer, transport and host consumer |
| Incorrect image/tensor output | Check layout/preprocessing and intermediate data |
| Override set but stale package | Verify runtime selection and packaged executable |
| RVC2 report with unrelated ELF | Refuse confident symbolization |
| Nested RVC4 dump | Identify SDK envelope versus actual core archive |
| Gate crash with delayed artifact | Preserve session and bound retrieval |
| Debugger causes termination | Distinguish watchdog/supervision layers |
| Busy device | Establish owner/session before cleanup |
| Missing core | Check collection, limits, handlers, storage and readiness |
| Replay passes, camera run fails | Retain acquisition/timing/hardware hypotheses |
| Missing private repo or symbols | Produce useful public evidence and precise handoff |
| Build prohibited or no selected device | Continue independent analysis; respect constraints |

Grade decisions and evidence, not exact wording. Correct platform/phase selection,
supported controls, target/artifact identity, preserved evidence, bounded recovery
and honest limitations matter more than number of commands executed.

## Freshness and coverage

For a changed SDK/firmware/OS, review sources referenced by the affected procedure:
logging/selection, pipeline limitations, archive formats, Gate lifecycle,
watchdogs, debugger launch and toolchain/sysroot. Symbol existence alone does not
prove unchanged semantics. Resolve documentation discrepancies with the
implementation while following active execution instructions.

Use these validation statuses in investigations: source-checked, host-exercised,
RVC2-exercised, RVC4-exercised, documented/unverified, or unavailable. The skill's
initial hardware procedures are source/documentation checked, not device-tested.
Vendor profiling, board probes and recovery remain capability-dependent.

Representative device validation spans RVC2 USB/PoE, RVC4 USB/Ethernet/local,
host-only, C++/Python and relevant Linux/macOS/Windows hosts. Run only the portions
needed for the change and authorized hardware. Public fixtures must be synthetic
or deliberately sanitized; do not commit private crash artifacts.

Sources: [test registration](../../../../tests/CMakeLists.txt),
[host tests](../../../../tests/src/onhost_tests),
[device tests](../../../../tests/src/ondevice_tests),
[CI workflows](../../../../.github/workflows).
