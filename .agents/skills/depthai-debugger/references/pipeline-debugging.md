# Pipeline debugging

Use for stalls, missing messages, latency and incorrect graph placement.
Instrumentation can alter timing; keep an uninstrumented baseline.

## Schema and placement

Capture startup with `DEPTHAI_LEVEL=debug`; locate schema and node IDs. Compare
requested links/configuration with generated bridges and host/device placement.
Node groups can contain independently tracked nodes. For C++, inspect
`getPipelineSchema(dai::SerializationType::JSON, false)`; check binding
signatures before translating to Python.

The optional [pipeline graph tool](https://docs.luxonis.com/software-v3/depthai/tools/pipeline-graph/)
visualizes schemas/logs. Keep the underlying artifact for headless analysis.
Check installed tool help/API compatibility before using older examples.

## State snapshots

Enable instrumentation before pipeline build/start:

```python
pipeline.enablePipelineDebugging()
# Configure nodes, links, and all intended consumers before starting.
pipeline.start()
state = pipeline.getPipelineState().nodes().detailed()
for node_id, node_state in state.nodeStates.items():
    print(node_id, node_state)
```

Use `nodes(node_id)`, `summary()`, `inputs()`, `outputs()`, and
`otherTimings()` to narrow collection. Capture multiple timestamped snapshots
alongside sequence progression. If requesting state hangs, investigate host/device
threads and transport rather than issuing more blocking requests.

For automatic periodic state logs in a fresh process:

```bash
DEPTHAI_PIPELINE_DEBUGGING=1 DEPTHAI_LEVEL=trace python3 -u /path/to/repro.py
```

Programmatic queries do not require trace printing. The checked revision rejects
pipeline debugging for multi-device pipelines. Use per-stream counters,
timestamps, logs and separate reproductions instead.

## Interpret states with the graph

| Evidence | Interpretation to test |
| --- | --- |
| Input WAITING / empty | Stopped producer, wrong link/config, or legitimate dependency wait |
| Input BLOCKED / full | Upstream sender cannot complete; inspect this node's processing/consumers |
| Output remains SENDING | Check every downstream branch for a full blocking input |
| Growing occupancy and latency | Consumer slower than producer; measure both |
| Pool exhaustion with short queues | Messages retained by callbacks/application or other branches |
| Sync waits while streams run | Check clock domain, matching policy, missing sequence and thresholds |
| Config-dependent node has no output | Check config delivery, wait-for-config and reuse behavior |
| Invalid/absent timing | Unsupported instrumentation or too few events; not measured zero FPS |

A full queue can be the consequence of a blockage farther downstream. Trace all
consumers. An output queue created but never drained can block the graph.
Nonblocking queues can drop data: switching them changes semantics and is an
experiment, not proof of a fix.

## Events and platform differences

Events track loops, input/output operations, blocks and custom timings. The
aggregator batches them; snapshots are not complete instruction traces.

Direct subscriptions through `node.pipelineEventOutput.createOutputQueue()`
are documented for host/RVC4, with RVC2 limitations. RVC2 input/output timing
coverage exceeds its loop/block coverage. Check `isValid()` and the relevant
platform test before interpreting statistics.

For custom host nodes use existing dispatcher/block-event examples to instrument
the suspected interval. Preserve rate/queue settings and compare with
instrumentation disabled afterward.

## Minimal stall reproduction

Keep the original configuration and target. Remove UI, network sinks and unrelated
branches in separate experiments. Drain required outputs and record sequence,
receive time and size without expensive rendering. Replace a live source with
deterministic input when it preserves the failure. If removing a branch helps,
restore it and vary consumer behavior to establish the causal dependency.

Retain schema, snapshots, stacks, consumer behavior and counters. For resource
effects continue in [performance](performance-and-resources.md).

Sources: [design](../../../../PipelineDebugging.md),
[state example](../../../../examples/python/Misc/PipelineDebugging/get_pipeline_state.py),
[event example](../../../../examples/python/Misc/PipelineDebugging/node_pipeline_events.py),
[RVC2 tests](../../../../tests/src/ondevice_tests/pipeline_debugging_rvc2_test.cpp),
[RVC4 tests](../../../../tests/src/ondevice_tests/pipeline_debugging_rvc4_test.cpp),
[host tests](../../../../tests/src/onhost_tests/pipeline_debugging_host_test.cpp).
