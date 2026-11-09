# NNData inside Script

The RVC2 Script interpreter has a smaller API than host Python and RVC4
Script. It does not provide NumPy or `NNData.getTensor()`. Use its typed
layer getters instead. The method names below are case sensitive.

| Operation | RVC2 Script | Host Python / RVC4 Script |
| --- | --- | --- |
| List tensor names | `getAllLayerNames()` | `getAllLayerNames()` |
| Check a name | `hasLayer(name)` | `hasLayer(name)` |
| Read FP16 values | `getLayerFp16(name)` | `getTensor(name)` |
| Read unsigned bytes | `getLayerUInt8(name)` | `getTensor(name)` |
| Read signed 32-bit integers | `getLayerInt32(name)` | `getTensor(name)` |
| Read the first tensor | `getFirstLayerFp16()`, `getFirstLayerUInt8()`, `getFirstLayerInt32()` | Select a name, then `getTensor(name)` |
| Inspect tensor metadata | `getAllLayers()` | `getAllLayers()` |

RVC2 getters return flat Python lists. Choose the getter that matches the
tensor's datatype; these methods interpret the stored bytes rather than
converting arbitrary datatypes. Obtain dimensions, byte strides and datatype
from `getAllLayers()` when shape matters. Check the name before reading it,
and ensure a tensor exists before using a `getFirstLayer*` method.

`getLayerAsFp16()` is not the RVC2 Script method name. Host Python's v3
`getTensor()` returns a NumPy array with shape information.

## Read a tensor

For an FP16 output named `scores`, this Script body works on both platforms:

```python
while True:
    message = node.io["in"].get()
    if not message.hasLayer("scores"):
        node.warn("Missing scores tensor")
        continue
    if hasattr(message, "getTensor"):
        scores = message.getTensor("scores").reshape(-1).tolist()
    else:
        scores = message.getLayerFp16("scores")
    # Use scores here, or forward the original message without copying its data.
    node.io["out"].send(message)
```

Create and link the Script's `in` and `out` ports before starting the
pipeline. Queue reads and sends can block, so consume every connected output
and choose queue sizes for the intended processing rate.

## Create a tensor on RVC2

RVC2 makes `NNData` available directly in Script globals. Its constructor
requires an explicit payload capacity in bytes. A list of floats passed to
`setLayer()` creates an FP16 tensor:

```python
message = NNData(8)  # Four FP16 values, two bytes each.
message.setLayer("scores", [0.25, 0.5, 0.75, 1.0])
node.io["out"].send(message)
```

Allocate enough bytes for all layers. Use a new message when changing tensor
sizes or datatypes: the RVC2 setter's existing-layer path does not resize
the tensor metadata. Use floats explicitly for FP16; integer lists select
the byte-valued setter.

On RVC4, use the regular v3 Python API, for example
`message.addTensor("scores", array, dai.TensorInfo.DataType.FP16)` with a
NumPy array and `import depthai as dai` inside Script.
