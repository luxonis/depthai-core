# Beta namespace

The `beta` namespace is a staging area for experimental DepthAI features. It
allows new features to be developed and iterated on quickly before they are
promoted to the main `depthai` namespace.

Beta features are well-developed, but minor API and behavioral changes may occur between DepthAI releases without notice.

## Usage

In C++, beta nodes are available under `dai::beta::node`:

```cpp
auto node = pipeline.create<dai::beta::node::ImgDetectionsFilter>();
```

In Python, they are available under `dai.beta.node`:

```python
node = pipeline.create(dai.beta.node.ImgDetectionsFilter)
```

## Device support

On-device execution of Beta nodes is supported only on RVC4. If running Beta nodes on RVC2, DepthAI
automatically configures beta nodes to run on the host.
