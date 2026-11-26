"""A C++ constructor must retain the Python part of a shared_ptr argument."""

import gc
import weakref

import depthai as dai
from depthai_pybind11_tests.node_holder import BaseNodeOwner, NodeOwner


def test_super_constructor_keeps_last_python_reference():
    class Node(dai.node.ThreadedHostNode):
        def __init__(self):
            super().__init__()
            self.calls = []

        def run(self):
            self.calls.append("run")

        def onStart(self):
            self.calls.append("start")

    class Owner(NodeOwner):
        def __init__(self, node):
            super().__init__(node)

    pipeline = dai.Pipeline(False)
    node = pipeline.create(Node)
    ref = weakref.ref(node)
    owner = Owner(node)
    second = NodeOwner(node)  # Exercise repeated independent shared_ptr casts.
    del node, pipeline
    gc.collect()
    assert ref() is not None, "C++ shared_ptr lost the Python object"
    owner.on_start()
    owner.run()
    assert owner.get() is ref()
    assert ref().calls == ["start", "run"]
    del owner
    gc.collect()
    assert ref() is not None, "Another C++ owner is still alive"
    second.run()
    assert ref().calls == ["start", "run", "run"]
    del second
    gc.collect()
    assert ref() is None, "Last C++ owner failed to release the Python object"


def test_python_host_node_can_be_removed_via_base_node_shared_ptr():
    class Node(dai.node.ThreadedHostNode):
        def run(self):
            pass

    pipeline = dai.Pipeline(False)
    node = pipeline.create(Node)
    # remove takes shared_ptr<dai::Node>, rather than ThreadedHostNode.
    # This must work as well as the more derived constructor argument above.
    pipeline.remove(node)
    assert node not in pipeline.getAllNodes()


def test_super_constructor_accepts_base_node_shared_ptr():
    class Node(dai.node.ThreadedHostNode):
        def run(self):
            pass

    class Owner(BaseNodeOwner):
        def __init__(self, node):
            super().__init__(node)

    pipeline = dai.Pipeline(False)
    node = pipeline.create(Node)
    ref = weakref.ref(node)
    owner = Owner(node)
    del node, pipeline
    gc.collect()
    assert ref() is not None
    assert owner.get() is ref()
    del owner
    gc.collect()
    assert ref() is None


def test_shared_from_this_preserves_python_virtual_dispatch_and_final_release():
    calls = []

    class Node(dai.node.ThreadedHostNode):
        def __init__(self):
            super().__init__()
            self.marker = "Python state survived"

        def run(self):
            calls.append(self.marker)

    pipeline = dai.Pipeline(False)
    node = pipeline.create(Node)
    ref = weakref.ref(node)
    owner = NodeOwner(node)
    original_owner = owner.from_shared_from_this()
    del node, owner, pipeline
    gc.collect()
    # Only a C++ pointer obtained through shared_from_this now survives.
    # Invoke the native virtual even if the wrapper died: on the original
    # binary this throws instead of reaching the existing Python override.
    original_owner.run()
    assert calls == ["Python state survived"]
    assert ref() is not None
    assert original_owner.shares_shared_from_this_owner()
    del original_owner
    gc.collect()
    assert ref() is None, "shared_from_this ownership introduced a leak"


def test_host_node_shared_from_this_preserves_process_callback():
    class Node(dai.node.HostNode):
        def __init__(self):
            super().__init__()
            # Registration must finish during the base initializer, before
            # Python creates subnodes or otherwise uses the parent pipeline.
            assert self in self.getParentPipeline().getAllNodes()
            self.marker = 73

        def process(self):
            result = dai.NNData()
            result.setSequenceNum(self.marker)
            return result

    pipeline = dai.Pipeline(False)
    node = pipeline.create(Node)
    ref = weakref.ref(node)
    owner = NodeOwner(node).from_shared_from_this()
    assert owner.shares_shared_from_this_owner()
    del node, pipeline
    gc.collect()
    assert owner.process_group().getSequenceNum() == 73
    assert ref() is not None
    del owner
    gc.collect()
    assert ref() is None
