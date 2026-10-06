"""Use child processes so a GIL regression fails with a timeout, not a hung suite."""
import subprocess
import sys
import textwrap

import pytest


@pytest.mark.parametrize("body", [
    '''
    import threading
    import time
    queue = dai.MessageQueue(maxSize=4)
    entered = threading.Event()
    def callback(message):
        entered.set()
        time.sleep(0.05)
    queue.addCallback(callback)
    sender = threading.Thread(target=lambda: queue.send(dai.Buffer()))
    sender.start()
    assert entered.wait(5)
    assert queue.trySend(dai.Buffer())
    sender.join(5)
    assert not sender.is_alive()
    ''',
    '''
    import threading
    class Sink(dai.node.ThreadedHostNode):
        def __init__(self):
            super().__init__()
            self.input = self.createInput("in")
            self.input.setMaxSize(1)
            self.input.setBlocking(True)
            self.done = threading.Event()
        def run(self):
            for _ in range(20):
                self.input.get()
            self.done.set()
    with dai.Pipeline(False) as pipeline:
        sink = pipeline.create(Sink)
        queue = sink.input.createInputQueue(maxSize=1, blocking=True)
        pipeline.start()
        for _ in range(20):
            queue.send(dai.Buffer())
        assert sink.done.wait(5)
        pipeline.stop()
    ''',
])
def test_queue_calls_allow_python_consumer_progress(body):
    subprocess.run([sys.executable, "-c", "import depthai as dai\n" + textwrap.dedent(body)],
                   check=True, capture_output=True, text=True, timeout=15)
