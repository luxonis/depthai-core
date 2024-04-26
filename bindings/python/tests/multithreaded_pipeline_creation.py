import threading
import time
import depthai as dai

class X(dai.node.HostNode):
    def process(self):
        pass

def createPipeline():
    with dai.Pipeline(createImplicitDevice=False) as pipeline:
        X() 
        time.sleep(1) # Yield to other threads
        X()
        assert sum(1 for node in pipeline.getAllNodes() if isinstance(node, dai.node.HostNode)) == 2

threads = [threading.Thread(target=createPipeline) for _ in range(500)]
for thread in threads: thread.start()
for thread in threads: thread.join()
