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

for i in range(500):
    thread = threading.Thread(target=createPipeline)
    thread.start()
