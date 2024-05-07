import depthai as dai
import cv2


class HostDisplay(dai.node.HostNode):
    def build(self, frameOutput: dai.Node.Output):
        self.link_args(frameOutput) # Has to match the inputs to the `process` method

        # This sends all the processing to the pipeline where it's executed by the `pipeline.runTasks()` or implicitly by `pipeline.run()` method.
        # It's needed as the GUI window needs to be updated in the main thread, and the `process` method is by default called in a separate thread.
        self.sendProcessingToPipeline(True)
        return self

    def process(self, message: dai.ImgFrame):
        cv2.imshow("HostDisplay", message.getCvFrame())
        key = cv2.waitKey(1)
        if key == ord('q'):
            print("Detected 'q' - stopping the pipeline...")
            self.stopPipeline()

with dai.Pipeline() as p:
    camera = p.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)

    hostDisplay = p.create(HostDisplay).build(camera.video)

    p.run() # Will block until the pipeline is stopped by someone else (in this case it's the display node)