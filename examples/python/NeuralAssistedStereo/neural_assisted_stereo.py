import cv2 as cv
import depthai as dai

FPS = 20

if __name__ == "__main__":
    device = dai.Device()
    pipeline = dai.Pipeline(device)
    if not device.isNeuralDepthSupported():
        print("Exiting NeuralAssistedStereo example: device doesn't support NeuralDepth.")
        exit()

    monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B, sensorFps=FPS)
    monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C, sensorFps=FPS)

    monoLeftOut = monoLeft.requestFullResolutionOutput()
    monoRightOut = monoRight.requestFullResolutionOutput()

    neuralAssistedStereo = pipeline.create(dai.node.NeuralAssistedStereo).build(monoLeftOut, monoRightOut, neuralModel=dai.DeviceModelZoo.NEURAL_DEPTH_NANO)

    disparityQueue = neuralAssistedStereo.disparity.createOutputQueue()

    with pipeline:
        pipeline.start()
        while pipeline.isRunning():
            disparity = disparityQueue.get()
            cv.imshow("Depth", dai.colorizeDepthFrame(disparity, 500, 12000, cv.COLORMAP_TURBO, False).getCvFrame())

            key = cv.waitKey(1)
            if key == ord('q'):
                quit()
