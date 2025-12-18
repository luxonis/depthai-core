
#!/usr/bin/env python3
import depthai as dai
from pathlib import Path
from argparse import ArgumentParser

scriptDir = Path(__file__).resolve().parent
examplesRoot = (scriptDir / Path('../')).resolve()  # This resolves the parent directory correctly
models = examplesRoot / 'models'
videoPath = models / 'construction_vest.mp4'

parser = ArgumentParser()
parser.add_argument("--webSocketPort", type=int, default=8765)
parser.add_argument("--httpPort", type=int, default=8082)
parser.add_argument("-i", "--inputVideo", default=videoPath, help="Input video name")
args = parser.parse_args()

remoteConnector = dai.RemoteConnection(webSocketPort=args.webSocketPort, httpPort=args.httpPort)
# Create pipeline
with dai.Pipeline() as pipeline:
    replay = pipeline.create(dai.node.ReplayVideo)
    replay.setReplayVideoFile(Path(args.inputVideo))
    detectionNetwork = pipeline.create(dai.node.DetectionNetwork).build(
        replay, dai.NNModelDescription("yolov6-nano")
    )

    remoteConnector.addTopic("detections", detectionNetwork.out, "img")
    remoteConnector.addTopic("images", replay.out, "img")

    pipeline.start()
    remoteConnector.registerPipeline(pipeline)

    while pipeline.isRunning():
        key = remoteConnector.waitKey(1)
        if key == ord("q"):
            print("Got q key from the remote connection!")
            break
