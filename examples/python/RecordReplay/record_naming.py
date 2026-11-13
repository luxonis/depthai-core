# Showcases multiple record nodes with different names

import os
from pathlib import Path

import depthai as dai

import example_util as rr

def main():
    pipeline = None
    try:
        pipeline = dai.Pipeline()

        stem = rr.output_dir()
        name = rr.unique_name("naming")
        recorders = []
        for socket in (dai.CameraBoardSocket.CAM_A, dai.CameraBoardSocket.CAM_B):
            cam = pipeline.create(dai.node.Camera).build(socket)
            stream = cam.requestOutput((640, 480), dai.ImgFrame.Type.NV12, dai.ImgResizeMode.CROP, 10)
        
            encoder = pipeline.create(dai.node.VideoEncoder)
            encoder.setProfile(dai.VideoEncoderProperties.Profile.H264_MAIN)
            stream.link(encoder.input)

            record = pipeline.create(dai.node.RecordVideo)
            record.setKeepOnDevice(False)
            record.setFps(10)
            record.setOutputDirectory(stem)
            record.setStreamName(socket.__str__())
            record.setRecordingName(name)
            encoder.out.link(record.input)

            recorders.append(record)
            
        print(f'Saving video to {os.fspath(recorders[0].getOutputDirectory())}')
        return rr.record_for(pipeline, record, 5)
    except Exception as ex:
        return rr.classify_record_failure(str(ex))
    finally:
        record = None
        frames = None
        if pipeline is not None:
            pipeline.stop()


if __name__ == "__main__":
    raise SystemExit(main())
