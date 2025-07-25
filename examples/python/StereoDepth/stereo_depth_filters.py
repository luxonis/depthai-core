#!/usr/bin/env python3

import argparse
import random
import cv2
import depthai as dai
import numpy as np

FPS = 20

def getRandomMedianFilterParams():
    return random.choice(
        [
            dai.node.ImageFilters.MedianFilterParams.MEDIAN_OFF,
            dai.node.ImageFilters.MedianFilterParams.KERNEL_3x3,
            dai.node.ImageFilters.MedianFilterParams.KERNEL_5x5,
        ]
    )


def getRandomTemporalFilterParams():
    params = dai.node.ImageFilters.TemporalFilterParams()
    params.enable = random.choice([True, False])
    params.persistencyMode = random.choice(
        [
            dai.filters.params.TemporalFilter.PersistencyMode.PERSISTENCY_OFF,
            dai.filters.params.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_4,
            dai.filters.params.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_2,
            dai.filters.params.TemporalFilter.PersistencyMode.VALID_1_IN_LAST_8,
            dai.filters.params.TemporalFilter.PersistencyMode.PERSISTENCY_INDEFINITELY,
        ]
    )
    params.alpha = random.uniform(0.3, 0.9)
    return params


def getRandomSpeckleFilterParams():
    params = dai.node.ImageFilters.SpeckleFilterParams()
    params.enable = random.choice([True, False])
    return params


def getRandomSpatialFilterParams():
    params = dai.node.ImageFilters.SpatialFilterParams()
    params.enable = random.choice([True, False])
    return params


def main(args: argparse.Namespace):
    # Create pipeline
    with dai.Pipeline() as pipeline:
        monoLeft = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        monoRight = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        outLeft = monoLeft.requestOutput((640, 400), fps=FPS)
        outRight = monoRight.requestOutput((640, 400), fps=FPS)

        depth = pipeline.create(dai.node.StereoDepth)

        filterPipeline = pipeline.create(dai.node.ImageFilters)
        filterFactories = [
            getRandomSpeckleFilterParams,
            getRandomTemporalFilterParams,
            getRandomSpatialFilterParams,
            getRandomMedianFilterParams,
        ]

        filterPipeline.setRunOnHost(True)

        depth.setLeftRightCheck(args.lr_check)
        depth.setExtendedDisparity(args.extended_disparity)
        depth.setSubpixel(args.subpixel)
        depth.inputConfig.setBlocking(False)

        # Linking
        outLeft.link(depth.left)
        outRight.link(depth.right)
        depthQueue = depth.disparity.createOutputQueue()

        filterPipeline.build(depth.disparity)

        ## Create a new filter pipeline
        filterPipeline.initialConfig.filterIndices = []
        filterPipeline.initialConfig.filterParams = [
            filterFactory() for filterFactory in filterFactories
        ]

        configInputQueue = filterPipeline.inputConfig.createInputQueue()
        filterOutputQueue = filterPipeline.output.createOutputQueue()

        pipeline.start()
        import time

        tSwitch = time.time()
        while pipeline.isRunning():
            inDisparity: dai.ImgFrame = (
                depthQueue.get()
            )  # blocking call, will wait until a new data has arrived
            frame = inDisparity.getFrame()
            filterFrame = filterOutputQueue.get()
            filterFrame = (
                filterFrame.getFrame() * (255 / depth.initialConfig.getMaxDisparity())
            ).astype(np.uint8)
            frame = (frame * (255 / depth.initialConfig.getMaxDisparity())).astype(
                np.uint8
            )
            cv2.imshow("disparity", frame)
            frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            cv2.imshow("disparity_color", frame)
            cv2.imshow(
                "filtered_disparity_color",
                cv2.applyColorMap(filterFrame, cv2.COLORMAP_JET),
            )

            ## Update filter pipeline
            if time.time() - tSwitch > 1.0:
                index = random.randint(0, len(filterFactories) - 1)
                new_params = filterFactories[index]()
                config = dai.ImageFiltersConfig().updateFilterAtIndex(
                    index, new_params
                )
                configInputQueue.send(config)
                tSwitch = time.time()
                print(f"Filter at index {index} changed to {new_params}")

            key = cv2.waitKey(1)
            if key == ord("q"):
                break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--extended_disparity", action="store_true", help="Use extended disparity"
    )
    parser.add_argument("--subpixel", action="store_true", help="Use subpixel")
    parser.add_argument("--lr_check", action="store_true", help="Use left-right check")
    args = parser.parse_args()
    main(args)
