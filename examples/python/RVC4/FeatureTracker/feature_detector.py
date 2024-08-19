#!/usr/bin/env python3

import depthai as dai
import cv2
import time
import numpy as np
from pathlib import Path

examplePath = str((Path(__file__).parent / Path('../../')).resolve().absolute())
inputImagePath1 = examplePath + "/models/dataset/1/in_left.png"
inputImagePath2 = examplePath + "/models/dataset/1/in_right.png"

# Check if file exists otherwise provoke the user to run `python3 examples/python/install_requirements.py`
if (not Path(inputImagePath1).exists() or not Path(inputImagePath2).exists()):
    import sys
    raise FileNotFoundError(f'Required file/s not found, please run "{sys.executable} {examplePath}/install_requirements.py"')

inputImage1 = cv2.imread(inputImagePath1, cv2.IMREAD_GRAYSCALE)
inputImage2 = cv2.imread(inputImagePath2, cv2.IMREAD_GRAYSCALE)

inputImage2 = cv2.resize(inputImage2, (640, 640), cv2.INTER_CUBIC)

kernel = np.ones((3,3),np.float32)/9
inputImage2 = cv2.filter2D(inputImage2,-1,kernel)


M = np.float32([[1, 0, 25], [0, 1, 50]])
shifted = cv2.warpAffine(inputImage2, M, (inputImage2.shape[1], inputImage2.shape[0]))
inputImage2 = shifted


inputImage1 = cv2.resize(inputImage1, (640, 640), 
               interpolation = cv2.INTER_LINEAR)
inputImage2 = cv2.resize(inputImage2, (640, 640), 
               interpolation = cv2.INTER_LINEAR)

inputImageData1 = (inputImage1).astype('uint8')
inputImageData2 = (inputImage2).astype('uint8')

cv2.imshow("InputImg1", inputImage1)
cv2.imshow("InputImg2", inputImage2)
cv2.waitKey(10)


# create switch for ON/OFF functionality
switchMotion = 'Motion:  0 : OFF \n1 : ON'
switchDescriptorCalculation = 'Descriptor Calculation:  0 : OFF \n1 : ON'

class InputImgNode1(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output = dai.Node.Output(self)
    def run(self):
        while self.isRunning():
            frame = inputImage1

            imgFrame = dai.ImgFrame()
            imgFrame.setData(inputImageData1)
            imgFrame.setWidth(frame.shape[1])
            imgFrame.setHeight(frame.shape[0])
            imgFrame.setType(dai.ImgFrame.Type.GRAY8)
            # Send the message
            self.output.send(imgFrame)
            # Wait for the next frame
            time.sleep(0.1)

class InputImgNode2(dai.node.ThreadedHostNode):
    def __init__(self):
        dai.node.ThreadedHostNode.__init__(self)
        self.output = dai.Node.Output(self)
    def run(self):
        while self.isRunning():
            frame = inputImage2

            imgFrame = dai.ImgFrame()
            imgFrame.setData(inputImageData2)
            imgFrame.setWidth(frame.shape[1])
            imgFrame.setHeight(frame.shape[0])
            imgFrame.setType(dai.ImgFrame.Type.GRAY8)
            # Send the message
            self.output.send(imgFrame)
            # Wait for the next frame
            time.sleep(0.1)

def drawFeatures(frame, features):
        featureArray = ()
        descriptorArray = ()

        pointColor = (0, 0, 255)
        greenColor = (0, 255, 0)
        circleRadius = 2
        for feature in features:
            keyPoint = cv2.KeyPoint()
            keyPoint.pt = (feature.position.x, feature.position.y)

            featureArray += (keyPoint,)

            keyDescriptor = feature.descriptor

            descriptorArray += (keyDescriptor,)

            if feature.age > 2:
                cv2.circle(frame, (int(feature.position.x), int(feature.position.y)), circleRadius, pointColor, -1, cv2.LINE_AA, 0)
            else:
                cv2.circle(frame, (int(feature.position.x), int(feature.position.y)), circleRadius, greenColor, -1, cv2.LINE_AA, 0)
        
        return [featureArray, descriptorArray]

def on_trackbar1(val):
    cfg1 = dai.FeatureTrackerConfig()
    cornerDetector1 = dai.FeatureTrackerConfig.CornerDetector()
    cornerDetector1.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features1')
    cornerDetector1.robustness = cv2.getTrackbarPos('robustness', 'Features1')

    thresholds1 = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
    thresholds1.initialValue = cv2.getTrackbarPos('harris_score','Features1')
    cornerDetector1.thresholds = thresholds1

    motionEstimator1.enable = cv2.getTrackbarPos(switchMotion, 'Features1')
    cornerDetector1.enableDescriptorCalculation = cv2.getTrackbarPos(switchDescriptorCalculation, 'Features1')


    cfg1.setCornerDetector(cornerDetector1)
    cfg1.setMotionEstimator(motionEstimator1)

    inputConfigQueue1.send(cfg1)

def on_trackbar2(val):
    cfg2 = dai.FeatureTrackerConfig()
    cornerDetector2 = dai.FeatureTrackerConfig.CornerDetector()
    cornerDetector2.numMaxFeatures = cv2.getTrackbarPos('numMaxFeatures', 'Features2')
    cornerDetector2.robustness = cv2.getTrackbarPos('robustness', 'Features2')

    thresholds2 = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
    thresholds2.initialValue = cv2.getTrackbarPos('harris_score','Features2')
    cornerDetector2.thresholds = thresholds2

    motionEstimator2.enable = cv2.getTrackbarPos(switchMotion, 'Features2')
    cornerDetector2.enableDescriptorCalculation = cv2.getTrackbarPos(switchDescriptorCalculation, 'Features2')

    cfg2.setCornerDetector(cornerDetector2)
    cfg2.setMotionEstimator(motionEstimator2)

    inputConfigQueue2.send(cfg2)

cv2.namedWindow('Features1')
cv2.namedWindow('Features2')

# create trackbars threshold and robustness change
cv2.createTrackbar('harris_score','Features1',2000,25000, on_trackbar1)
cv2.createTrackbar('harris_score','Features2',2000,25000, on_trackbar2)

cv2.createTrackbar('numMaxFeatures','Features1',256,1024, on_trackbar1)
cv2.createTrackbar('numMaxFeatures','Features2',256,1024, on_trackbar2)

cv2.createTrackbar('robustness','Features1',0, 127, on_trackbar1)
cv2.createTrackbar('robustness','Features2',0,127, on_trackbar2)

cv2.createTrackbar(switchMotion, 'Features1',0,1,on_trackbar1)
cv2.createTrackbar(switchMotion, 'Features2',0,1,on_trackbar2)

cv2.createTrackbar(switchDescriptorCalculation, 'Features1',1,1,on_trackbar1)
cv2.createTrackbar(switchDescriptorCalculation, 'Features2',1,1,on_trackbar2)


# Create pipeline

with dai.Pipeline() as pipeline:
    InputImgNode1 = pipeline.create(InputImgNode1)
    InputImgNode2 = pipeline.create(InputImgNode2)

    featureTracker1 = pipeline.create(dai.node.FeatureTracker)
    featureTracker2 = pipeline.create(dai.node.FeatureTracker)
    
    featureTracker1.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)
    featureTracker2.initialConfig.setCornerDetector(dai.FeatureTrackerConfig.CornerDetector.Type.HARRIS)

    featureTracker1.initialConfig.setMotionEstimator(False)
    featureTracker2.initialConfig.setMotionEstimator(False)

    motionEstimator1 = dai.FeatureTrackerConfig.MotionEstimator()
    motionEstimator2 = dai.FeatureTrackerConfig.MotionEstimator()

    motionEstimator1.enable = False
    motionEstimator2.enable = False

    featureTracker1.initialConfig.setMotionEstimator(motionEstimator1)
    featureTracker2.initialConfig.setMotionEstimator(motionEstimator2)

    cornerDetector1 = dai.FeatureTrackerConfig.CornerDetector()
    cornerDetector2 = dai.FeatureTrackerConfig.CornerDetector()

    cornerDetector1.numMaxFeatures = 256
    cornerDetector2.numMaxFeatures = 256

    outputFeaturePassthroughQueue1 = featureTracker1.passthroughInputImage.createOutputQueue()
    outputFeaturePassthroughQueue2 = featureTracker2.passthroughInputImage.createOutputQueue()

    outputFeatureQueue1 = featureTracker1.outputFeatures.createOutputQueue()
    outputFeatureQueue2 = featureTracker2.outputFeatures.createOutputQueue()

    InputImgNode1.output.link(featureTracker1.inputImage)
    InputImgNode2.output.link(featureTracker2.inputImage)

    inputConfigQueue1 = featureTracker1.inputConfig.createInputQueue()
    inputConfigQueue2 = featureTracker2.inputConfig.createInputQueue()

    thresholds1 = dai.FeatureTrackerConfig.CornerDetector.Thresholds()
    thresholds2 = dai.FeatureTrackerConfig.CornerDetector.Thresholds()

    thresholds1.initialValue = cv2.getTrackbarPos('harris_score','Features1')
    thresholds2.initialValue = cv2.getTrackbarPos('harris_score','Features2')

    cornerDetector1.thresholds = thresholds1
    cornerDetector2.thresholds = thresholds2

    featureTracker1.initialConfig.setCornerDetector(cornerDetector1)
    featureTracker2.initialConfig.setCornerDetector(cornerDetector2)

    # Create a Brute Force Matcher object.
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck = True)

    pipeline.start()
    while pipeline.isRunning():
        trackedFeatures1 = outputFeatureQueue1.get().trackedFeatures
        trackedFeatures2 = outputFeatureQueue2.get().trackedFeatures

        outputPassthroughImage1 : dai.ImgFrame = outputFeaturePassthroughQueue1.get()
        outputPassthroughImage2 : dai.ImgFrame = outputFeaturePassthroughQueue2.get()

        passthroughImage1 = outputPassthroughImage1.getCvFrame()
        passthroughImage2 = outputPassthroughImage2.getCvFrame()

        passthroughImage1 = cv2.cvtColor(passthroughImage1, cv2.COLOR_GRAY2BGR)
        passthroughImage2 = cv2.cvtColor(passthroughImage2, cv2.COLOR_GRAY2BGR)

        [keyPoints1, keyDescriptors1] = drawFeatures(passthroughImage1, trackedFeatures1)
        [keyPoints2, keyDescriptors2] = drawFeatures(passthroughImage2, trackedFeatures2)

        train_descriptorNp = np.asarray(keyDescriptors1).reshape((-1, 32))
        test_descriptorNp = np.asarray(keyDescriptors2).reshape(-1, 32)

        train_descriptorNp = (train_descriptorNp).astype('uint8')
        test_descriptorNp = (test_descriptorNp).astype('uint8')

        train_keypoints = keyPoints1
        test_keypoints = keyPoints2

        if (cv2.getTrackbarPos(switchDescriptorCalculation, 'Features1') == 1 and cv2.getTrackbarPos(switchDescriptorCalculation, 'Features2') == 1):
            if (test_descriptorNp.size <= train_descriptorNp.size):
                print ('correct size of descriptors')
            else:
                raise Exception("Test image should not have more descriptors than train image") 
            # Perform the matching between the descriptors of the training image and the test image
            matches = bf.match(train_descriptorNp, test_descriptorNp)

            matches = sorted(matches, key = lambda x : x.distance)
            closeMatches = matches[:20]


            print (train_keypoints[closeMatches[0].queryIdx].pt[0])
            print (train_keypoints[closeMatches[0].queryIdx].pt[1])
            print (test_keypoints[closeMatches[0].trainIdx].pt[0])
            print (test_keypoints[closeMatches[0].trainIdx].pt[1])



            result = cv2.drawMatches(inputImage1, train_keypoints, inputImage2, test_keypoints, closeMatches, inputImage2, flags = 2)

            # Display the best matching points
            cv2.imshow('Result', result)


        # Show the frame
        cv2.imshow("Features1", passthroughImage1)
        cv2.imshow("Features2", passthroughImage2)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
