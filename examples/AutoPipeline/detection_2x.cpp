#include <chrono>
#include <iostream>
#include <iterator>
#include <stdexcept>

// Includes common necessary includes for development using depthai library
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/depthai.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/properties/ColorCameraProperties.hpp"

/*
The code is the same as for Tiny-yolo-V3, the only difference is the blob file.
The blob was compiled following this tutorial: https://github.com/TNTWEN/OpenVINO-YOLOV4
*/

// TODO fix so all resolutions work
struct Res {
    int32_t width;
    int32_t height;
};
static const std::vector<Res> notWorkingResolutions = {
    {.width = 1352, .height = 1012}  // dai::ColorCameraProperties::SensorResolution::THE_1352X1012
};

dai::ColorCameraProperties::SensorResolution resolutionFromSensorConfig(const dai::CameraSensorConfig& config) {
    const auto width = config.width;
    const auto height = config.height;
    using R = dai::ColorCameraProperties::SensorResolution;
    if(width == 1920 && height == 1080) {
        return R::THE_1080_P;
    } else if(width == 3840 && height == 2160) {
        return R::THE_4_K;
    } else if(width == 4056 && height == 3040) {
        return R::THE_12_MP;
    } else if(width == 4208 && height == 3120) {
        return R::THE_13_MP;
    } else if(width == 1280 && height == 720) {
        return R::THE_720_P;
    } else if(width == 1280 && height == 800) {
        return R::THE_800_P;
    } else if(width == 1920 && height == 1200) {
        return R::THE_1200_P;
    } else if(width == 2592 && height == 1944) {
        return R::THE_5_MP;
    } else if(width == 4000 && height == 3000) {
        return R::THE_4000X3000;
    } else if(width == 5312 && height == 6000) {
        return R::THE_5312X6000;
    } else if(width == 8000 && height == 6000) {
        return R::THE_48_MP;
    } else if(width == 240 && height == 180) {
        return R::THE_240X180;
    } else if(width == 1280 && height == 962) {
        return R::THE_1280X962;
    } else if(width == 2000 && height == 1500) {
        return R::THE_2000X1500;
    } else if(width == 2028 && height == 1520) {
        return R::THE_2028X1520;
    } else if(width == 2104 && height == 1560) {
        return R::THE_2104X1560;
    } else if(width == 1440 && height == 1080) {
        return R::THE_1440X1080;
    } else if(width == 1352 && height == 1012) {
        return R::THE_1352X1012;
    } else if(width == 2024 && height == 1520) {
        return R::THE_2024X1520;
    } else {
        throw std::runtime_error("Unknown resolution " + std::to_string(width) + "x" + std::to_string(height) + " requested");
    }
}

dai::CameraSensorConfig getClosestCameraConfig(dai::Device& device, int64_t width, int64_t height) {
    const auto& cameraFeatures = device.getConnectedCameraFeatures();
    const auto& camera = std::find_if(
        cameraFeatures.begin(), cameraFeatures.end(), [](const dai::CameraFeatures& itr) -> bool { return itr.socket == dai::CameraBoardSocket::CAM_A; });
    if(camera == cameraFeatures.end()) {
        throw std::runtime_error("Device doesn't support ColorCamera");
    }
    std::vector<dai::CameraSensorConfig> colorCameraModes;
    std::copy_if(camera->configs.begin(), camera->configs.end(), std::back_inserter(colorCameraModes), [](const auto& itr) {
        return itr.type == dai::CameraSensorType::COLOR;
    });
    int64_t minAdditionalPixels = -1;
    ssize_t foundIndex = -1;
    ssize_t index = 0;
    for(const auto& mode : colorCameraModes) {
        if(mode.width >= width && mode.height >= height) {
            if(std::find_if(notWorkingResolutions.begin(),
                            notWorkingResolutions.end(),
                            [&mode = std::as_const(mode)](const auto& itr) { return itr.width == mode.width && itr.height == mode.height; })
               != notWorkingResolutions.end()) {
                std::cout << "Warning: ignoring possible best resolution " << mode.width << "x" << mode.height << " because of possible firmware bugs"
                          << std::endl;
            } else {
                int64_t additionalPixels = (mode.width - width) * mode.height + (mode.height - height) * mode.width;
                if(minAdditionalPixels == -1 || additionalPixels < minAdditionalPixels) {
                    foundIndex = index;
                    minAdditionalPixels = additionalPixels;
                }
            }
        }
        ++index;
    }
    if(minAdditionalPixels == -1 || foundIndex == -1) {
        throw std::runtime_error("This camera can't provide the wanted resolution" + std::to_string(width) + "x" + std::to_string(height));
    }
    return colorCameraModes[foundIndex];
}

static const std::vector<std::string> labelMap = {
    "person",        "bicycle",      "car",           "motorbike",     "aeroplane",   "bus",         "train",       "truck",        "boat",
    "traffic light", "fire hydrant", "stop sign",     "parking meter", "bench",       "bird",        "cat",         "dog",          "horse",
    "sheep",         "cow",          "elephant",      "bear",          "zebra",       "giraffe",     "backpack",    "umbrella",     "handbag",
    "tie",           "suitcase",     "frisbee",       "skis",          "snowboard",   "sports ball", "kite",        "baseball bat", "baseball glove",
    "skateboard",    "surfboard",    "tennis racket", "bottle",        "wine glass",  "cup",         "fork",        "knife",        "spoon",
    "bowl",          "banana",       "apple",         "sandwich",      "orange",      "broccoli",    "carrot",      "hot dog",      "pizza",
    "donut",         "cake",         "chair",         "sofa",          "pottedplant", "bed",         "diningtable", "toilet",       "tvmonitor",
    "laptop",        "mouse",        "remote",        "keyboard",      "cell phone",  "microwave",   "oven",        "toaster",      "sink",
    "refrigerator",  "book",         "clock",         "vase",          "scissors",    "teddy bear",  "hair drier",  "toothbrush"};

int main(int argc, char** argv) {  // NOLINT
    using namespace std;           // NOLINT
    using namespace std::chrono;   // NOLINT

    std::string nnArchivePath(NN_ARCHIVE_PATH);
    std::string nnArchivePath2(NN_ARCHIVE_PATH_2);

    std::cout << "Using archive at path: " << nnArchivePath << "\n";
    std::cout << "Using archive 2 at path: " << nnArchivePath2 << "\n";

    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();

    camRgb->setInterleaved(false);                                       // NOLINT
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);  // NOLINT
    camRgb->setFps(40);

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    const dai::NNArchive archive(nnArchivePath);
    const dai::NNArchive archive2(nnArchivePath2);
    const auto& config = archive.getConfig().getConfigV1();
    const auto& config2 = archive2.getConfig().getConfigV1();
    if(!config || !config2) {
        throw std::runtime_error("Wrong config version");
    }
    const auto width = (*config).model.inputs[0].shape[2];
    const auto height = (*config).model.inputs[0].shape[3];
    const auto width2 = (*config2).model.inputs[0].shape[2];
    const auto height2 = (*config2).model.inputs[0].shape[3];

    const auto device = pipeline.getDefaultDevice();
    if(!device) {
        throw std::runtime_error("The pipeline was without device");
    }
    const auto& mode = getClosestCameraConfig(*device, width, height);
    std::cout << "FOUND CLOSEST RESOLUTION: " << mode.width << "x" << mode.height << std::endl;
    std::cout << "SETTING PREVIEW SIZE TO: " << width << "x" << height << std::endl;
    camRgb->setResolution(resolutionFromSensorConfig(mode));
    camRgb->setPreviewSize(static_cast<int>(width), static_cast<int>(height));
    detectionNetwork->setNNArchive(archive);

    detectionNetwork->setNumInferenceThreads(2);
    detectionNetwork->input.setBlocking(false);

    // Linking
    camRgb->preview.link(detectionNetwork->input);
    const auto queueFrames = camRgb->preview.getQueue();
    auto detectionQueue = detectionNetwork->out.getQueue();

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    const auto& qRgb = queueFrames;
    const auto& qDet = detectionQueue;
    pipeline.start();

    cv::Mat frame;
    std::vector<dai::ImgDetection> detections;
    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color2 = cv::Scalar(255, 255, 255);

    // Add bounding boxes and text to the frame and show it to the user
    auto displayFrame = [](const std::string& name, cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        auto color = cv::Scalar(255, 0, 0);
        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;  // NOLINT
            int y1 = detection.ymin * frame.rows;  // NOLINT
            int x2 = detection.xmax * frame.cols;  // NOLINT
            int y2 = detection.ymax * frame.rows;  // NOLINT

            uint32_t labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow(name, frame);
    };

    while(true) {
        const auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        const auto inDet = qDet->tryGet<dai::ImgDetections>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = static_cast<float>(counter) / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        if(inRgb) {
            frame = inRgb->getCvFrame();
            std::stringstream fpsStr;
            fpsStr << "NN fps: " << std::fixed << std::setprecision(2) << fps;
            cv::putText(frame, fpsStr.str(), cv::Point(2, static_cast<int>(inRgb->getHeight()) - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color2);
        }

        if(inDet) {
            detections = inDet->detections;
        }

        if(!frame.empty()) {
            displayFrame("rgb", frame, detections);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            break;
        }
    }
    pipeline.stop();
    pipeline.wait();
    return 0;
}
