/**
 * Focused Depth demo.
 *
 * Runs an object-detection network on the color camera and feeds the resulting
 * ImgDetections into the Depth node's inputDetections. The Depth node computes a
 * focused depth map (full frame, only the detected regions filled) and a matching
 * confidence map.
 *
 * Works on RVC4 devices. The ImgDetections are automatically remapped to the left
 * stereo frame using the ImgDetections transformation when available.
 */

#include <algorithm>
#include <argparse/argparse.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/CameraSensorType.hpp"
#include "depthai/depthai.hpp"
#include "depthai/modelzoo/Zoo.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/Depth.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"

namespace {

const char* algorithmName(dai::node::Depth::Algorithm algorithm) {
    switch(algorithm) {
        case dai::node::Depth::Algorithm::AUTO:
            return "auto";
        case dai::node::Depth::Algorithm::STEREO:
            return "stereo";
        case dai::node::Depth::Algorithm::NEURAL:
            return "neural";
        case dai::node::Depth::Algorithm::NEURAL_ASSISTED_STEREO:
            return "neural_assisted_stereo";
        case dai::node::Depth::Algorithm::TOF:
            return "tof";
        case dai::node::Depth::Algorithm::GPU_STEREO:
            return "gpu_stereo";
    }
    return "unknown";
}

cv::Mat colorizeDepth(const cv::Mat& frameDepth) {
    if(frameDepth.empty() || frameDepth.channels() != 1) {
        return cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }

    cv::Mat depth32f;
    frameDepth.convertTo(depth32f, CV_32F);

    const cv::Mat nonZeroMask = depth32f != 0.0f;
    const int nz = cv::countNonZero(nonZeroMask);
    if(nz == 0) {
        return cv::Mat::zeros(frameDepth.size(), CV_8UC3);
    }

    std::vector<float> values;
    values.reserve(static_cast<size_t>(nz));
    for(int r = 0; r < depth32f.rows; ++r) {
        const float* d = depth32f.ptr<float>(r);
        const uchar* m = nonZeroMask.ptr<uchar>(r);
        for(int c = 0; c < depth32f.cols; ++c) {
            if(m[c]) {
                values.push_back(d[c]);
            }
        }
    }

    std::sort(values.begin(), values.end());
    auto pct = [&](double p) {
        const size_t idx = static_cast<size_t>(std::round((p / 100.0) * (values.size() - 1)));
        return values[idx];
    };

    const float minDepth = pct(3.0);
    const float maxDepth = pct(95.0);

    cv::Mat logDepth;
    depth32f.copyTo(logDepth);
    logDepth.setTo(minDepth, ~nonZeroMask);
    cv::log(logDepth, logDepth);

    const float logMinDepth = std::log(minDepth);
    const float logMaxDepth = std::log(maxDepth);

    cv::min(logDepth, logMaxDepth, logDepth);
    cv::max(logDepth, logMinDepth, logDepth);
    logDepth = (logDepth - logMinDepth) * (255.0f / (logMaxDepth - logMinDepth));

    cv::Mat depth8U;
    logDepth.convertTo(depth8U, CV_8U);

    cv::Mat depthFrameColor;
    cv::applyColorMap(depth8U, depthFrameColor, cv::COLORMAP_JET);
    depthFrameColor.setTo(cv::Scalar::all(0), ~nonZeroMask);
    return depthFrameColor;
}

cv::Mat colorizeConfidence(const cv::Mat& frame) {
    if(frame.empty() || frame.channels() != 1) {
        return cv::Mat::zeros(frame.size(), CV_8UC3);
    }

    cv::Mat normalized;
    if(frame.depth() == CV_16U) {
        double maxValue = 0.0;
        cv::minMaxLoc(frame, nullptr, &maxValue);
        if(maxValue <= 0.0) {
            return cv::Mat::zeros(frame.size(), CV_8UC3);
        }
        cv::Mat scaled;
        frame.convertTo(scaled, CV_32F, 255.0 / maxValue);
        scaled.convertTo(normalized, CV_8U);
    } else {
        frame.convertTo(normalized, CV_8U);
    }

    cv::Mat colorized;
    cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
    return colorized;
}

}  // namespace

int main(int argc, char** argv) {
    argparse::ArgumentParser program("focused_depth", "1.0.0");
    program.add_description(
        "Focused Depth demo.\n\n"
        "Runs an object-detection network on the color camera and feeds the resulting "
        "ImgDetections into the Depth node's inputDetections. The Depth node computes a "
        "focused depth map (full frame, only the detected regions filled) and a matching "
        "confidence map.");
    program.add_argument("--model").default_value(std::string("yolov6-nano")).help("Detection model description (default: yolov6-nano)");
    program.add_argument("--fps").scan<'g', float>().help("Stereo camera FPS for the Depth node");

    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& err) {
        std::cerr << err.what() << '\n';
        std::cerr << program;
        return EXIT_FAILURE;
    }

    const std::string model = program.get<std::string>("--model");
    std::optional<float> fps;
    if(program.is_used("--fps")) {
        fps = program.get<float>("--fps");
    }

    dai::Pipeline pipeline;
    const auto device = pipeline.getDefaultDevice();
    if(device == nullptr) {
        throw std::runtime_error("No default device available.");
    }

    std::optional<dai::CameraBoardSocket> colorSocket;
    for(const auto& features : device->getConnectedCameraFeatures()) {
        if(std::find(features.supportedTypes.begin(), features.supportedTypes.end(), dai::CameraSensorType::COLOR) != features.supportedTypes.end()) {
            colorSocket = features.socket;
            break;
        }
    }
    if(!colorSocket.has_value()) {
        throw std::runtime_error("No color camera found on this device for object detection.");
    }
    std::cout << "Running detection on color camera socket: " << static_cast<int>(*colorSocket) << std::endl;

    auto camRgb = pipeline.create<dai::node::Camera>();
    camRgb->build(*colorSocket);

    auto detectionNetwork = pipeline.create<dai::node::DetectionNetwork>();
    detectionNetwork->build(camRgb, dai::NNModelDescription{model});
    detectionNetwork->setConfidenceThreshold(0.5f);

    auto depthNode = pipeline.create<dai::node::Depth>();
    if(fps.has_value()) {
        depthNode->build(*fps);
    } else {
        depthNode->build();
    }
    detectionNetwork->out.link(depthNode->inputDetections);

    auto rgbQueue = detectionNetwork->passthrough.createOutputQueue(4, false);
    auto detQueue = detectionNetwork->out.createOutputQueue(4, false);
    auto focusedDepthQueue = depthNode->focusedDepth().createOutputQueue(4, false);
    auto focusedConfQueue = depthNode->focusedConfidence().createOutputQueue(4, false);

    pipeline.build();

    std::cout << "Resolved algorithm: " << algorithmName(depthNode->getResolvedAlgorithm()) << std::endl;
    std::visit(
        [](const auto& value) {
            using T = std::decay_t<decltype(value)>;
            if constexpr(std::is_same_v<T, std::monostate>) {
                std::cout << "Resolved config: none" << std::endl;
            } else {
                std::cout << "Resolved config: " << static_cast<int>(value) << std::endl;
            }
        },
        depthNode->getResolvedConfig());

    pipeline.start();

    while(pipeline.isRunning()) {
        auto frameRgb = rgbQueue->get<dai::ImgFrame>();
        auto detections = detQueue->get<dai::ImgDetections>();
        auto frameFocusedDepth = focusedDepthQueue->get<dai::ImgFrame>();
        auto frameFocusedConf = focusedConfQueue->get<dai::ImgFrame>();

        if(frameRgb != nullptr && detections != nullptr) {
            cv::Mat cvFrame = frameRgb->getCvFrame();
            if(cvFrame.channels() == 1) {
                cv::cvtColor(cvFrame, cvFrame, cv::COLOR_GRAY2BGR);
            }

            for(const auto& det : detections->detections) {
                const auto bbox = det.getOuterBoundingBox();
                const int xmin = static_cast<int>(bbox[0] * cvFrame.cols);
                const int ymin = static_cast<int>(bbox[1] * cvFrame.rows);
                const int xmax = static_cast<int>(bbox[2] * cvFrame.cols);
                const int ymax = static_cast<int>(bbox[3] * cvFrame.rows);
                cv::rectangle(cvFrame, cv::Point(xmin, ymin), cv::Point(xmax, ymax), cv::Scalar(0, 255, 0), 2);
            }
            cv::imshow("rgb", cvFrame);
        }

        if(frameFocusedDepth != nullptr) {
            cv::imshow("focused depth", colorizeDepth(frameFocusedDepth->getFrame()));
        }
        if(frameFocusedConf != nullptr) {
            cv::imshow("focused confidence", colorizeConfidence(frameFocusedConf->getFrame()));
        }

        if(cv::waitKey(1) == 'q') {
            pipeline.stop();
            break;
        }
    }

    cv::destroyAllWindows();
    return EXIT_SUCCESS;
}
