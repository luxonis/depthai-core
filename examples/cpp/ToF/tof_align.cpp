#include <argparse/argparse.hpp>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "depthai/depthai.hpp"

constexpr float FPS = 30.0f;
const cv::Size CAMERA_SIZE(640, 400);

constexpr float MIN_DEPTH = 100.0f;
constexpr float MAX_DEPTH = 7000.0f;
float rgbWeight = 0.5f;
float depthWeight = 0.5f;

void updateBlendWeights(int percentRgb, void*) {
    rgbWeight = static_cast<float>(percentRgb) / 100.0f;
    depthWeight = 1.0f - rgbWeight;
}

int main(int argc, char** argv) {
    argparse::ArgumentParser program("tof_align");
    program.add_description("Align ToF depth over left or right camera and show a blended overlay.");
    program.add_argument("--camera")
        .default_value(std::string("left"))
        .choices("left", "right")
        .help("Camera to align depth onto: left=CAM_B, right=CAM_C (default: left)");

    try {
        program.parse_args(argc, argv);
    } catch(const std::runtime_error& err) {
        std::cerr << err.what() << '\n';
        std::cerr << program;
        return EXIT_FAILURE;
    }

    const std::string cameraArg = program.get<std::string>("--camera");
    const dai::CameraBoardSocket alignSocket = (cameraArg == "right") ? dai::CameraBoardSocket::CAM_C : dai::CameraBoardSocket::CAM_B;
    std::cout << "Aligning ToF depth over " << cameraArg << " camera\n";

    dai::Pipeline pipeline;

    auto tof = pipeline.create<dai::node::ToF>();
    tof->build(dai::CameraBoardSocket::AUTO, dai::ToFConfig::Profile::MID_RANGE, FPS);

    auto cam = pipeline.create<dai::node::Camera>()->build(alignSocket);
    auto camOut = cam->requestOutput(std::make_pair(CAMERA_SIZE.width, CAMERA_SIZE.height), std::nullopt, dai::ImgResizeMode::CROP, FPS, true);

    auto align = pipeline.create<dai::node::ImageAlign>();
    align->setRunOnHost(true);
    tof->depth.link(align->input);
    camOut->link(align->inputAlignTo);

    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(0.5 / FPS)));
    sync->setRunOnHost(true);
    camOut->link(sync->inputs["rgb"]);
    align->outputAligned.link(sync->inputs["depth_aligned"]);
    sync->inputs["rgb"].setBlocking(false);

    auto syncQueue = sync->out.createOutputQueue();

    const std::string windowBlend = "tof-overlay-" + cameraArg;
    const std::string windowDepth = "depth-aligned";

    pipeline.start();
    cv::namedWindow(windowBlend);
    cv::namedWindow(windowDepth);
    cv::createTrackbar("RGB Weight %", windowBlend, nullptr, 100, updateBlendWeights);
    cv::setTrackbarPos("RGB Weight %", windowBlend, static_cast<int>(rgbWeight * 100));

    while(pipeline.isRunning()) {
        auto messageGroup = syncQueue->get<dai::MessageGroup>();
        if(messageGroup == nullptr) continue;

        auto frameRgb = messageGroup->get<dai::ImgFrame>("rgb");
        auto frameDepth = messageGroup->get<dai::ImgFrame>("depth_aligned");

        cv::Mat cvFrame = frameRgb->getCvFrame();
        if(cvFrame.channels() == 1) {
            cv::cvtColor(cvFrame, cvFrame, cv::COLOR_GRAY2BGR);
        }

        cv::Mat depthColorized = dai::utility::colorizeDepthFrame(*frameDepth, MIN_DEPTH, MAX_DEPTH, cv::COLORMAP_JET, true).getCvFrame();
        if(depthColorized.size() != cvFrame.size()) {
            cv::resize(depthColorized, depthColorized, cvFrame.size());
        }

        cv::imshow(windowDepth, depthColorized);

        cv::Mat blended;
        cv::addWeighted(cvFrame, rgbWeight, depthColorized, depthWeight, 0, blended);
        cv::imshow(windowBlend, blended);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
