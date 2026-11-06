/**
 * Minimal GPUStereo example — disparity from a stereo camera pair (RVC4 only).
 */
#include <depthai/depthai.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

int main() {
    std::shared_ptr<dai::Device> device = std::make_shared<dai::Device>();

    if(!device->isGpuStereoSupported()) {
        std::cout << "Exiting GPUStereo example: GPUStereo is not supported on this device.\n";
        return 0;
    }

    dai::Pipeline pipeline(device);
    auto camLeft = pipeline.create<dai::node::Camera>();
    auto camRight = pipeline.create<dai::node::Camera>();
    camLeft->build(dai::CameraBoardSocket::CAM_B);
    camRight->build(dai::CameraBoardSocket::CAM_C);

    auto gpu = pipeline.create<dai::node::StereoDepth>();
    // Supported input sizes: 1280x800 and 640x400 (rectified inside the node).
    gpu->build(*camLeft->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8), *camRight->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8));
    // Confidence 0..255, higher = more confident; pixels below the threshold are invalidated (0 = off).
    gpu->initialConfig->setConfidenceThreshold(10);
    // AUTO defaults to GPU_CPU on supported devices; GPU keeps matching entirely on the GPU.
    gpu->setBackend(dai::node::StereoDepth::Backend::GPU_CPU);
    gpu->initialConfig->postProcessing.temporalFilter.enable = true;

    auto dispQ = gpu->disparity.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto frame = dispQ->get<dai::ImgFrame>();
        cv::Mat disp(frame->getHeight(), frame->getWidth(), CV_16UC1, frame->getData().data());
        cv::Mat dispF;
        disp.convertTo(dispF, CV_32F);
        cv::Mat disp8;
        cv::normalize(dispF, disp8, 0, 255, cv::NORM_MINMAX, CV_8U);
        cv::Mat dispColor;
        cv::applyColorMap(disp8, dispColor, cv::COLORMAP_JET);
        dispColor.setTo(cv::Scalar(0, 0, 0), disp == 0);  // invalid pixels in black
        cv::imshow("GPUStereo Disparity", dispColor);
        if(cv::waitKey(1) == 'q') break;
    }

    return 0;
}
