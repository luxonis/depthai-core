#include "depthai/depthai.hpp"
#include <iostream>

void printExtrinsics(const dai::Extrinsics& extr) {
        std::cout << "To camera socket: " << static_cast<int>(extr.toCameraSocket) << std::endl;
        std::cout << "Translation x: " << extr.translation.x << std::endl;
        std::cout << "Translation y: " << extr.translation.y << std::endl;
        std::cout << "Translation z: " << extr.translation.z << std::endl;
        std::cout << "Spec Translation x: " << extr.specTranslation.x << std::endl;
        std::cout << "Spec Translation y: " << extr.specTranslation.y << std::endl;
        std::cout << "Spec Translation z: " << extr.specTranslation.z << std::endl;
        if(extr.toCameraSocket == dai::CameraBoardSocket::AUTO){
            return;
        }
        std::cout << "Rotation: " << std::endl;
        for(int i = 0; i < 3; ++i) {
            for(int j = 0; j < 3; ++j) {
                std::cout << extr.rotationMatrix[i][j]<< " ";
            }
            std::cout << std::endl;
        }
}

int main() {
    dai::Pipeline pipeline;

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto color = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto monoLeftOut = monoLeft->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);
    auto monoRightOut = monoRight->requestFullResolutionOutput(dai::ImgFrame::Type::NV12);

    auto colorOut = color->requestOutput(std::make_pair(640, 400));

    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);
    colorOut->link(stereo->inputAlignTo);

    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);

    auto syncedLeftQueue = stereo->syncedLeft.createOutputQueue();
    auto syncedRightQueue = stereo->syncedRight.createOutputQueue();
    auto rectifiedLeftQueue = stereo->rectifiedLeft.createOutputQueue();
    auto rectifiedRightQueue = stereo->rectifiedRight.createOutputQueue();
    auto disparityQueue = stereo->disparity.createOutputQueue();

    double maxDisparity = 1.0;
    pipeline.start();
    while(true) {
        auto leftSynced = syncedLeftQueue->get<dai::ImgFrame>();
        auto rightSynced = syncedRightQueue->get<dai::ImgFrame>();
        auto leftRectified = rectifiedLeftQueue->get<dai::ImgFrame>();
        auto rightRectified = rectifiedRightQueue->get<dai::ImgFrame>();
        std::cout << "Extrinsics:" << std::endl;
        std::cout << "Left Synced:" << std::endl;
        printExtrinsics(leftSynced->transformation.getExtrinsics());
        std::cout << "Left Rectified:" << std::endl;
        printExtrinsics(leftRectified->transformation.getExtrinsics());
        std::cout << "Right Synced:" << std::endl;
        printExtrinsics(rightSynced->transformation.getExtrinsics());
        std::cout << "Right Rectified:" << std::endl;
        printExtrinsics(rightRectified->transformation.getExtrinsics());
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        std::cout << "Disparity:" << std::endl;
        printExtrinsics(disparity->transformation.getExtrinsics());
    }

    pipeline.stop();
    return 0;
}
