#include <iostream>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;
    pipeline.enablePipelineDebugging();

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto monoLeftOut = monoLeft->requestFullResolutionOutput();
    auto monoRightOut = monoRight->requestFullResolutionOutput();

    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    stereo->setRectification(true);
    stereo->setExtendedDisparity(true);
    stereo->setLeftRightCheck(true);

    auto depthQueue = stereo->depth.createOutputQueue();

    pipeline.start();
    while(pipeline.isRunning()) {
        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::imshow("depth", dai::utility::colorizeDepthFrame(*depth).getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 's') {
            auto state = pipeline.getPipelineState().nodes().detailed();
            try {
                // Assuming these APIs exist similarly in C++
                auto pipelineState = pipeline.getPipelineState().nodes().detailed();

                for(const auto& [nodeId, nodeState] : pipelineState.nodeStates) {
                    std::string nodeName = pipeline.getNode(nodeId)->getName();

                    std::cout << "\n# State for node " << nodeName << " (" << nodeId << "):\n";

                    std::cout << "## State: " << (int)nodeState.state << "\n";

                    std::cout << "## mainLoopTiming: " << (nodeState.mainLoopTiming.isValid() ? "" : "invalid") << "\n";
                    if(nodeState.mainLoopTiming.isValid()) {
                        std::cout << "-----\n" << nodeState.mainLoopTiming.str() << "\n-----\n";
                    }

                    std::cout << "## inputsGetTiming: " << (nodeState.inputsGetTiming.isValid() ? "" : "invalid") << "\n";
                    if(nodeState.inputsGetTiming.isValid()) {
                        std::cout << "-----\n" << nodeState.inputsGetTiming.str() << "\n-----\n";
                    }

                    std::cout << "## outputsSendTiming: " << (nodeState.outputsSendTiming.isValid() ? "" : "invalid") << "\n";
                    if(nodeState.outputsSendTiming.isValid()) {
                        std::cout << "-----\n" << nodeState.outputsSendTiming.str() << "\n-----\n";
                    }

                    std::cout << "## inputStates: " << (nodeState.inputStates.empty() ? "empty" : "") << "\n";
                    for(const auto& [inputName, inputState] : nodeState.inputStates) {
                        if(inputState.isValid())
                            std::cout << "### " << inputName << ":\n-----" << inputState.str() << "\n-----\n";
                        else
                            std::cout << "### " << inputName << ": invalid\n";
                    }

                    std::cout << "## outputStates: " << (nodeState.outputStates.empty() ? "empty" : "") << "\n";
                    for(const auto& [outputName, outputState] : nodeState.outputStates) {
                        std::cout << "### " << outputName << ":\n-----" << outputState.str() << "\n-----\n";
                    }

                    std::cout << "## otherTimings: " << (nodeState.otherTimings.empty() ? "empty" : "") << "\n";
                    for(const auto& [otherName, otherTiming] : nodeState.otherTimings) {
                        std::cout << "### " << otherName << ":\n-----" << otherTiming.str() << "\n-----\n";
                    }
                }
            } catch(const std::runtime_error& e) {
                std::cerr << "Error getting pipeline state: " << e.what() << "\n";
            }
        }
    }

    pipeline.stop();

    return 0;
}
