
#include <csignal>
#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>

#include "depthai/depthai.hpp"

class CustomPCLProcessingNode : public dai::NodeCRTP<dai::node::ThreadedHostNode, CustomPCLProcessingNode> {
   public:
    constexpr static const char* NAME = "CustomPCLProcessingNode";
    constexpr static const float thresholdDistance = 3000.0f;

   public:
    Input inputPCL{*this, {"inPCL", DEFAULT_GROUP, DEFAULT_BLOCKING, DEFAULT_QUEUE_SIZE, {{{dai::DatatypeEnum::PointCloudData, true}}}}};
    Output outputPCL{*this, {"outPCL", DEFAULT_GROUP, {{{dai::DatatypeEnum::PointCloudData, true}}}}};

    void run() override {
        while(isRunning()) {
            auto pclIn = inputPCL.get<dai::PointCloudData>();
            auto pclOut = std::make_shared<dai::PointCloudData>();
            if(pclIn != nullptr) {
                const auto& pclData = pclIn->getPointsRGB();
                std::vector<dai::Point3fRGBA> updatedPoints;
                const float thresholdSquared = thresholdDistance * thresholdDistance;
                updatedPoints.reserve(pclData.size());
                for(const auto& point : pclData) {
                    const float distance = point.x * point.x + point.y * point.y + point.z * point.z;
                    if(distance <= thresholdSquared) {
                        updatedPoints.emplace_back(point.x, point.y, point.z, point.r, point.g, point.b, point.a);
                    }
                }
                updatedPoints.shrink_to_fit();
                pclOut->setPointsRGB(updatedPoints);
                outputPCL.send(pclOut);
            }
        }
    }
};

// Signal handling for clean shutdown
static bool isRunning = true;
void signalHandler(int signum) {
    isRunning = false;
}

int main() {
    using namespace std;
    // Default port values
    int webSocketPort = 8765;
    int httpPort = 8082;

    // Register signal handler
    std::signal(SIGINT, signalHandler);

    // Create RemoteConnection
    dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);
    // Create pipeline
    dai::Pipeline pipeline;
    auto rgbd = pipeline.create<dai::node::RGBD>()->build(true, dai::node::StereoDepth::PresetMode::DEFAULT);
    auto customNode = pipeline.create<CustomPCLProcessingNode>();

    rgbd->pcl.link(customNode->inputPCL);

    remoteConnector.addTopic("pcl", rgbd->pcl);
    remoteConnector.addTopic("processed_pcl", customNode->outputPCL);
    pipeline.start();
    remoteConnector.registerPipeline(pipeline);
    auto device = pipeline.getDefaultDevice();
    device->setIrLaserDotProjectorIntensity(0.7);
    // Main loop
    while(isRunning && pipeline.isRunning()) {
        int key = remoteConnector.waitKey(1);
        if(key == 'q') {
            std::cout << "Got 'q' key from the remote connection!" << std::endl;
            break;
        }
    }

    std::cout << "Pipeline stopped." << std::endl;
    return 0;
}
