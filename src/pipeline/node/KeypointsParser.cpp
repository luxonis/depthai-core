#include "depthai/pipeline/node/KeypointsParser.hpp"

namespace dai {
namespace node {

void KeypointsParser::setScaleFactor(float scaleFactor) {
    properties.scaleFactor = scaleFactor;
}
float KeypointsParser::getScaleFactor() const {
    return properties.scaleFactor;
}

void KeypointsParser::setNumKeypoints(int numKeypoints) {
    properties.numKeypoints = numKeypoints;
}
int KeypointsParser::getNumKeypoints() const {
    return properties.numKeypoints;
}

void KeypointsParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}
bool KeypointsParser::runOnHost() const {
    return runOnHostVar;
}

void KeypointsParser::run() {
    auto numKeypoints = properties.numKeypoints;
    auto scaleFactor = properties.scaleFactor;

    if(numKeypoints == -1) {
        throw std::runtime_error("Number of keypoints must be specified!");
    }

    while(isRunning()) {
        auto inputData = input.get<NNData>();
        if(inputData == nullptr) {
                throw std::invalid_argument("Received nullptr from input");
        }

        std::vector<std::string> outputLayerNames = inputData->getAllLayerNames();
        if (outputLayerNames.size() != 1) {
            throw std::invalid_argument("Expected 1 output layer, got " + std::to_string(outputLayerNames.size()));
        }

        xt::xarray<float> keypoints = inputData->getFirstTensor<float>(true);
        int totalCoords = std::accumulate(keypoints.shape().begin(), keypoints.shape().end(), 1, std::multiplies<int>());

        if (numKeypoints * 2 != totalCoords && numKeypoints * 3 != totalCoords) {
            throw std::runtime_error("Expected 2 or 3 coordinates per keypoint, got " + std::to_string(static_cast<float>(totalCoords) / static_cast<float>(numKeypoints)));
        }
        int pointDimension = totalCoords / numKeypoints;

        keypoints = keypoints.reshape({numKeypoints, pointDimension});
        keypoints /= scaleFactor;

        std::shared_ptr<Keypoints> msg = std::make_shared<Keypoints>();

        if (pointDimension == 2) {
            std::vector<Point2f> keypointsVector = std::vector<Point2f>(numKeypoints);
            for (int i = 0; i < numKeypoints; i++) {
                keypointsVector[i].x = keypoints(i, 0);
                keypointsVector[i].y = keypoints(i, 1);
            }
            msg->setKeypoints(keypointsVector);
        }
        else {
            std::vector<Point3f> keypointsVector = std::vector<Point3f>(numKeypoints);
            for (int i = 0; i < numKeypoints; i++) {
                keypointsVector[i].x = keypoints(i, 0);
                keypointsVector[i].y = keypoints(i, 1);
                keypointsVector[i].z = keypoints(i, 2);
            }
            msg->setKeypoints(keypointsVector);
        }

        msg->setTimestamp(inputData->getTimestamp());
        msg->setTimestampDevice(inputData->getTimestampDevice());
        msg->setSequenceNum(inputData->getSequenceNum());
        out.send(msg);
    }
}

}  // namespace node
}  // namespace dai
