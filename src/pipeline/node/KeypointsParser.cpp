#include "depthai/pipeline/node/KeypointsParser.hpp"

namespace dai {
namespace node {

std::shared_ptr<KeypointsParser> KeypointsParser::build(const NNArchive& nnArchive) {
    if(isBuilt) {
        throw std::runtime_error("KeypointsParser node is already built");
    }

    if (nnArchive.getConfig().getConfigV1().has_value()
    && nnArchive.getConfig().getConfigV1().value().model.heads.has_value()
    && !nnArchive.getConfig().getConfigV1().value().model.heads.value().empty()) {
        nlohmann::json metadata = nnArchive.getConfig().getConfigV1().value().model.heads.value()[0].metadata.extraParams;

        if (metadata.contains("n_keypoints")) {
            setNumKeypoints(metadata["n_keypoints"]);
        }
        if (metadata.contains("scale_factor")) {
            setScaleFactor(metadata["scale_factor"]);
        }
    }

    isBuilt = true;
    return std::static_pointer_cast<KeypointsParser>(shared_from_this());
}

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

#ifdef DEPTHAI_XTENSOR_SUPPORT
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

        xt::xarray<float> keypointsData = inputData->getFirstTensor<float>(true);
        int totalCoords = std::accumulate(keypointsData.shape().begin(), keypointsData.shape().end(), 1, std::multiplies<int>());

        if (numKeypoints * 2 != totalCoords && numKeypoints * 3 != totalCoords) {
            throw std::runtime_error("Expected 2 or 3 coordinates per keypoint, got " + std::to_string(static_cast<float>(totalCoords) / static_cast<float>(numKeypoints)));
        }
        int pointDimension = totalCoords / numKeypoints;

        keypointsData = keypointsData.reshape({numKeypoints, pointDimension});
        keypointsData /= scaleFactor;

        std::shared_ptr<Keypoints> msg = std::make_shared<Keypoints>();

        if (pointDimension == 2) {
            std::vector<Point2f> keypoints = std::vector<Point2f>(numKeypoints);
            for (int i = 0; i < numKeypoints; i++) {
                keypoints[i].x = keypointsData(i, 0);
                keypoints[i].y = keypointsData(i, 1);
            }
            msg->setKeypoints(keypoints);
        }
        else {
            std::vector<Point3f> keypoints = std::vector<Point3f>(numKeypoints);
            for (int i = 0; i < numKeypoints; i++) {
                keypoints[i].x = keypointsData(i, 0);
                keypoints[i].y = keypointsData(i, 1);
                keypoints[i].z = keypointsData(i, 2);
            }
            msg->setKeypoints(keypoints);
        }

        msg->setTimestamp(inputData->getTimestamp());
        msg->setTimestampDevice(inputData->getTimestampDevice());
        msg->setSequenceNum(inputData->getSequenceNum());
        out.send(msg);
    }
}
#else
void KeypointsParser::run() {
    throw std::runtime_error("KeypointsParser node requires xtensor support");
}
#endif

}  // namespace node
}  // namespace dai
