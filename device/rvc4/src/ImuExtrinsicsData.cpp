#include "ImuExtrinsicsData.hpp"

#include <algorithm>
#include <cctype>

namespace dai {
namespace {

Extrinsics makeCamAExtrinsics(Point3f specTranslation) {
    Extrinsics extrinsics;
    extrinsics.specTranslation = specTranslation;
    extrinsics.toCameraSocket = CameraBoardSocket::CAM_A;
    return extrinsics;
}

const Extrinsics kOak4DImuExtrinsicsOverride = makeCamAExtrinsics({-4.4032f, -0.2697f, -0.2f});  // From HW design / depthai_boards; empty rotation = identity
const Extrinsics kOak4SImuExtrinsicsOverride = makeCamAExtrinsics({1.2263f, 1.9577f, -0.2f});
const Extrinsics kOak4LiteImuExtrinsicsOverride = makeCamAExtrinsics({-5.3f, 0.7792f, 0.0f});

}  // namespace

std::optional<Extrinsics> getFwImuExtrinsicsOverride(gate::DeviceModel::Model model, const std::string& imuType) {
    std::string normalizedImuType = imuType;
    std::transform(normalizedImuType.begin(), normalizedImuType.end(), normalizedImuType.begin(), ::toupper);

    if(normalizedImuType != "LSM6" && normalizedImuType != "ACCEL") {
        return std::nullopt;
    }

    // TODO(stasbucik) @MaticTonin - add OAK4-MAX support
    switch(model) {
        case gate::DeviceModel::Model::OAK4_S:
            return kOak4SImuExtrinsicsOverride;
        case gate::DeviceModel::Model::OAK4_D:
            return kOak4DImuExtrinsicsOverride;
        case gate::DeviceModel::Model::OAK4_LITE:
            return kOak4LiteImuExtrinsicsOverride;
        default:
            return std::nullopt;
    }
}

}
