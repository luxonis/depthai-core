#pragma once

#include <optional>
#include <string>

#include "depthai/common/Extrinsics.hpp"
#include "utilities/DeviceModel.hpp"

namespace dai {

std::optional<Extrinsics> getFwImuExtrinsicsOverride(gate::DeviceModel::Model model, const std::string& imuType);

}
