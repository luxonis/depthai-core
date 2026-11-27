#pragma once
#include <cstdint>

#include "depthai/properties/Properties.hpp"
namespace dai {
/** Properties for the native RVC4 OpticalFlow node. */
struct OpticalFlowProperties : PropertiesSerializable<Properties, OpticalFlowProperties> {
    /** Reserved wire-format field. Must remain zero. */
    std::uint32_t reserved = 0;
    ~OpticalFlowProperties() override;
};
DEPTHAI_SERIALIZE_EXT_DECL(OpticalFlowProperties, reserved);
}  // namespace dai
extern template struct dai::PropertiesSerializable<dai::Properties, dai::OpticalFlowProperties>;
