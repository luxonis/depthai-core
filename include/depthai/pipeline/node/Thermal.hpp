#pragma once
#include "depthai/pipeline/Node.hpp"

// shared
#include <depthai-shared/properties/ThermalProperties.hpp>

namespace dai {
namespace node {

/**
 * @brief Thermal node. For use with grayscale sensors.
 */
class Thermal : public NodeCRTP<Node, Thermal, ThermalProperties> {
   public:
    constexpr static const char* NAME = "Thermal";

   protected:
    Properties& getProperties();

   public:
    Thermal(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId);
    Thermal(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId, std::unique_ptr<Properties> props);

    /**
     * Outputs ImgFrame message that carries RAW8 encoded (grayscale) frame data.
     *
     * Suitable for use StereoDepth node. Processed by ISP
     */
    Output out{*this, "out", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};
    Output raw{*this, "raw", Output::Type::MSender, {{DatatypeEnum::ImgFrame, false}}};

};

}  // namespace node
}  // namespace dai
