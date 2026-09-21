#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for XFeatStereoParser.
 */
class XFeatStereoParserConfig : public Buffer {
   public:
    int maxKeypoints = 4096;

    XFeatStereoParserConfig() = default;

    ~XFeatStereoParserConfig() override;

    /**
     * Set the maximum number of keypoints to retain from each frame in the stereo pair.
     * @param maxKeypoints Maximum keypoint count, which must be positive
     */
    void setMaxKeypoints(int maxKeypoints);

    /**
     * Get the maximum number of keypoints to retain from each frame in the stereo pair.
     * @return Maximum keypoint count applied to each frame
     */
    int getMaxKeypoints() const;

    /**
     * Check whether all configuration values are valid.
     * @return True when maxKeypoints is positive
     */
    bool validate() const;

    /**
     * Serialize this configuration into stream metadata.
     * @param metadata Output buffer that receives the serialized configuration
     * @param datatype Output datatype identifier, set to XFeatStereoParserConfig
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Get the datatype identifier for XFeatStereoParserConfig.
     * @return DatatypeEnum::XFeatStereoParserConfig
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::XFeatStereoParserConfig;
    }

    DEPTHAI_SERIALIZE(XFeatStereoParserConfig, maxKeypoints);
};

}  // namespace beta
}  // namespace dai
