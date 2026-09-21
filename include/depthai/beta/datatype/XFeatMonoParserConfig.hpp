#pragma once

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {
namespace beta {

/**
 * Runtime configuration for XFeatMonoParser.
 */
class XFeatMonoParserConfig : public Buffer {
   public:
    int maxKeypoints = 4096;

    XFeatMonoParserConfig() = default;

    ~XFeatMonoParserConfig() override;

    /**
     * Set the maximum number of keypoints to retain per frame.
     * @param maxKeypoints Maximum keypoint count, which must be positive
     */
    void setMaxKeypoints(int maxKeypoints);

    /**
     * Get the maximum number of keypoints to retain per frame.
     * @return Maximum keypoint count
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
     * @param datatype Output datatype identifier, set to XFeatMonoParserConfig
     */
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    /**
     * Get the datatype identifier for XFeatMonoParserConfig.
     * @return DatatypeEnum::XFeatMonoParserConfig
     */
    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::XFeatMonoParserConfig;
    }

    DEPTHAI_SERIALIZE(XFeatMonoParserConfig, maxKeypoints);
};

}  // namespace beta
}  // namespace dai
