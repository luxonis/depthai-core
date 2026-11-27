#pragma once
#include "depthai/common/optional.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImageFiltersConfig.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * ToFConfig message. Carries config for feature tracking algorithm
 */
class ToFConfig : public Buffer {
   public:
    enum class Profile : uint32_t {
        LOW_RANGE,
        MID_RANGE,
        HIGH_RANGE,
    };

    Profile profile = Profile::MID_RANGE;
    /**
     * Set kernel size for depth median filtering, or disable
     */
    filters::params::MedianFilter median = filters::params::MedianFilter::MEDIAN_OFF;

    int phaseUnwrappingLevel = 4;

    uint16_t phaseUnwrapErrorThreshold = 100;

    bool enablePhaseShuffleTemporalFilter = true;

    bool enableBurstMode = false;  // Decodes 4 frames at a time: output fps is 4 times lower, less motion blur

    bool enableDistortionCorrection = true;  // Only when calibration is present

    std::optional<bool> enableFPPNCorrection;
    std::optional<bool> enableOpticalCorrection;
    std::optional<bool> enableTemperatureCorrection;
    std::optional<bool> enableWiggleCorrection;
    std::optional<bool> enablePhaseUnwrapping;

    /**
     * Publish intermediate S5K63D depth updates using previous-stack frequency history.
     * Enabled by default to preserve the existing output cadence. Set false to publish
     * only depth using both frequency pairs from the current stack.
     * Applies when phase unwrapping is enabled; burst mode always suppresses intermediate updates.
     * Other sensors are unaffected. Supports initialConfig and runtime inputConfig.
     */
    bool enableIntermediateDepth = true;

    /**
     * Construct ToFConfig message.
     */
    ToFConfig() = default;
    virtual ~ToFConfig();

    /**
     * @param median Set kernel size for median filtering, or disable
     */
    ToFConfig& setMedianFilter(filters::params::MedianFilter median);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DatatypeEnum getDatatype() const override {
        return DatatypeEnum::ToFConfig;
    }

    /**
     * Set preset mode for ToFConfig.
     * @param presetMode Preset mode for ToFConfig.
     */
    void setProfilePreset(Profile profile);

    DEPTHAI_SERIALIZE_DECL(ToFConfig,
                           profile,
                           median,
                           enablePhaseShuffleTemporalFilter,
                           enableBurstMode,
                           enableDistortionCorrection,
                           enableFPPNCorrection,
                           enableOpticalCorrection,
                           enableTemperatureCorrection,
                           enableWiggleCorrection,
                           enablePhaseUnwrapping,
                           phaseUnwrappingLevel,
                           phaseUnwrapErrorThreshold,
                           enableIntermediateDepth);
};

}  // namespace dai
