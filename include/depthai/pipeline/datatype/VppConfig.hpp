#pragma once
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

/**
 * VppConfig message. Carries config for Virtual Projection Pattern algorithm
 */
class VppConfig : public Buffer {
   public:
    enum PatchColoringType : int {
        RANDOM = 0,
        MAXDIST = 1  // color the patch with the most distant color
    };

    struct InjectionParameters {
        bool useInjection = true;
        int kernelSize = 3;
        float textureThreshold = 0.5;
        float confidenceThreshold = 0.5;
        int morphologyIterations = 5;
        bool useMorphology = true;

        // clang-format off
        DEPTHAI_SERIALIZE(
	    InjectionParameters,
	    useInjection,
	    kernelSize,
	    textureThreshold,
	    confidenceThreshold,
	    morphologyIterations,
	    useMorphology);
        // clang-format on
    };

    float blending = 0.5;
    float distanceGamma = 0.3;
    uint8_t maxPatchSize = 3;
    PatchColoringType patchColoringType = PatchColoringType::RANDOM;
    bool uniformPatch = true;
    InjectionParameters injectionParameters;
    int maxNumThreads = 8;
    int maxFPS = 0;

    VppConfig() = default;
    virtual ~VppConfig();

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::VppConfig;
    };

    DEPTHAI_SERIALIZE(VppConfig, blending, distanceGamma, maxPatchSize, patchColoringType, uniformPatch, injectionParameters, maxNumThreads, maxFPS);
};

}  // namespace dai
