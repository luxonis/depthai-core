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
        // Getters and setters with docstrings
        /**
         * @brief Check if injection is enabled
         * False: all possible pixels will be used.
         * @return True if injection is used
         */
        bool getUseInjection() const {
            return useInjection;
        }

        /**
         * @brief Enable or disable injection
         * @param value True to use injection, false to disable
         */
        void setUseInjection(bool value) {
            useInjection = value;
        }

        /**
         * @brief Get kernel size for injection
         * @return Kernel size
         */
        int getKernelSize() const {
            return kernelSize;
        }

        /**
         * @brief Set kernel size for injection
         * @param value Kernel size to set
         */
        void setKernelSize(int value) {
            kernelSize = value;
        }

        /**
         * @brief Get texture threshold
         * @return Texture threshold value
         */
        float getTextureThreshold() const {
            return textureThreshold;
        }

        /**
         * @brief Set texture threshold
         * @param value Texture threshold to set
         */
        void setTextureThreshold(float value) {
            textureThreshold = value;
        }

        /**
         * @brief Get confidence threshold
         * @return Confidence threshold value
         */
        float getConfidenceThreshold() const {
            return confidenceThreshold;
        }

        /**
         * @brief Set confidence threshold
         * @param value Confidence threshold to set
         */
        void setConfidenceThreshold(float value) {
            confidenceThreshold = value;
        }

        /**
         * @brief Get number of morphology iterations
         * @return Number of iterations
         */
        int getMorphologyIterations() const {
            return morphologyIterations;
        }

        /**
         * @brief Set number of morphology iterations
         * @param value Number of iterations
         */
        void setMorphologyIterations(int value) {
            morphologyIterations = value;
        }

        /**
         * @brief Check if morphology is used
         * @return True if morphology is applied
         */
        bool isUseMorphology() const {
            return useMorphology;
        }

        /**
         * @brief Enable or disable morphology
         * @param value True to use morphology, false to disable
         */
        void setUseMorphology(bool value) {
            useMorphology = value;
        }

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
    int maxNumThreads = 1;
    int maxFPS = 0;

    VppConfig() = default;
    virtual ~VppConfig();

    // Getters and setters with docstrings
    /**
     * @brief Get blending factor between 0 and 1
     * @return Current blending value
     */
    float getBlending() const {
        return blending;
    }

    /**
     * @brief Set blending factor between 0 and 1:
     *      resulting pixel: patchColor * blending + (1 - blending) * imgFrame
     * @param value Blending value to set
     */
    void setBlending(float value) {
        blending = value;
    }

    /**
     * @brief Get distance gamma
     * @return Current distance gamma
     */
    float getDistanceGamma() const {
        return distanceGamma;
    }

    /**
     * @brief Set distance gamma
     * @param value Distance gamma to set
     */
    void setDistanceGamma(float value) {
        distanceGamma = value;
    }

    /**
     * @brief Get maximum patch size
     * @return Maximum patch size
     */
    uint8_t getMaxPatchSize() const {
        return maxPatchSize;
    }

    /**
     * @brief Set maximum patch size
     * @param value Patch size to set
     */
    void setMaxPatchSize(uint8_t value) {
        maxPatchSize = value;
    }

    /**
     * @brief Get patch coloring type: UNIFORM/MAXDIST
     * @return Current patch coloring type
     */
    PatchColoringType getPatchColoringType() const {
        return patchColoringType;
    }

    /**
     * @brief Set patch coloring type: UNIFORM/MAXDIST
     * @param type Patch coloring type to set
     */
    void setPatchColoringType(PatchColoringType type) {
        patchColoringType = type;
    }

    /**
     * @brief Check if uniform patching is enabled
     * @return True if uniform patching is enabled
     */
    bool getUniformPatch() const {
        return uniformPatch;
    }

    /**
     * @brief Enable or disable uniform patching
     * @param value True to enable, false to disable
     */
    void setUniformPatch(bool value) {
        uniformPatch = value;
    }

    /**
     * @brief Get injection parameters
     * @return Current injection parameters
     */
    InjectionParameters getInjectionParameters() const {
        return injectionParameters;
    }

    /**
     * @brief Set injection parameters
     * @param params Injection parameters to set
     */
    void setInjectionParameters(const InjectionParameters& params) {
        injectionParameters = params;
    }

    /**
     * @brief Get maximum number of threads
     * @return Maximum number of threads
     */
    int getMaxNumThreads() const {
        return maxNumThreads;
    }

    /**
     * @brief Set maximum number of threads
     * @param value Number of threads to set
     */
    void setMaxNumThreads(int value) {
        maxNumThreads = value;
    }

    /**
     * @brief Get maximum FPS
     * @return Maximum FPS
     */
    int getMaxFPS() const {
        return maxFPS;
    }

    /**
     * @brief Set maximum FPS
     * @param value Maximum FPS to set
     */
    void setMaxFPS(int value) {
        maxFPS = value;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::VppConfig;
    };

    DEPTHAI_SERIALIZE(VppConfig, blending, distanceGamma, maxPatchSize, patchColoringType, uniformPatch, injectionParameters, maxNumThreads, maxFPS);
};

}  // namespace dai
