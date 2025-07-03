#pragma once
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/utility/Serialization.hpp"


namespace dai {

/**
 * VppConfig message. Carries config for Virtual Projection Pattern algorithm
 */
class VppConfig : public Buffer {
   public:
    /**
     * Virtual projection pattern generation method
     */
    enum class Method {
        RANDOM,        // Random pattern generation
        MAX_DISTANCE   // Maximum distance based pattern generation
    };

    /**
     * Pattern generation method (Default: RANDOM)
     */
    Method method = Method::RANDOM;

    /**
     * Max projection patch size (Default: 5)
     */
    int patchSize = 5;

    /**
     * Window size for color computation in x direction (Default: 64)
     */
    int windowSizeAggX = 64;

    /**
     * Window size for color computation in y direction (Default: 3)
     */
    int windowSizeAggY = 3;

    /**
     * Projection direction (true: left->right, false: right->left) (Default: true)
     */
    bool left2right = true;

    /**
     * Alpha blending factor (Default: 0.4)
     */
    float blending = 0.4f;

    /**
     * Alpha blending factor in occluded areas (Default: 0.0)
     */
    float blendingOcclusion = 0.0f;

    /**
     * Use patch size based on distance (Default: false)
     */
    bool useDistancePatch = false;

    /**
     * Use adaptive patch based on bilateral filling (Default: false)
     */
    bool useBilateralPatch = false;

    /**
     * Distance patch hyperparameter (Default: 0.3)
     */
    float distanceGamma = 0.3f;

    /**
     * Bilateral filtering spatial parameter (Default: 2.0)
     */
    float bilateralSpatialSigma = 2.0f;

    /**
     * Bilateral filtering intensity parameter (Default: 1.0)
     */
    float bilateralIntensitySigma = 1.0f;

    /**
     * Bilateral filtering threshold (Default: 0.001)
     */
    float bilateralThreshold = 0.001f;

    /**
     * Use uniform color for pattern generation (Default: false)
     */
    bool uniformColor = false;

    /**
     * Use "NO" occlusion strategy -- ie, discard occluded points (Default: false)
     */
    bool discardOcclusion = false;

    /**
     * Use weighted splatting of patterns in target view (Default: true)
     */
    bool interpolate = true;

    /**
     * Minimum disparity threshold for processing (Default: 0.0)
     */
    float disparityMinThreshold = 0.0f;

    /**
     * Maximum disparity threshold for processing (Default: 255.0)
     */
    float disparityMaxThreshold = 255.0f;

    /**
     * Construct VppConfig message.
     */
    VppConfig() = default;
    virtual ~VppConfig() = default;

    /**
     * Set pattern generation method
     * @param method Pattern generation method
     */
    VppConfig& setMethod(Method method);

    /**
     * Set patch size for projection
     * @param size Patch size (must be odd number)
     */
    VppConfig& setPatchSize(int size);

    /**
     * Set aggregation window size
     * @param sizeX Window size in X direction
     * @param sizeY Window size in Y direction
     */
    VppConfig& setAggregationWindowSize(int sizeX, int sizeY);

    /**
     * Set projection direction
     * @param left2right True for left to right projection
     */
    VppConfig& setProjectionDirection(bool left2right);

    /**
     * Set blending factors
     * @param blending Alpha blending factor
     * @param blendingOcclusion Alpha blending factor for occluded areas
     */
    VppConfig& setBlending(float blending, float blendingOcclusion);

    /**
     * Enable/disable distance-based patch sizing
     * @param enable Enable distance patch
     * @param gamma Distance gamma parameter
     */
    VppConfig& setDistancePatch(bool enable, float gamma = 0.3f);

    /**
     * Enable/disable bilateral patch filtering
     * @param enable Enable bilateral patch
     * @param spatialSigma Spatial sigma parameter
     * @param intensitySigma Intensity sigma parameter
     * @param threshold Bilateral threshold
     */
    VppConfig& setBilateralPatch(bool enable, float spatialSigma = 2.0f, float intensitySigma = 1.0f, float threshold = 0.001f);

    /**
     * Set disparity processing range
     * @param minThreshold Minimum disparity threshold
     * @param maxThreshold Maximum disparity threshold
     */
    VppConfig& setDisparityRange(float minThreshold, float maxThreshold);

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::VppConfig;
    };

    DEPTHAI_SERIALIZE(VppConfig,
                      method,
                      patchSize,
                      windowSizeAggX,
                      windowSizeAggY,
                      left2right,
                      blending,
                      blendingOcclusion,
                      useDistancePatch,
                      useBilateralPatch,
                      distanceGamma,
                      bilateralSpatialSigma,
                      bilateralIntensitySigma,
                      bilateralThreshold,
                      uniformColor,
                      discardOcclusion,
                      interpolate,
                      disparityMinThreshold,
                      disparityMaxThreshold);
};

}  // namespace dai