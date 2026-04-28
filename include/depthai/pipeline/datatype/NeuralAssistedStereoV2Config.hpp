#pragma once

#include <depthai/common/DepthUnit.hpp>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class NeuralAssistedStereoV2Config : public Buffer {
   public:
    enum class CostMethod { SAD, ZNCC };

    enum class PrefilterMethod { NONE, SOBEL_X, BOX_MEAN_SUBTRACT, GAUSSIAN_3x3 };

    struct AlgorithmControl {
        using DepthUnit = dai::DepthUnit;
        DepthUnit depthUnit = DepthUnit::MILLIMETER;
        float customDepthUnitMultiplier = 1000.f;

        DEPTHAI_SERIALIZE(AlgorithmControl, depthUnit, customDepthUnitMultiplier);
    };

    // Neural fusion search window
    float kappa0 = 5.0f;            // confidence knee: α = curv/(curv+κ₀); higher → trust neural more
    float radius_scale = 0.05f;     // search radius = d_prior × scale
    int radius_min = 2;             // minimum search radius (pixels)
    int radius_max = 12;            // maximum search radius (pixels, ≤15 for register array)

    // Matching
    CostMethod cost_method = CostMethod::ZNCC;
    int block_match_radius = 3;
    int subpixel_bits = 4;
    int min_disp = 0;
    int max_disparity = 128;        // safety cap on disparity range

    // Prefilter
    PrefilterMethod prefilter_method = PrefilterMethod::NONE;
    float prefilter_sigma_spatial = 2.f;
    float prefilter_sigma_range = 0.08f;

    // Post-processing
    bool lr_check = true;
    bool lr_check_fast = false;
    int median_size = 3;
    int confidence_threshold = 10;
    int speckle_max_size = 0;
    int speckle_max_diff = 1;
    int hole_fill_radius = 0;
    float hole_fill_sigma_spatial = 2.0f;
    float hole_fill_sigma_range = 0.05f;
    float temporal_alpha = 0.f;
    int temporal_delta = 0;
    int temporal_persistency_mode = 3;
    float second_peak_threshold = 0.f;
    int second_peak_min_gap = 0;

    // Output
    bool use_fp16 = true;
    AlgorithmControl algorithmControl;

    // Debug
    int debug_zncc_plot_x = -1;
    int debug_zncc_plot_y = -1;

    NeuralAssistedStereoV2Config() = default;
    virtual ~NeuralAssistedStereoV2Config();

    int invalid_disparity() const {
        int scale = subpixel_bits > 0 ? (1 << subpixel_bits) : 1;
        int raw = (min_disp - 1) * scale;
        return raw < 0 ? 0 : raw;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(NeuralAssistedStereoV2Config,
                      kappa0,
                      radius_scale,
                      radius_min,
                      radius_max,
                      cost_method,
                      block_match_radius,
                      subpixel_bits,
                      min_disp,
                      max_disparity,
                      prefilter_method,
                      prefilter_sigma_spatial,
                      prefilter_sigma_range,
                      lr_check,
                      lr_check_fast,
                      median_size,
                      confidence_threshold,
                      speckle_max_size,
                      speckle_max_diff,
                      hole_fill_radius,
                      hole_fill_sigma_spatial,
                      hole_fill_sigma_range,
                      temporal_alpha,
                      temporal_delta,
                      temporal_persistency_mode,
                      second_peak_threshold,
                      second_peak_min_gap,
                      use_fp16,
                      algorithmControl,
                      debug_zncc_plot_x,
                      debug_zncc_plot_y);
};

}  // namespace dai
