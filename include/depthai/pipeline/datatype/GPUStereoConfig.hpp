#pragma once

#include <depthai/common/DepthUnit.hpp>

#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

class GPUStereoConfig : public Buffer {
   public:
    enum class DownsampleMethod { BOX_FILTER, BILINEAR, GAUSSIAN_5x5, GAUSSIAN_3x3, NEAREST };

    enum class PrefilterMethod { NONE, SOBEL_X, BOX_MEAN_SUBTRACT, GAUSSIAN_3x3, BILATERAL_PREFILTER };

    enum class CostVolumeAggregation { WTA, BOX, BILATERAL };

    enum class CostMethod { SAD, CENSUS, GRADIENT, ZNCC, RANK };

    enum class PathAggregation { NONE, SGM_2, SGM_4, SGM_8, SGM_MGM_INPLACE };

    struct AlgorithmControl {
        using DepthUnit = dai::DepthUnit;
        DepthUnit depthUnit = DepthUnit::MILLIMETER;
        float customDepthUnitMultiplier = 1000.f;

        DEPTHAI_SERIALIZE(AlgorithmControl, depthUnit, customDepthUnitMultiplier);
    };

    int max_disparity = 128;
    int num_pyramid_levels = 3;
    DownsampleMethod downsample_method = DownsampleMethod::BOX_FILTER;
    PrefilterMethod prefilter_method = PrefilterMethod::NONE;
    int block_match_radius = 3;
    float adaptive_support_range_sigma = 0.f;
    float prefilter_bilateral_sigma_spatial = 2.f;
    float prefilter_bilateral_sigma_range = 0.08f;
    int refinement_radius = 6;
    int refinement_radius_full = 3;
    int subpixel_bits = 4;
    bool lr_check = true;
    bool lr_check_fast = true;
    int median_size = 3;
    int min_disp = 0;
    int confidence_threshold = 25;
    bool use_cost_volume = false;
    CostVolumeAggregation cost_volume_aggregation = CostVolumeAggregation::BOX;
    int box_aggregation_radius = 2;
    float bilateral_spatial_sigma = 2.0f;
    float bilateral_range_sigma = 0.08f;
    int bilateral_aggregation_radius = 2;
    CostMethod cost_method = CostMethod::ZNCC;
    PathAggregation path_aggregation = PathAggregation::NONE;
    float sgm_p1 = 0.5f;
    float sgm_p2 = 2.0f;
    bool sgm_adaptive_p2 = true;
    bool use_fp16 = true;
    bool use_qcom_accelerated_ops = false;
    int debug_pyramid_level = -1;
    int debug_pyramid_disparity_level = -1;
    int debug_zncc_plot_x = -1;
    int debug_zncc_plot_y = -1;
    float second_peak_threshold = 0.f;
    int second_peak_min_disparity_gap = 0;
    int census_radius_x = 2;
    int census_radius_y = 2;
    int speckle_max_size = 0;
    int speckle_max_diff = 1;
    int texture_filter_radius = 0;
    float texture_threshold = 25.0f;
    float feature_mask_edge_thresh = 0.f;
    float feature_mask_corner_thresh = 0.f;
    int feature_mask_morph_radius = 1;
    int edge_aware_radius = 0;
    float edge_aware_eps = 0.01f;
    int hole_fill_radius = 0;
    float hole_fill_sigma_spatial = 2.0f;
    float hole_fill_sigma_range = 0.05f;
    float temporal_alpha = 0.f;
    int temporal_delta = 0;
    int temporal_persistency_mode = 3;
    bool region_refine = false;
    int region_refine_cell_size = 16;
    float region_refine_plane_residual_thresh = 3.0f;

    AlgorithmControl algorithmControl;

    GPUStereoConfig() = default;
    virtual ~GPUStereoConfig();

    int invalid_disparity() const {
        int scale = subpixel_bits > 0 ? (1 << subpixel_bits) : 1;
        int raw = (min_disp - 1) * scale;
        return raw < 0 ? 0 : raw;
    }

    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override;

    DEPTHAI_SERIALIZE(GPUStereoConfig,
                      max_disparity,
                      num_pyramid_levels,
                      downsample_method,
                      prefilter_method,
                      block_match_radius,
                      adaptive_support_range_sigma,
                      prefilter_bilateral_sigma_spatial,
                      prefilter_bilateral_sigma_range,
                      refinement_radius,
                      refinement_radius_full,
                      subpixel_bits,
                      lr_check,
                      lr_check_fast,
                      median_size,
                      min_disp,
                      confidence_threshold,
                      use_cost_volume,
                      cost_volume_aggregation,
                      box_aggregation_radius,
                      bilateral_spatial_sigma,
                      bilateral_range_sigma,
                      bilateral_aggregation_radius,
                      cost_method,
                      path_aggregation,
                      sgm_p1,
                      sgm_p2,
                      sgm_adaptive_p2,
                      use_fp16,
                      use_qcom_accelerated_ops,
                      debug_pyramid_level,
                      debug_pyramid_disparity_level,
                      debug_zncc_plot_x,
                      debug_zncc_plot_y,
                      second_peak_threshold,
                      second_peak_min_disparity_gap,
                      census_radius_x,
                      census_radius_y,
                      speckle_max_size,
                      speckle_max_diff,
                      texture_filter_radius,
                      texture_threshold,
                      feature_mask_edge_thresh,
                      feature_mask_corner_thresh,
                      feature_mask_morph_radius,
                      edge_aware_radius,
                      edge_aware_eps,
                      hole_fill_radius,
                      hole_fill_sigma_spatial,
                      hole_fill_sigma_range,
                      temporal_alpha,
                      temporal_delta,
                      temporal_persistency_mode,
                      region_refine,
                      region_refine_cell_size,
                      region_refine_plane_residual_thresh,
                      algorithmControl);
};

}  // namespace dai
