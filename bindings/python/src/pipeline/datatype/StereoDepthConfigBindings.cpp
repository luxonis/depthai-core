#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_stereodepthconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawStereoDepthConfig, RawBuffer, std::shared_ptr<RawStereoDepthConfig>> rawStereoDepthConfig(m, "RawStereoDepthConfig", DOC(dai,
    // RawStereoDepthConfig));
    py::class_<StereoDepthConfig, Py<StereoDepthConfig>, Buffer, std::shared_ptr<StereoDepthConfig>> stereoDepthConfig(
        m, "StereoDepthConfig", DOC(dai, StereoDepthConfig));
    py::enum_<StereoDepthConfig::MedianFilter> medianFilter(m, "MedianFilter", DOC(dai, MedianFilter));
    py::class_<StereoDepthConfig::ConfidenceMetrics> _ConfidenceMetrics(stereoDepthConfig, "ConfidenceMetrics", DOC(dai, StereoDepthConfig, ConfidenceMetrics));
    py::class_<StereoDepthConfig::AlgorithmControl> algorithmControl(stereoDepthConfig, "AlgorithmControl", DOC(dai, StereoDepthConfig, AlgorithmControl));
    py::enum_<StereoDepthConfig::AlgorithmControl::DepthAlign> depthAlign(
        algorithmControl, "DepthAlign", DOC(dai, StereoDepthConfig, AlgorithmControl, DepthAlign));
    py::enum_<StereoDepthConfig::AlgorithmControl::DepthUnit> depthUnit(
        algorithmControl, "DepthUnit", DOC(dai, StereoDepthConfig, AlgorithmControl, DepthUnit));
    py::class_<StereoDepthConfig::PostProcessing> postProcessing(stereoDepthConfig, "PostProcessing", DOC(dai, StereoDepthConfig, PostProcessing));
    py::class_<StereoDepthConfig::PostProcessing::SpatialFilter> spatialFilter(
        postProcessing, "SpatialFilter", DOC(dai, StereoDepthConfig, PostProcessing, SpatialFilter));
    py::class_<StereoDepthConfig::PostProcessing::TemporalFilter> temporalFilter(
        postProcessing, "TemporalFilter", DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter));
    py::enum_<StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode> persistencyMode(
        temporalFilter, "PersistencyMode", DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode));
    py::class_<StereoDepthConfig::PostProcessing::ThresholdFilter> thresholdFilter(
        postProcessing, "ThresholdFilter", DOC(dai, StereoDepthConfig, PostProcessing, ThresholdFilter));
    py::class_<StereoDepthConfig::PostProcessing::BrightnessFilter> brightnessFilter(
        postProcessing, "BrightnessFilter", DOC(dai, StereoDepthConfig, PostProcessing, BrightnessFilter));
    py::class_<StereoDepthConfig::PostProcessing::SpeckleFilter> speckleFilter(
        postProcessing, "SpeckleFilter", DOC(dai, StereoDepthConfig, PostProcessing, SpeckleFilter));
    py::class_<StereoDepthConfig::PostProcessing::DecimationFilter> decimationFilter(
        postProcessing, "DecimationFilter", DOC(dai, StereoDepthConfig, PostProcessing, DecimationFilter));
    py::enum_<StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode> decimationMode(
        decimationFilter, "DecimationMode", DOC(dai, StereoDepthConfig, PostProcessing, DecimationFilter, DecimationMode));
    py::class_<StereoDepthConfig::PostProcessing::HoleFilling> _HoleFilling(
        postProcessing, "HoleFilling", DOC(dai, StereoDepthConfig, PostProcessing, HoleFilling));
    py::class_<StereoDepthConfig::PostProcessing::AdaptiveMedianFilter> _AdaptiveMedianFilter(
        postProcessing, "AdaptiveMedianFilter", DOC(dai, StereoDepthConfig, PostProcessing, AdaptiveMedianFilter));
    py::class_<StereoDepthConfig::CostAggregation> costAggregation(stereoDepthConfig, "CostAggregation", DOC(dai, StereoDepthConfig, CostAggregation));
    py::class_<StereoDepthConfig::CostAggregation::P1Config> _P1Config(costAggregation, "P1Config", DOC(dai, StereoDepthConfig, CostAggregation, P1Config));
    py::class_<StereoDepthConfig::CostAggregation::P2Config> _P2Config(costAggregation, "P2Config", DOC(dai, StereoDepthConfig, CostAggregation, P2Config));
    py::class_<StereoDepthConfig::CostMatching> costMatching(stereoDepthConfig, "CostMatching", DOC(dai, StereoDepthConfig, CostMatching));
    py::class_<StereoDepthConfig::CostMatching::LinearEquationParameters> costMatchingLinearEquationParameters(
        costMatching, "LinearEquationParameters", DOC(dai, StereoDepthConfig, CostMatching, LinearEquationParameters));
    py::enum_<StereoDepthConfig::CostMatching::DisparityWidth> costMatchingDisparityWidth(
        costMatching, "DisparityWidth", DOC(dai, StereoDepthConfig, CostMatching, DisparityWidth));
    py::class_<StereoDepthConfig::CensusTransform> censusTransform(stereoDepthConfig, "CensusTransform", DOC(dai, StereoDepthConfig, CensusTransform));
    py::enum_<StereoDepthConfig::CensusTransform::KernelSize> censusTransformKernelSize(
        censusTransform, "KernelSize", DOC(dai, StereoDepthConfig, CensusTransform, KernelSize));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Metadata / raw
    medianFilter.value("MEDIAN_OFF", StereoDepthConfig::MedianFilter::MEDIAN_OFF)
        .value("KERNEL_3x3", StereoDepthConfig::MedianFilter::KERNEL_3x3)
        .value("KERNEL_5x5", StereoDepthConfig::MedianFilter::KERNEL_5x5)
        .value("KERNEL_7x7", StereoDepthConfig::MedianFilter::KERNEL_7x7);
    // m.attr("StereoDepthProperties").attr("MedianFilter") = medianFilter;

    depthAlign
        .value("RECTIFIED_RIGHT",
               StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT,
               DOC(dai, StereoDepthConfig, AlgorithmControl, DepthAlign, RECTIFIED_RIGHT))
        .value("RECTIFIED_LEFT",
               StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT,
               DOC(dai, StereoDepthConfig, AlgorithmControl, DepthAlign, RECTIFIED_LEFT))
        .value("CENTER", StereoDepthConfig::AlgorithmControl::DepthAlign::CENTER, DOC(dai, StereoDepthConfig, AlgorithmControl, DepthAlign, CENTER));

    // m.attr("StereoDepthProperties").attr("DepthAlign") = depthAlign;

    depthUnit.value("METER", StereoDepthConfig::AlgorithmControl::DepthUnit::METER, DOC(dai, StereoDepthConfig, AlgorithmControl, DepthUnit, METER))
        .value("CENTIMETER", StereoDepthConfig::AlgorithmControl::DepthUnit::CENTIMETER, DOC(dai, StereoDepthConfig, AlgorithmControl, DepthUnit, CENTIMETER))
        .value("MILLIMETER", StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER, DOC(dai, StereoDepthConfig, AlgorithmControl, DepthUnit, MILLIMETER))
        .value("INCH", StereoDepthConfig::AlgorithmControl::DepthUnit::INCH, DOC(dai, StereoDepthConfig, AlgorithmControl, DepthUnit, INCH))
        .value("FOOT", StereoDepthConfig::AlgorithmControl::DepthUnit::FOOT, DOC(dai, StereoDepthConfig, AlgorithmControl, DepthUnit, FOOT))
        .value("CUSTOM", StereoDepthConfig::AlgorithmControl::DepthUnit::CUSTOM, DOC(dai, StereoDepthConfig, AlgorithmControl, DepthUnit, CUSTOM));

    algorithmControl.def(py::init<>())
        .def_readwrite("depthAlign", &StereoDepthConfig::AlgorithmControl::depthAlign, DOC(dai, StereoDepthConfig, AlgorithmControl, depthAlign))
        .def_readwrite("depthUnit", &StereoDepthConfig::AlgorithmControl::depthUnit, DOC(dai, StereoDepthConfig, AlgorithmControl, depthUnit))
        .def_readwrite("customDepthUnitMultiplier",
                       &StereoDepthConfig::AlgorithmControl::customDepthUnitMultiplier,
                       DOC(dai, StereoDepthConfig, AlgorithmControl, customDepthUnitMultiplier))
        .def_readwrite("enableLeftRightCheck",
                       &StereoDepthConfig::AlgorithmControl::enableLeftRightCheck,
                       DOC(dai, StereoDepthConfig, AlgorithmControl, enableLeftRightCheck))
        .def_readwrite("enableSwLeftRightCheck",
                       &StereoDepthConfig::AlgorithmControl::enableSwLeftRightCheck,
                       DOC(dai, StereoDepthConfig, AlgorithmControl, enableSwLeftRightCheck))
        .def_readwrite("enableExtended", &StereoDepthConfig::AlgorithmControl::enableExtended, DOC(dai, StereoDepthConfig, AlgorithmControl, enableExtended))
        .def_readwrite("enableSubpixel", &StereoDepthConfig::AlgorithmControl::enableSubpixel, DOC(dai, StereoDepthConfig, AlgorithmControl, enableSubpixel))
        .def_readwrite("leftRightCheckThreshold",
                       &StereoDepthConfig::AlgorithmControl::leftRightCheckThreshold,
                       DOC(dai, StereoDepthConfig, AlgorithmControl, leftRightCheckThreshold))
        .def_readwrite("subpixelFractionalBits",
                       &StereoDepthConfig::AlgorithmControl::subpixelFractionalBits,
                       DOC(dai, StereoDepthConfig, AlgorithmControl, subpixelFractionalBits))
        .def_readwrite("disparityShift", &StereoDepthConfig::AlgorithmControl::disparityShift, DOC(dai, StereoDepthConfig, AlgorithmControl, disparityShift))
        .def_readwrite("centerAlignmentShiftFactor",
                       &StereoDepthConfig::AlgorithmControl::centerAlignmentShiftFactor,
                       DOC(dai, StereoDepthConfig, AlgorithmControl, centerAlignmentShiftFactor))
        .def_readwrite("numInvalidateEdgePixels",
                       &StereoDepthConfig::AlgorithmControl::numInvalidateEdgePixels,
                       DOC(dai, StereoDepthConfig, AlgorithmControl, numInvalidateEdgePixels));

    _ConfidenceMetrics.def(py::init<>())
        .def_readwrite("occlusionConfidenceWeight",
                       &StereoDepthConfig::ConfidenceMetrics::occlusionConfidenceWeight,
                       DOC(dai, StereoDepthConfig, ConfidenceMetrics, occlusionConfidenceWeight))
        .def_readwrite("motionVectorConfidenceWeight",
                       &StereoDepthConfig::ConfidenceMetrics::motionVectorConfidenceWeight,
                       DOC(dai, StereoDepthConfig, ConfidenceMetrics, motionVectorConfidenceWeight))
        .def_readwrite("motionVectorConfidenceThreshold",
                       &StereoDepthConfig::ConfidenceMetrics::motionVectorConfidenceThreshold,
                       DOC(dai, StereoDepthConfig, ConfidenceMetrics, motionVectorConfidenceThreshold))
        .def_readwrite("flatnessConfidenceWeight",
                       &StereoDepthConfig::ConfidenceMetrics::flatnessConfidenceWeight,
                       DOC(dai, StereoDepthConfig, ConfidenceMetrics, flatnessConfidenceWeight))
        .def_readwrite("flatnessConfidenceThreshold",
                       &StereoDepthConfig::ConfidenceMetrics::flatnessConfidenceThreshold,
                       DOC(dai, StereoDepthConfig, ConfidenceMetrics, flatnessConfidenceThreshold))
        .def_readwrite(
            "flatnessOverride", &StereoDepthConfig::ConfidenceMetrics::flatnessOverride, DOC(dai, StereoDepthConfig, ConfidenceMetrics, flatnessOverride));

    spatialFilter.def(py::init<>())
        .def_readwrite("enable", &StereoDepthConfig::PostProcessing::SpatialFilter::enable, DOC(dai, StereoDepthConfig, PostProcessing, SpatialFilter, enable))
        .def_readwrite("holeFillingRadius",
                       &StereoDepthConfig::PostProcessing::SpatialFilter::holeFillingRadius,
                       DOC(dai, StereoDepthConfig, PostProcessing, SpatialFilter, holeFillingRadius))
        .def_readwrite("alpha", &StereoDepthConfig::PostProcessing::SpatialFilter::alpha, DOC(dai, StereoDepthConfig, PostProcessing, SpatialFilter, alpha))
        .def_readwrite("delta", &StereoDepthConfig::PostProcessing::SpatialFilter::delta, DOC(dai, StereoDepthConfig, PostProcessing, SpatialFilter, delta))
        .def_readwrite("numIterations",
                       &StereoDepthConfig::PostProcessing::SpatialFilter::numIterations,
                       DOC(dai, StereoDepthConfig, PostProcessing, SpatialFilter, numIterations));

    persistencyMode
        .value("PERSISTENCY_OFF",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::PERSISTENCY_OFF,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, PERSISTENCY_OFF))
        .value("VALID_8_OUT_OF_8",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_8_OUT_OF_8,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, VALID_8_OUT_OF_8))
        .value("VALID_2_IN_LAST_3",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_3,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, VALID_2_IN_LAST_3))
        .value("VALID_2_IN_LAST_4",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_IN_LAST_4,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, VALID_2_IN_LAST_4))
        .value("VALID_2_OUT_OF_8",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_2_OUT_OF_8,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, VALID_2_OUT_OF_8))
        .value("VALID_1_IN_LAST_2",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_2,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, VALID_1_IN_LAST_2))
        .value("VALID_1_IN_LAST_5",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_5,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, VALID_1_IN_LAST_5))
        .value("VALID_1_IN_LAST_8",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::VALID_1_IN_LAST_8,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, VALID_1_IN_LAST_8))
        .value("PERSISTENCY_INDEFINITELY",
               StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode::PERSISTENCY_INDEFINITELY,
               DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, PersistencyMode, PERSISTENCY_INDEFINITELY));

    temporalFilter.def(py::init<>())
        .def_readwrite(
            "enable", &StereoDepthConfig::PostProcessing::TemporalFilter::enable, DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, enable))
        .def_readwrite("persistencyMode",
                       &StereoDepthConfig::PostProcessing::TemporalFilter::persistencyMode,
                       DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, persistencyMode))
        .def_readwrite("alpha", &StereoDepthConfig::PostProcessing::TemporalFilter::alpha, DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, alpha))
        .def_readwrite("delta", &StereoDepthConfig::PostProcessing::TemporalFilter::delta, DOC(dai, StereoDepthConfig, PostProcessing, TemporalFilter, delta));

    thresholdFilter.def(py::init<>())
        .def_readwrite(
            "minRange", &StereoDepthConfig::PostProcessing::ThresholdFilter::minRange, DOC(dai, StereoDepthConfig, PostProcessing, ThresholdFilter, minRange))
        .def_readwrite(
            "maxRange", &StereoDepthConfig::PostProcessing::ThresholdFilter::maxRange, DOC(dai, StereoDepthConfig, PostProcessing, ThresholdFilter, maxRange));

    brightnessFilter.def(py::init<>())
        .def_readwrite("minBrightness",
                       &StereoDepthConfig::PostProcessing::BrightnessFilter::minBrightness,
                       DOC(dai, StereoDepthConfig, PostProcessing, BrightnessFilter, minBrightness))
        .def_readwrite("maxBrightness",
                       &StereoDepthConfig::PostProcessing::BrightnessFilter::maxBrightness,
                       DOC(dai, StereoDepthConfig, PostProcessing, BrightnessFilter, maxBrightness));

    speckleFilter.def(py::init<>())
        .def_readwrite("enable", &StereoDepthConfig::PostProcessing::SpeckleFilter::enable, DOC(dai, StereoDepthConfig, PostProcessing, SpeckleFilter, enable))
        .def_readwrite("speckleRange",
                       &StereoDepthConfig::PostProcessing::SpeckleFilter::speckleRange,
                       DOC(dai, StereoDepthConfig, PostProcessing, SpeckleFilter, speckleRange))
        .def_readwrite("differenceThreshold",
                       &StereoDepthConfig::PostProcessing::SpeckleFilter::differenceThreshold,
                       DOC(dai, StereoDepthConfig, PostProcessing, SpeckleFilter, differenceThreshold));

    decimationMode
        .value("PIXEL_SKIPPING",
               StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::PIXEL_SKIPPING,
               DOC(dai, StereoDepthConfig, PostProcessing, DecimationFilter, DecimationMode, PIXEL_SKIPPING))
        .value("NON_ZERO_MEDIAN",
               StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEDIAN,
               DOC(dai, StereoDepthConfig, PostProcessing, DecimationFilter, DecimationMode, NON_ZERO_MEDIAN))
        .value("NON_ZERO_MEAN",
               StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode::NON_ZERO_MEAN,
               DOC(dai, StereoDepthConfig, PostProcessing, DecimationFilter, DecimationMode, NON_ZERO_MEAN));

    decimationFilter.def(py::init<>())
        .def_readwrite("decimationFactor",
                       &StereoDepthConfig::PostProcessing::DecimationFilter::decimationFactor,
                       DOC(dai, StereoDepthConfig, PostProcessing, DecimationFilter, decimationFactor))
        .def_readwrite("decimationMode",
                       &StereoDepthConfig::PostProcessing::DecimationFilter::decimationMode,
                       DOC(dai, StereoDepthConfig, PostProcessing, DecimationFilter, decimationMode));

    _HoleFilling.def(py::init<>())
        .def_readwrite("enable", &StereoDepthConfig::PostProcessing::HoleFilling::enable, DOC(dai, StereoDepthConfig, PostProcessing, HoleFilling, enable))
        .def_readwrite("highConfidenceThreshold",
                       &StereoDepthConfig::PostProcessing::HoleFilling::highConfidenceThreshold,
                       DOC(dai, StereoDepthConfig, PostProcessing, HoleFilling, highConfidenceThreshold))
        .def_readwrite("fillConfidenceThreshold",
                       &StereoDepthConfig::PostProcessing::HoleFilling::fillConfidenceThreshold,
                       DOC(dai, StereoDepthConfig, PostProcessing, HoleFilling, fillConfidenceThreshold))
        .def_readwrite("minValidDisparity",
                       &StereoDepthConfig::PostProcessing::HoleFilling::minValidDisparity,
                       DOC(dai, StereoDepthConfig, PostProcessing, HoleFilling, minValidDisparity))
        .def_readwrite("invalidateDisparities",
                       &StereoDepthConfig::PostProcessing::HoleFilling::invalidateDisparities,
                       DOC(dai, StereoDepthConfig, PostProcessing, HoleFilling, invalidateDisparities));

    _AdaptiveMedianFilter.def(py::init<>())
        .def_readwrite("enable",
                       &StereoDepthConfig::PostProcessing::AdaptiveMedianFilter::enable,
                       DOC(dai, StereoDepthConfig, PostProcessing, AdaptiveMedianFilter, enable))
        .def_readwrite("confidenceThreshold",
                       &StereoDepthConfig::PostProcessing::AdaptiveMedianFilter::confidenceThreshold,
                       DOC(dai, StereoDepthConfig, PostProcessing, AdaptiveMedianFilter, confidenceThreshold));

    postProcessing.def(py::init<>())
        .def_readwrite("filteringOrder", &StereoDepthConfig::PostProcessing::filteringOrder, DOC(dai, StereoDepthConfig, PostProcessing, filteringOrder))
        .def_readwrite("median", &StereoDepthConfig::PostProcessing::median, DOC(dai, StereoDepthConfig, PostProcessing, median))
        .def_readwrite(
            "bilateralSigmaValue", &StereoDepthConfig::PostProcessing::bilateralSigmaValue, DOC(dai, StereoDepthConfig, PostProcessing, bilateralSigmaValue))
        .def_readwrite("spatialFilter", &StereoDepthConfig::PostProcessing::spatialFilter, DOC(dai, StereoDepthConfig, PostProcessing, spatialFilter))
        .def_readwrite("temporalFilter", &StereoDepthConfig::PostProcessing::temporalFilter, DOC(dai, StereoDepthConfig, PostProcessing, temporalFilter))
        .def_readwrite("thresholdFilter", &StereoDepthConfig::PostProcessing::thresholdFilter, DOC(dai, StereoDepthConfig, PostProcessing, thresholdFilter))
        .def_readwrite("brightnessFilter", &StereoDepthConfig::PostProcessing::brightnessFilter, DOC(dai, StereoDepthConfig, PostProcessing, brightnessFilter))
        .def_readwrite("speckleFilter", &StereoDepthConfig::PostProcessing::speckleFilter, DOC(dai, StereoDepthConfig, PostProcessing, speckleFilter))
        .def_readwrite("decimationFilter", &StereoDepthConfig::PostProcessing::decimationFilter, DOC(dai, StereoDepthConfig, PostProcessing, decimationFilter))
        .def_readwrite(
            "adaptiveMedianFilter", &StereoDepthConfig::PostProcessing::adaptiveMedianFilter, DOC(dai, StereoDepthConfig, PostProcessing, adaptiveMedianFilter))
        .def_readwrite("holeFilling", &StereoDepthConfig::PostProcessing::holeFilling, DOC(dai, StereoDepthConfig, PostProcessing, holeFilling));

    // KernelSize
    censusTransformKernelSize
        .value("AUTO", StereoDepthConfig::CensusTransform::KernelSize::AUTO, DOC(dai, StereoDepthConfig, CensusTransform, KernelSize, AUTO))
        .value("KERNEL_5x5", StereoDepthConfig::CensusTransform::KernelSize::KERNEL_5x5, DOC(dai, StereoDepthConfig, CensusTransform, KernelSize, KERNEL_5x5))
        .value("KERNEL_7x7", StereoDepthConfig::CensusTransform::KernelSize::KERNEL_7x7, DOC(dai, StereoDepthConfig, CensusTransform, KernelSize, KERNEL_7x7))
        .value("KERNEL_7x9", StereoDepthConfig::CensusTransform::KernelSize::KERNEL_7x9, DOC(dai, StereoDepthConfig, CensusTransform, KernelSize, KERNEL_7x9));

    censusTransform.def(py::init<>())
        .def_readwrite("kernelSize", &StereoDepthConfig::CensusTransform::kernelSize, DOC(dai, StereoDepthConfig, CensusTransform, kernelSize))
        .def_readwrite("kernelMask", &StereoDepthConfig::CensusTransform::kernelMask, DOC(dai, StereoDepthConfig, CensusTransform, kernelMask))
        .def_readwrite("enableMeanMode", &StereoDepthConfig::CensusTransform::enableMeanMode, DOC(dai, StereoDepthConfig, CensusTransform, enableMeanMode))
        .def_readwrite("threshold", &StereoDepthConfig::CensusTransform::threshold, DOC(dai, StereoDepthConfig, CensusTransform, threshold))
        .def_readwrite("noiseThresholdOffset",
                       &StereoDepthConfig::CensusTransform::noiseThresholdOffset,
                       DOC(dai, StereoDepthConfig, CensusTransform, noiseThresholdOffset))
        .def_readwrite(
            "noiseThresholdScale", &StereoDepthConfig::CensusTransform::noiseThresholdScale, DOC(dai, StereoDepthConfig, CensusTransform, noiseThresholdScale));

    costMatchingLinearEquationParameters.def(py::init<>())
        .def_readwrite("alpha",
                       &StereoDepthConfig::CostMatching::LinearEquationParameters::alpha,
                       DOC(dai, StereoDepthConfig, CostMatching, LinearEquationParameters, alpha))
        .def_readwrite(
            "beta", &StereoDepthConfig::CostMatching::LinearEquationParameters::beta, DOC(dai, StereoDepthConfig, CostMatching, LinearEquationParameters, beta))
        .def_readwrite("threshold",
                       &StereoDepthConfig::CostMatching::LinearEquationParameters::threshold,
                       DOC(dai, StereoDepthConfig, CostMatching, LinearEquationParameters, threshold));

    // Disparity width
    costMatchingDisparityWidth
        .value("DISPARITY_64",
               StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_64,
               DOC(dai, StereoDepthConfig, CostMatching, DisparityWidth, DISPARITY_64))
        .value("DISPARITY_96",
               StereoDepthConfig::CostMatching::DisparityWidth::DISPARITY_96,
               DOC(dai, StereoDepthConfig, CostMatching, DisparityWidth, DISPARITY_96));

    costMatching.def(py::init<>())
        .def_readwrite("disparityWidth", &StereoDepthConfig::CostMatching::disparityWidth, DOC(dai, StereoDepthConfig, CostMatching, disparityWidth))
        .def_readwrite("enableCompanding", &StereoDepthConfig::CostMatching::enableCompanding, DOC(dai, StereoDepthConfig, CostMatching, enableCompanding))
        .def_readwrite(
            "invalidDisparityValue", &StereoDepthConfig::CostMatching::invalidDisparityValue, DOC(dai, StereoDepthConfig, CostMatching, invalidDisparityValue))
        .def_readwrite(
            "confidenceThreshold", &StereoDepthConfig::CostMatching::confidenceThreshold, DOC(dai, StereoDepthConfig, CostMatching, confidenceThreshold))
        .def_readwrite("enableSwConfidenceThresholding",
                       &StereoDepthConfig::CostMatching::enableSwConfidenceThresholding,
                       DOC(dai, StereoDepthConfig, CostMatching, enableSwConfidenceThresholding))
        .def_readwrite("linearEquationParameters",
                       &StereoDepthConfig::CostMatching::linearEquationParameters,
                       DOC(dai, StereoDepthConfig, CostMatching, linearEquationParameters));

    _P1Config.def(py::init<>())
        .def_readwrite("enableAdaptive",
                       &StereoDepthConfig::CostAggregation::P1Config::enableAdaptive,
                       DOC(dai, StereoDepthConfig, CostAggregation, P1Config, enableAdaptive))
        .def_readwrite(
            "defaultValue", &StereoDepthConfig::CostAggregation::P1Config::defaultValue, DOC(dai, StereoDepthConfig, CostAggregation, P1Config, defaultValue))
        .def_readwrite("edgeValue", &StereoDepthConfig::CostAggregation::P1Config::edgeValue, DOC(dai, StereoDepthConfig, CostAggregation, P1Config, edgeValue))
        .def_readwrite(
            "smoothValue", &StereoDepthConfig::CostAggregation::P1Config::smoothValue, DOC(dai, StereoDepthConfig, CostAggregation, P1Config, smoothValue))
        .def_readwrite("edgeThreshold",
                       &StereoDepthConfig::CostAggregation::P1Config::edgeThreshold,
                       DOC(dai, StereoDepthConfig, CostAggregation, P1Config, edgeThreshold))
        .def_readwrite("smoothThreshold",
                       &StereoDepthConfig::CostAggregation::P1Config::smoothThreshold,
                       DOC(dai, StereoDepthConfig, CostAggregation, P1Config, smoothThreshold));

    _P2Config.def(py::init<>())
        .def_readwrite("enableAdaptive",
                       &StereoDepthConfig::CostAggregation::P2Config::enableAdaptive,
                       DOC(dai, StereoDepthConfig, CostAggregation, P2Config, enableAdaptive))
        .def_readwrite(
            "defaultValue", &StereoDepthConfig::CostAggregation::P2Config::defaultValue, DOC(dai, StereoDepthConfig, CostAggregation, P2Config, defaultValue))
        .def_readwrite("edgeValue", &StereoDepthConfig::CostAggregation::P2Config::edgeValue, DOC(dai, StereoDepthConfig, CostAggregation, P2Config, edgeValue))
        .def_readwrite(
            "smoothValue", &StereoDepthConfig::CostAggregation::P2Config::smoothValue, DOC(dai, StereoDepthConfig, CostAggregation, P2Config, smoothValue));

    costAggregation.def(py::init<>())
        .def_readwrite("divisionFactor", &StereoDepthConfig::CostAggregation::divisionFactor, DOC(dai, StereoDepthConfig, CostAggregation, divisionFactor))
        .def_readwrite("horizontalPenaltyCostP1",
                       &StereoDepthConfig::CostAggregation::horizontalPenaltyCostP1,
                       DOC(dai, StereoDepthConfig, CostAggregation, horizontalPenaltyCostP1))
        .def_readwrite("horizontalPenaltyCostP2",
                       &StereoDepthConfig::CostAggregation::horizontalPenaltyCostP2,
                       DOC(dai, StereoDepthConfig, CostAggregation, horizontalPenaltyCostP2))
        .def_readwrite("verticalPenaltyCostP1",
                       &StereoDepthConfig::CostAggregation::verticalPenaltyCostP1,
                       DOC(dai, StereoDepthConfig, CostAggregation, verticalPenaltyCostP1))
        .def_readwrite("verticalPenaltyCostP2",
                       &StereoDepthConfig::CostAggregation::verticalPenaltyCostP2,
                       DOC(dai, StereoDepthConfig, CostAggregation, verticalPenaltyCostP2))
        .def_readwrite("p1Config", &StereoDepthConfig::CostAggregation::p1Config, DOC(dai, StereoDepthConfig, CostAggregation, p1Config))
        .def_readwrite("p2Config", &StereoDepthConfig::CostAggregation::p2Config, DOC(dai, StereoDepthConfig, CostAggregation, p2Config));

    stereoDepthConfig.def(py::init<>())
        .def_readwrite("algorithmControl", &StereoDepthConfig::algorithmControl, DOC(dai, StereoDepthConfig, algorithmControl))
        .def_readwrite("postProcessing", &StereoDepthConfig::postProcessing, DOC(dai, StereoDepthConfig, postProcessing))
        .def_readwrite("censusTransform", &StereoDepthConfig::censusTransform, DOC(dai, StereoDepthConfig, censusTransform))
        .def_readwrite("costMatching", &StereoDepthConfig::costMatching, DOC(dai, StereoDepthConfig, costMatching))
        .def_readwrite("costAggregation", &StereoDepthConfig::costAggregation, DOC(dai, StereoDepthConfig, costAggregation))
        .def_readwrite("confidenceMetrics", &StereoDepthConfig::confidenceMetrics, DOC(dai, StereoDepthConfig, confidenceMetrics));
    // legacy
    m.attr("StereoDepthConfig").attr("MedianFilter") = medianFilter;

    // Message
    stereoDepthConfig.def(py::init<>())
        .def("__repr__", &StereoDepthConfig::str)
        .def("setDepthAlign", &StereoDepthConfig::setDepthAlign, py::arg("align"), DOC(dai, StereoDepthConfig, setDepthAlign))
        .def("setConfidenceThreshold", &StereoDepthConfig::setConfidenceThreshold, py::arg("confThr"), DOC(dai, StereoDepthConfig, setConfidenceThreshold))
        .def("setMedianFilter", &StereoDepthConfig::setMedianFilter, py::arg("median"), DOC(dai, StereoDepthConfig, setMedianFilter))
        .def("setBilateralFilterSigma", &StereoDepthConfig::setBilateralFilterSigma, py::arg("sigma"), DOC(dai, StereoDepthConfig, setBilateralFilterSigma))
        .def("setLeftRightCheckThreshold",
             &StereoDepthConfig::setLeftRightCheckThreshold,
             py::arg("sigma"),
             DOC(dai, StereoDepthConfig, setLeftRightCheckThreshold))
        .def("getConfidenceThreshold", &StereoDepthConfig::getConfidenceThreshold, DOC(dai, StereoDepthConfig, getConfidenceThreshold))
        .def("getMedianFilter", &StereoDepthConfig::getMedianFilter, DOC(dai, StereoDepthConfig, getMedianFilter))
        .def("getBilateralFilterSigma", &StereoDepthConfig::getBilateralFilterSigma, DOC(dai, StereoDepthConfig, getBilateralFilterSigma))
        .def("getLeftRightCheckThreshold", &StereoDepthConfig::getLeftRightCheckThreshold, DOC(dai, StereoDepthConfig, getLeftRightCheckThreshold))
        .def("setLeftRightCheck", &StereoDepthConfig::setLeftRightCheck, py::arg("enable"), DOC(dai, StereoDepthConfig, setLeftRightCheck))
        .def("getLeftRightCheck", &StereoDepthConfig::getLeftRightCheck, DOC(dai, StereoDepthConfig, getLeftRightCheck))
        .def("setExtendedDisparity", &StereoDepthConfig::setExtendedDisparity, py::arg("enable"), DOC(dai, StereoDepthConfig, setExtendedDisparity))
        .def("getExtendedDisparity", &StereoDepthConfig::getExtendedDisparity, DOC(dai, StereoDepthConfig, getExtendedDisparity))
        .def("setSubpixel", &StereoDepthConfig::setSubpixel, py::arg("enable"), DOC(dai, StereoDepthConfig, setSubpixel))
        .def("getSubpixel", &StereoDepthConfig::getSubpixel, DOC(dai, StereoDepthConfig, getSubpixel))
        .def("setSubpixelFractionalBits",
             &StereoDepthConfig::setSubpixelFractionalBits,
             py::arg("subpixelFractionalBits"),
             DOC(dai, StereoDepthConfig, setSubpixelFractionalBits))
        .def("getSubpixelFractionalBits", &StereoDepthConfig::getSubpixelFractionalBits, DOC(dai, StereoDepthConfig, getSubpixelFractionalBits))
        .def("getMaxDisparity", &StereoDepthConfig::getMaxDisparity, DOC(dai, StereoDepthConfig, getMaxDisparity))
        .def("setDepthUnit", &StereoDepthConfig::setDepthUnit, DOC(dai, StereoDepthConfig, setDepthUnit))
        .def("getDepthUnit", &StereoDepthConfig::getDepthUnit, DOC(dai, StereoDepthConfig, getDepthUnit))
        .def("setDisparityShift", &StereoDepthConfig::setDisparityShift, DOC(dai, StereoDepthConfig, setDisparityShift))
        .def("setNumInvalidateEdgePixels", &StereoDepthConfig::setNumInvalidateEdgePixels, DOC(dai, StereoDepthConfig, setNumInvalidateEdgePixels))
        // .def("set",                     &StereoDepthConfig::set, py::arg("config"), DOC(dai, StereoDepthConfig, set))
        // .def("get",                     &StereoDepthConfig::get, DOC(dai, StereoDepthConfig, get))
        .def("setFiltersComputeBackend",
             &StereoDepthConfig::setFiltersComputeBackend,
             py::arg("filtersBackend"),
             DOC(dai, StereoDepthConfig, setFiltersComputeBackend))
        .def("getFiltersComputeBackend", &StereoDepthConfig::getFiltersComputeBackend, DOC(dai, StereoDepthConfig, getFiltersComputeBackend));
    m.attr("StereoDepthConfig").attr("MedianFilter") = medianFilter;
    m.attr("StereoDepthConfig").attr("AlgorithmControl") = algorithmControl;
    m.attr("StereoDepthConfig").attr("PostProcessing") = postProcessing;
    m.attr("StereoDepthConfig").attr("CensusTransform") = censusTransform;
    m.attr("StereoDepthConfig").attr("CostMatching") = costMatching;
    m.attr("StereoDepthConfig").attr("CostAggregation") = costAggregation;
}
