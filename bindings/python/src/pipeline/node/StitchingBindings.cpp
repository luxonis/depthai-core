#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "depthai/pipeline/node/host/Stitching.hpp"
#endif

extern py::handle daiNodeModule;

void bind_stitching(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    // declare upfront
    auto stitchingNode = ADD_NODE_DERIVED(Stitching, ThreadedHostNode);
    py::enum_<Stitching::Mode> stitchingMode(stitchingNode, "Mode", DOC(dai, node, Stitching, Mode));
    py::enum_<Stitching::CameraModel> stitchingCameraModel(stitchingNode, "CameraModel", DOC(dai, node, Stitching, CameraModel));
    py::enum_<Stitching::FeaturesFinder> stitchingFeaturesFinder(stitchingNode, "FeaturesFinder", DOC(dai, node, Stitching, FeaturesFinder));
    py::enum_<Stitching::FeaturesMatcher> stitchingFeaturesMatcher(stitchingNode, "FeaturesMatcher", DOC(dai, node, Stitching, FeaturesMatcher));
    py::enum_<Stitching::Estimator> stitchingEstimator(stitchingNode, "Estimator", DOC(dai, node, Stitching, Estimator));
    py::enum_<Stitching::BundleAdjuster> stitchingBundleAdjuster(stitchingNode, "BundleAdjuster", DOC(dai, node, Stitching, BundleAdjuster));
    py::enum_<Stitching::ExposureCompensator> stitchingExposureCompensator(
        stitchingNode, "ExposureCompensator", DOC(dai, node, Stitching, ExposureCompensator));
    py::enum_<Stitching::SeamFinder> stitchingSeamFinder(stitchingNode, "SeamFinder", DOC(dai, node, Stitching, SeamFinder));
    py::enum_<Stitching::Blender> stitchingBlender(stitchingNode, "Blender", DOC(dai, node, Stitching, Blender));
    py::enum_<Stitching::WaveCorrection> stitchingWaveCorrection(stitchingNode, "WaveCorrection", DOC(dai, node, Stitching, WaveCorrection));
    py::enum_<Stitching::Interpolation> stitchingInterpolation(stitchingNode, "Interpolation", DOC(dai, node, Stitching, Interpolation));
#endif

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

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    stitchingMode.value("PANORAMA", Stitching::Mode::PANORAMA).value("PLANAR_PROJECTION", Stitching::Mode::PLANAR_PROJECTION);

    stitchingCameraModel.value("SPHERICAL", Stitching::CameraModel::SPHERICAL)
        .value("PINHOLE", Stitching::CameraModel::PINHOLE)
        .value("CYLINDRICAL", Stitching::CameraModel::CYLINDRICAL);

    stitchingFeaturesFinder.value("ORB", Stitching::FeaturesFinder::ORB)
        .value("SIFT", Stitching::FeaturesFinder::SIFT)
        .value("AKAZE", Stitching::FeaturesFinder::AKAZE)
        .value("BRISK", Stitching::FeaturesFinder::BRISK);

    stitchingFeaturesMatcher.value("HOMOGRAPHY", Stitching::FeaturesMatcher::HOMOGRAPHY).value("AFFINE", Stitching::FeaturesMatcher::AFFINE);

    stitchingEstimator.value("HOMOGRAPHY", Stitching::Estimator::HOMOGRAPHY).value("AFFINE", Stitching::Estimator::AFFINE);

    stitchingBundleAdjuster.value("NONE", Stitching::BundleAdjuster::NONE)
        .value("RAY", Stitching::BundleAdjuster::RAY)
        .value("REPROJECTION", Stitching::BundleAdjuster::REPROJECTION)
        .value("AFFINE", Stitching::BundleAdjuster::AFFINE)
        .value("AFFINE_PARTIAL", Stitching::BundleAdjuster::AFFINE_PARTIAL);

    stitchingExposureCompensator.value("NONE", Stitching::ExposureCompensator::NONE)
        .value("GAIN", Stitching::ExposureCompensator::GAIN)
        .value("GAIN_BLOCKS", Stitching::ExposureCompensator::GAIN_BLOCKS)
        .value("CHANNELS", Stitching::ExposureCompensator::CHANNELS)
        .value("CHANNELS_BLOCKS", Stitching::ExposureCompensator::CHANNELS_BLOCKS);

    stitchingSeamFinder.value("NONE", Stitching::SeamFinder::NONE)
        .value("VORONOI", Stitching::SeamFinder::VORONOI)
        .value("DP_COLOR", Stitching::SeamFinder::DP_COLOR)
        .value("DP_COLOR_GRAD", Stitching::SeamFinder::DP_COLOR_GRAD)
        .value("GRAPHCUT_COLOR", Stitching::SeamFinder::GRAPHCUT_COLOR)
        .value("GRAPHCUT_COLOR_GRAD", Stitching::SeamFinder::GRAPHCUT_COLOR_GRAD);

    stitchingBlender.value("NONE", Stitching::Blender::NONE).value("FEATHER", Stitching::Blender::FEATHER).value("MULTI_BAND", Stitching::Blender::MULTI_BAND);

    stitchingWaveCorrection.value("NONE", Stitching::WaveCorrection::NONE)
        .value("HORIZONTAL", Stitching::WaveCorrection::HORIZONTAL)
        .value("VERTICAL", Stitching::WaveCorrection::VERTICAL);

    stitchingInterpolation.value("NEAREST", Stitching::Interpolation::NEAREST)
        .value("LINEAR", Stitching::Interpolation::LINEAR)
        .value("CUBIC", Stitching::Interpolation::CUBIC)
        .value("AREA", Stitching::Interpolation::AREA)
        .value("LANCZOS4", Stitching::Interpolation::LANCZOS4);

    stitchingNode.def_readonly("sync", &Stitching::sync, DOC(dai, node, Stitching, sync))
        .def_property_readonly(
            "inputs", [](Stitching& node) { return &node.inputs; }, py::return_value_policy::reference_internal, DOC(dai, node, Stitching, inputs))
        .def_readonly("out", &Stitching::out, DOC(dai, node, Stitching, out))
        .def("build",
             static_cast<std::shared_ptr<Stitching> (Stitching::*)(const std::vector<Node::Output*>&)>(&Stitching::build),
             py::arg("sources"),
             DOC(dai, node, Stitching, build))
        .def("build",
             static_cast<std::shared_ptr<Stitching> (Stitching::*)(size_t)>(&Stitching::build),
             py::arg("numInputs"),
             DOC(dai, node, Stitching, build, 2))
        .def("getNumInputs", &Stitching::getNumInputs, DOC(dai, node, Stitching, getNumInputs))
        .def("setSyncThreshold", &Stitching::setSyncThreshold, py::arg("syncThreshold"), DOC(dai, node, Stitching, setSyncThreshold))
        .def("setMode", &Stitching::setMode, py::arg("mode"), DOC(dai, node, Stitching, setMode))
        .def("getMode", &Stitching::getMode, DOC(dai, node, Stitching, getMode))
        .def("setCameraModel", &Stitching::setCameraModel, py::arg("model"), DOC(dai, node, Stitching, setCameraModel))
        .def("getCameraModel", &Stitching::getCameraModel, DOC(dai, node, Stitching, getCameraModel))
        .def("setContinuous", &Stitching::setContinuous, py::arg("continuous"), DOC(dai, node, Stitching, setContinuous))
        .def("getContinuous", &Stitching::getContinuous, DOC(dai, node, Stitching, getContinuous))
        .def("setEstimationFrames", &Stitching::setEstimationFrames, py::arg("frames"), DOC(dai, node, Stitching, setEstimationFrames))
        .def("getEstimationFrames", &Stitching::getEstimationFrames, DOC(dai, node, Stitching, getEstimationFrames))
        .def("resetTransform", &Stitching::resetTransform, DOC(dai, node, Stitching, resetTransform))
        .def("setRegistrationResolution", &Stitching::setRegistrationResolution, py::arg("megapixels"), DOC(dai, node, Stitching, setRegistrationResolution))
        .def("getRegistrationResolution", &Stitching::getRegistrationResolution, DOC(dai, node, Stitching, getRegistrationResolution))
        .def("setSeamEstimationResolution",
             &Stitching::setSeamEstimationResolution,
             py::arg("megapixels"),
             DOC(dai, node, Stitching, setSeamEstimationResolution))
        .def("getSeamEstimationResolution", &Stitching::getSeamEstimationResolution, DOC(dai, node, Stitching, getSeamEstimationResolution))
        .def("setCompositingResolution", &Stitching::setCompositingResolution, py::arg("megapixels"), DOC(dai, node, Stitching, setCompositingResolution))
        .def("getCompositingResolution", &Stitching::getCompositingResolution, DOC(dai, node, Stitching, getCompositingResolution))
        .def("setPanoConfidenceThreshold", &Stitching::setPanoConfidenceThreshold, py::arg("threshold"), DOC(dai, node, Stitching, setPanoConfidenceThreshold))
        .def("getPanoConfidenceThreshold", &Stitching::getPanoConfidenceThreshold, DOC(dai, node, Stitching, getPanoConfidenceThreshold))
        .def("setWaveCorrection", &Stitching::setWaveCorrection, py::arg("correction"), DOC(dai, node, Stitching, setWaveCorrection))
        .def("getWaveCorrection", &Stitching::getWaveCorrection, DOC(dai, node, Stitching, getWaveCorrection))
        .def("setInterpolation", &Stitching::setInterpolation, py::arg("interpolation"), DOC(dai, node, Stitching, setInterpolation))
        .def("getInterpolation", &Stitching::getInterpolation, DOC(dai, node, Stitching, getInterpolation))
        .def("setFeaturesFinder", &Stitching::setFeaturesFinder, py::arg("finder"), DOC(dai, node, Stitching, setFeaturesFinder))
        .def("getFeaturesFinder", &Stitching::getFeaturesFinder, DOC(dai, node, Stitching, getFeaturesFinder))
        .def("setFeaturesMatcher",
             &Stitching::setFeaturesMatcher,
             py::arg("matcher"),
             py::arg("matchConf") = -1.0f,
             DOC(dai, node, Stitching, setFeaturesMatcher))
        .def("getFeaturesMatcher", &Stitching::getFeaturesMatcher, DOC(dai, node, Stitching, getFeaturesMatcher))
        .def("setEstimator", &Stitching::setEstimator, py::arg("estimator"), DOC(dai, node, Stitching, setEstimator))
        .def("getEstimator", &Stitching::getEstimator, DOC(dai, node, Stitching, getEstimator))
        .def("setBundleAdjuster", &Stitching::setBundleAdjuster, py::arg("adjuster"), DOC(dai, node, Stitching, setBundleAdjuster))
        .def("getBundleAdjuster", &Stitching::getBundleAdjuster, DOC(dai, node, Stitching, getBundleAdjuster))
        .def("setExposureCompensator", &Stitching::setExposureCompensator, py::arg("compensator"), DOC(dai, node, Stitching, setExposureCompensator))
        .def("getExposureCompensator", &Stitching::getExposureCompensator, DOC(dai, node, Stitching, getExposureCompensator))
        .def("setSeamFinder", &Stitching::setSeamFinder, py::arg("finder"), DOC(dai, node, Stitching, setSeamFinder))
        .def("getSeamFinder", &Stitching::getSeamFinder, DOC(dai, node, Stitching, getSeamFinder))
        .def("setBlender", &Stitching::setBlender, py::arg("blender"), py::arg("strength") = 5.0f, DOC(dai, node, Stitching, setBlender))
        .def("getBlender", &Stitching::getBlender, DOC(dai, node, Stitching, getBlender));
#else
    (void)m;
#endif
}
