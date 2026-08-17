#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/DeviceNode.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include "depthai/pipeline/node/host/Stitching.hpp"
#endif

extern py::handle daiNodeModule;

void bind_stitching(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    // declare upfront
    py::class_<StitchingProperties> stitchingProperties(m, "StitchingProperties", DOC(dai, StitchingProperties));
    auto stitchingNode = ADD_NODE(Stitching);
    py::enum_<Stitching::Mode> stitchingMode(stitchingNode, "Mode", DOC(dai, StitchingProperties, Mode));
    py::enum_<Stitching::CameraModel> stitchingCameraModel(stitchingNode, "CameraModel", DOC(dai, StitchingProperties, CameraModel));
    py::enum_<Stitching::SeamFinder> stitchingSeamFinder(stitchingNode, "SeamFinder", DOC(dai, StitchingProperties, SeamFinder));
    py::class_<Stitching::Plane> stitchingPlane(stitchingNode, "Plane", DOC(dai, StitchingProperties, Plane));
    py::class_<Stitching::VirtualCamera> stitchingVirtualCamera(stitchingNode, "VirtualCamera", DOC(dai, StitchingProperties, VirtualCamera));
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

    stitchingSeamFinder.value("NONE", Stitching::SeamFinder::NONE)
        .value("VORONOI", Stitching::SeamFinder::VORONOI)
        .value("DP_COLOR", Stitching::SeamFinder::DP_COLOR)
        .value("DP_COLOR_GRAD", Stitching::SeamFinder::DP_COLOR_GRAD)
        .value("GRAPHCUT_COLOR", Stitching::SeamFinder::GRAPHCUT_COLOR)
        .value("GRAPHCUT_COLOR_GRAD", Stitching::SeamFinder::GRAPHCUT_COLOR_GRAD);

    stitchingPlane.def(py::init<>())
        .def(py::init([](const Point3f& point, const Point3f& normal, LengthUnit unit) { return Stitching::Plane{point, normal, unit}; }),
             py::arg("point"),
             py::arg("normal"),
             py::arg("unit") = LengthUnit::CENTIMETER)
        .def_readwrite("point", &Stitching::Plane::point, DOC(dai, StitchingProperties, Plane, point))
        .def_readwrite("normal", &Stitching::Plane::normal, DOC(dai, StitchingProperties, Plane, normal))
        .def_readwrite("unit", &Stitching::Plane::unit, DOC(dai, StitchingProperties, Plane, unit));

    stitchingVirtualCamera.def(py::init<>())
        .def_static("lookAt",
                    &Stitching::VirtualCamera::lookAt,
                    py::arg("position"),
                    py::arg("target"),
                    py::arg("up"),
                    py::arg("hFovDegrees"),
                    py::arg("width"),
                    py::arg("height"),
                    py::arg("unit") = LengthUnit::CENTIMETER,
                    DOC(dai, StitchingProperties, VirtualCamera, lookAt))
        .def_readwrite("pose", &Stitching::VirtualCamera::pose, DOC(dai, StitchingProperties, VirtualCamera, pose))
        .def_readwrite("unit", &Stitching::VirtualCamera::unit, DOC(dai, StitchingProperties, VirtualCamera, unit))
        .def_readwrite("intrinsics", &Stitching::VirtualCamera::intrinsics, DOC(dai, StitchingProperties, VirtualCamera, intrinsics))
        .def_readwrite("width", &Stitching::VirtualCamera::width, DOC(dai, StitchingProperties, VirtualCamera, width))
        .def_readwrite("height", &Stitching::VirtualCamera::height, DOC(dai, StitchingProperties, VirtualCamera, height));

    stitchingProperties.def_readwrite("mode", &StitchingProperties::mode)
        .def_readwrite("cameraModel", &StitchingProperties::cameraModel)
        .def_readwrite("continuous", &StitchingProperties::continuous)
        .def_readwrite("estimationFrames", &StitchingProperties::estimationFrames)
        .def_readwrite("maxPanoramaWidth", &StitchingProperties::maxPanoramaWidth)
        .def_readwrite("maxPanoramaHeight", &StitchingProperties::maxPanoramaHeight)
        .def_readwrite("panoConfidenceThreshold", &StitchingProperties::panoConfidenceThreshold)
        .def_readwrite("seamFinder", &StitchingProperties::seamFinder)
        .def_readwrite("plane", &StitchingProperties::plane)
        .def_readwrite("view", &StitchingProperties::view)
        .def_readwrite("maxViewWidth", &StitchingProperties::maxViewWidth)
        .def_readwrite("maxViewHeight", &StitchingProperties::maxViewHeight)
        .def_readwrite("maxRange", &StitchingProperties::maxRange)
        .def_readwrite("minIncidenceAngle", &StitchingProperties::minIncidenceAngle);

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
        .def("setRunOnHost", &Stitching::setRunOnHost, py::arg("runOnHost"), DOC(dai, node, Stitching, setRunOnHost))
        .def("runOnHost", &Stitching::runOnHost, DOC(dai, node, Stitching, runOnHost))
        .def("setMode", &Stitching::setMode, py::arg("mode"), DOC(dai, node, Stitching, setMode))
        .def("getMode", &Stitching::getMode, DOC(dai, node, Stitching, getMode))
        .def("setPlane", static_cast<void (Stitching::*)(const Stitching::Plane&)>(&Stitching::setPlane), py::arg("plane"), DOC(dai, node, Stitching, setPlane))
        .def("setPlane",
             static_cast<void (Stitching::*)(const Point3f&, const Point3f&, LengthUnit)>(&Stitching::setPlane),
             py::arg("point"),
             py::arg("normal"),
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, node, Stitching, setPlane, 2))
        .def("getPlane", &Stitching::getPlane, DOC(dai, node, Stitching, getPlane))
        .def("setView", &Stitching::setView, py::arg("view"), DOC(dai, node, Stitching, setView))
        .def("setViewAuto", &Stitching::setViewAuto, DOC(dai, node, Stitching, setViewAuto))
        .def("getView", &Stitching::getView, DOC(dai, node, Stitching, getView))
        .def("setMaxViewSize", &Stitching::setMaxViewSize, py::arg("width"), py::arg("height"), DOC(dai, node, Stitching, setMaxViewSize))
        .def("setMaxRange", &Stitching::setMaxRange, py::arg("range"), py::arg("unit") = LengthUnit::CENTIMETER, DOC(dai, node, Stitching, setMaxRange))
        .def("getMaxRange", &Stitching::getMaxRange, py::arg("unit") = LengthUnit::CENTIMETER, DOC(dai, node, Stitching, getMaxRange))
        .def("setMinIncidenceAngle", &Stitching::setMinIncidenceAngle, py::arg("degrees"), DOC(dai, node, Stitching, setMinIncidenceAngle))
        .def("getMinIncidenceAngle", &Stitching::getMinIncidenceAngle, DOC(dai, node, Stitching, getMinIncidenceAngle))
        .def("setCameraModel", &Stitching::setCameraModel, py::arg("model"), DOC(dai, node, Stitching, setCameraModel))
        .def("getCameraModel", &Stitching::getCameraModel, DOC(dai, node, Stitching, getCameraModel))
        .def("setContinuous", &Stitching::setContinuous, py::arg("continuous"), DOC(dai, node, Stitching, setContinuous))
        .def("getContinuous", &Stitching::getContinuous, DOC(dai, node, Stitching, getContinuous))
        .def("setEstimationFrames", &Stitching::setEstimationFrames, py::arg("frames"), DOC(dai, node, Stitching, setEstimationFrames))
        .def("getEstimationFrames", &Stitching::getEstimationFrames, DOC(dai, node, Stitching, getEstimationFrames))
        .def("setMaxPanoramaSize", &Stitching::setMaxPanoramaSize, py::arg("width"), py::arg("height"), DOC(dai, node, Stitching, setMaxPanoramaSize))
        .def("resetTransform", &Stitching::resetTransform, DOC(dai, node, Stitching, resetTransform))
        .def("setPanoConfidenceThreshold", &Stitching::setPanoConfidenceThreshold, py::arg("threshold"), DOC(dai, node, Stitching, setPanoConfidenceThreshold))
        .def("getPanoConfidenceThreshold", &Stitching::getPanoConfidenceThreshold, DOC(dai, node, Stitching, getPanoConfidenceThreshold))
        .def("setSeamFinder", &Stitching::setSeamFinder, py::arg("finder"), DOC(dai, node, Stitching, setSeamFinder))
        .def("getSeamFinder", &Stitching::getSeamFinder, DOC(dai, node, Stitching, getSeamFinder));
    daiNodeModule.attr("Stitching").attr("Properties") = stitchingProperties;
#else
    (void)m;
#endif
}
