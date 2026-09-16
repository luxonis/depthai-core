#include <pybind11/chrono.h>
#include <pybind11/stl.h>

#include "depthai/beta/node/Stitching.hpp"
#include "depthai/pipeline/DeviceNode.hpp"
#include "pipeline/node/Common.hpp"

void bind_beta_stitching(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using dai::beta::StitchingProperties;
    using dai::beta::node::Stitching;

    // declare upfront
    auto betaModule = m.def_submodule("beta", "Experimental APIs");
    py::class_<StitchingProperties> stitchingProperties(betaModule, "StitchingProperties", DOC(dai, beta, StitchingProperties));
    auto stitchingNode = ADD_BETA_NODE_DERIVED(Stitching, dai::DeviceNode);
    py::enum_<Stitching::Mode> stitchingMode(stitchingNode, "Mode", DOC(dai, beta, StitchingProperties, Mode));
    py::enum_<Stitching::CameraModel> stitchingCameraModel(stitchingNode, "CameraModel", DOC(dai, beta, StitchingProperties, CameraModel));
    py::enum_<Stitching::SeamFinder> stitchingSeamFinder(stitchingNode, "SeamFinder", DOC(dai, beta, StitchingProperties, SeamFinder));
    py::class_<Stitching::Plane> stitchingPlane(stitchingNode, "Plane", DOC(dai, beta, StitchingProperties, Plane));
    py::class_<Stitching::VirtualCamera> stitchingVirtualCamera(stitchingNode, "VirtualCamera", DOC(dai, beta, StitchingProperties, VirtualCamera));

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
        .def_readwrite("point", &Stitching::Plane::point, DOC(dai, beta, StitchingProperties, Plane, point))
        .def_readwrite("normal", &Stitching::Plane::normal, DOC(dai, beta, StitchingProperties, Plane, normal))
        .def_readwrite("unit", &Stitching::Plane::unit, DOC(dai, beta, StitchingProperties, Plane, unit));

    stitchingVirtualCamera.def(py::init<>())
        .def_readwrite("pose", &Stitching::VirtualCamera::pose, DOC(dai, beta, StitchingProperties, VirtualCamera, pose))
        .def_readwrite("unit", &Stitching::VirtualCamera::unit, DOC(dai, beta, StitchingProperties, VirtualCamera, unit))
        .def_readwrite("intrinsics", &Stitching::VirtualCamera::intrinsics, DOC(dai, beta, StitchingProperties, VirtualCamera, intrinsics))
        .def_readwrite("width", &Stitching::VirtualCamera::width, DOC(dai, beta, StitchingProperties, VirtualCamera, width))
        .def_readwrite("height", &Stitching::VirtualCamera::height, DOC(dai, beta, StitchingProperties, VirtualCamera, height));

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

    stitchingNode.def_readonly("sync", &Stitching::sync, DOC(dai, beta, node, Stitching, sync))
        .def_property_readonly(
            "inputs", [](Stitching& node) { return &node.inputs; }, py::return_value_policy::reference_internal, DOC(dai, beta, node, Stitching, inputs))
        .def_readonly("out", &Stitching::out, DOC(dai, beta, node, Stitching, out))
        .def("build",
             static_cast<std::shared_ptr<Stitching> (Stitching::*)(const std::vector<Node::Output*>&)>(&Stitching::build),
             py::arg("sources"),
             DOC(dai, beta, node, Stitching, build))
        .def("build",
             static_cast<std::shared_ptr<Stitching> (Stitching::*)(size_t)>(&Stitching::build),
             py::arg("numInputs"),
             DOC(dai, beta, node, Stitching, build, 2))
        .def("getNumInputs", &Stitching::getNumInputs, DOC(dai, beta, node, Stitching, getNumInputs))
        .def("setSyncThreshold", &Stitching::setSyncThreshold, py::arg("syncThreshold"), DOC(dai, beta, node, Stitching, setSyncThreshold))
        .def("setRunOnHost", &Stitching::setRunOnHost, py::arg("runOnHost"), DOC(dai, beta, node, Stitching, setRunOnHost))
        .def("runOnHost", &Stitching::runOnHost, DOC(dai, beta, node, Stitching, runOnHost))
        .def("setMode", &Stitching::setMode, py::arg("mode"), DOC(dai, beta, node, Stitching, setMode))
        .def("getMode", &Stitching::getMode, DOC(dai, beta, node, Stitching, getMode))
        .def("setPlane",
             static_cast<void (Stitching::*)(const Stitching::Plane&)>(&Stitching::setPlane),
             py::arg("plane"),
             DOC(dai, beta, node, Stitching, setPlane))
        .def("setPlane",
             static_cast<void (Stitching::*)(const Point3f&, const Point3f&, LengthUnit)>(&Stitching::setPlane),
             py::arg("point"),
             py::arg("normal"),
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, beta, node, Stitching, setPlane, 2))
        .def("getPlane", &Stitching::getPlane, DOC(dai, beta, node, Stitching, getPlane))
        .def("setView", &Stitching::setView, py::arg("view"), DOC(dai, beta, node, Stitching, setView))
        .def("setViewAuto", &Stitching::setViewAuto, DOC(dai, beta, node, Stitching, setViewAuto))
        .def("getView", &Stitching::getView, DOC(dai, beta, node, Stitching, getView))
        .def("setMaxViewSize", &Stitching::setMaxViewSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, Stitching, setMaxViewSize))
        .def("setMaxRange", &Stitching::setMaxRange, py::arg("range"), py::arg("unit") = LengthUnit::CENTIMETER, DOC(dai, beta, node, Stitching, setMaxRange))
        .def("getMaxRange", &Stitching::getMaxRange, py::arg("unit") = LengthUnit::CENTIMETER, DOC(dai, beta, node, Stitching, getMaxRange))
        .def("setMinIncidenceAngle", &Stitching::setMinIncidenceAngle, py::arg("degrees"), DOC(dai, beta, node, Stitching, setMinIncidenceAngle))
        .def("getMinIncidenceAngle", &Stitching::getMinIncidenceAngle, DOC(dai, beta, node, Stitching, getMinIncidenceAngle))
        .def("setCameraModel", &Stitching::setCameraModel, py::arg("model"), DOC(dai, beta, node, Stitching, setCameraModel))
        .def("getCameraModel", &Stitching::getCameraModel, DOC(dai, beta, node, Stitching, getCameraModel))
        .def("setContinuous", &Stitching::setContinuous, py::arg("continuous"), DOC(dai, beta, node, Stitching, setContinuous))
        .def("getContinuous", &Stitching::getContinuous, DOC(dai, beta, node, Stitching, getContinuous))
        .def("setEstimationFrames", &Stitching::setEstimationFrames, py::arg("frames"), DOC(dai, beta, node, Stitching, setEstimationFrames))
        .def("getEstimationFrames", &Stitching::getEstimationFrames, DOC(dai, beta, node, Stitching, getEstimationFrames))
        .def("setMaxPanoramaSize", &Stitching::setMaxPanoramaSize, py::arg("width"), py::arg("height"), DOC(dai, beta, node, Stitching, setMaxPanoramaSize))
        .def("resetTransform", &Stitching::resetTransform, DOC(dai, beta, node, Stitching, resetTransform))
        .def("setPanoConfidenceThreshold",
             &Stitching::setPanoConfidenceThreshold,
             py::arg("threshold"),
             DOC(dai, beta, node, Stitching, setPanoConfidenceThreshold))
        .def("getPanoConfidenceThreshold", &Stitching::getPanoConfidenceThreshold, DOC(dai, beta, node, Stitching, getPanoConfidenceThreshold))
        .def("setSeamFinder", &Stitching::setSeamFinder, py::arg("finder"), DOC(dai, beta, node, Stitching, setSeamFinder))
        .def("getSeamFinder", &Stitching::getSeamFinder, DOC(dai, beta, node, Stitching, getSeamFinder));
    stitchingNode.attr("Properties") = stitchingProperties;
}
