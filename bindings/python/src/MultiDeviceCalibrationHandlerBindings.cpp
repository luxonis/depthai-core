#include "MultiDeviceCalibrationHandlerBindings.hpp"

#include <pybind11/stl.h>

#include "depthai/common/MultiDeviceCalibrationData.hpp"
#include "depthai/device/MultiDeviceCalibrationHandler.hpp"

void MultiDeviceCalibrationHandlerBindings::bind(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // Type definitions
    py::class_<RigEdge> rigEdge(m, "RigEdge", DOC(dai, RigEdge));
    py::class_<MultiDeviceCalibrationData> multiDeviceCalibrationData(m, "MultiDeviceCalibrationData", DOC(dai, MultiDeviceCalibrationData));
    py::class_<MultiDeviceCalibrationHandler> multiDeviceCalibrationHandler(m, "MultiDeviceCalibrationHandler", DOC(dai, MultiDeviceCalibrationHandler));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Bindings
    rigEdge.def(py::init<>())
        .def_readwrite("from_", &RigEdge::from, DOC(dai, RigEdge, from))
        .def_readwrite("to", &RigEdge::to, DOC(dai, RigEdge, to))
        .def_readwrite("transform", &RigEdge::transform, DOC(dai, RigEdge, transform))
        .def_readwrite("timestamp", &RigEdge::timestamp, DOC(dai, RigEdge, timestamp))
        .def_readwrite("source", &RigEdge::source, DOC(dai, RigEdge, source));

    multiDeviceCalibrationData.def(py::init<>())
        .def_readwrite("version", &MultiDeviceCalibrationData::version, DOC(dai, MultiDeviceCalibrationData, version))
        .def_readwrite("timestamp", &MultiDeviceCalibrationData::timestamp, DOC(dai, MultiDeviceCalibrationData, timestamp))
        .def_readwrite("edges", &MultiDeviceCalibrationData::edges, DOC(dai, MultiDeviceCalibrationData, edges))
        .def_readwrite("aliases", &MultiDeviceCalibrationData::aliases, DOC(dai, MultiDeviceCalibrationData, aliases));

    multiDeviceCalibrationHandler.def(py::init<>(), DOC(dai, MultiDeviceCalibrationHandler, MultiDeviceCalibrationHandler))
        .def(py::init<MultiDeviceCalibrationData, bool>(),
             py::arg("data"),
             py::arg("validate") = true,
             DOC(dai, MultiDeviceCalibrationHandler, MultiDeviceCalibrationHandler, 2))
        .def(py::init<std::filesystem::path>(), py::arg("jsonPath"), DOC(dai, MultiDeviceCalibrationHandler, MultiDeviceCalibrationHandler, 3))
        .def_static("fromJson",
                    &MultiDeviceCalibrationHandler::fromJson,
                    py::arg("json"),
                    py::arg("validate") = true,
                    DOC(dai, MultiDeviceCalibrationHandler, fromJson))
        .def("getData", &MultiDeviceCalibrationHandler::getData, DOC(dai, MultiDeviceCalibrationHandler, getData))
        .def("toJson", &MultiDeviceCalibrationHandler::toJson, DOC(dai, MultiDeviceCalibrationHandler, toJson))
        .def("toJsonFile", &MultiDeviceCalibrationHandler::toJsonFile, py::arg("jsonPath"), DOC(dai, MultiDeviceCalibrationHandler, toJsonFile))
        .def("getDeviceIds", &MultiDeviceCalibrationHandler::getDeviceIds, DOC(dai, MultiDeviceCalibrationHandler, getDeviceIds))
        .def("getFrames", &MultiDeviceCalibrationHandler::getFrames, DOC(dai, MultiDeviceCalibrationHandler, getFrames))
        .def("empty", &MultiDeviceCalibrationHandler::empty, DOC(dai, MultiDeviceCalibrationHandler, empty))
        .def(
            "canTransform", &MultiDeviceCalibrationHandler::canTransform, py::arg("from"), py::arg("to"), DOC(dai, MultiDeviceCalibrationHandler, canTransform))
        .def("getComponentRoot", &MultiDeviceCalibrationHandler::getComponentRoot, py::arg("frame"), DOC(dai, MultiDeviceCalibrationHandler, getComponentRoot))
        .def("getComponents", &MultiDeviceCalibrationHandler::getComponents, DOC(dai, MultiDeviceCalibrationHandler, getComponents))
        .def("getRevision", &MultiDeviceCalibrationHandler::getRevision, DOC(dai, MultiDeviceCalibrationHandler, getRevision))
        .def("getTransform",
             &MultiDeviceCalibrationHandler::getTransform,
             py::arg("from"),
             py::arg("to"),
             py::arg("unit") = LengthUnit::CENTIMETER,
             DOC(dai, MultiDeviceCalibrationHandler, getTransform))
        .def("setEdge", &MultiDeviceCalibrationHandler::setEdge, py::arg("edge"), DOC(dai, MultiDeviceCalibrationHandler, setEdge))
        .def("removeEdge", &MultiDeviceCalibrationHandler::removeEdge, py::arg("from"), py::arg("to"), DOC(dai, MultiDeviceCalibrationHandler, removeEdge))
        .def("resolveAliases",
             &MultiDeviceCalibrationHandler::resolveAliases,
             py::arg("aliasToDeviceId") = std::map<std::string, std::string>{},
             DOC(dai, MultiDeviceCalibrationHandler, resolveAliases));
}
