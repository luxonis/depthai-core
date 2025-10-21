#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "depthai/common/Keypoint.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/common/Point2f.hpp"
#include "depthai/common/Point3f.hpp"
#include "depthai/pipeline/datatype/Keypoints.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/detail/common.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

// #include "spdlog/spdlog.h"

void bind_keypoints(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<Keypoints, Py<Keypoints>, Buffer, std::shared_ptr<Keypoints>> keypoints(m, "Keypoints", DOC(dai, Keypoints));

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
    // Message

    py::class_<dai::Keypoints, dai::Buffer, std::shared_ptr<dai::Keypoints>>(m, "Keypoints")
        .def(py::init<>(), DOC(dai, Keypoints, Keypoints))
        .def("__repr__", &Keypoints::str)
        .def("setKeypoints", py::overload_cast<const std::vector<Keypoint>&>(&Keypoints::setKeypoints), py::arg("keypoints"))
        .def("setKeypoints", py::overload_cast<const std::vector<Point3f>&>(&Keypoints::setKeypoints), py::arg("keypoints"))
        .def("setKeypoints", py::overload_cast<const std::vector<Point2f>&>(&Keypoints::setKeypoints), py::arg("keypoints"))
        .def("setKeypoints",
             py::overload_cast<const std::vector<Keypoint>&, const std::vector<std::array<uint32_t, 2>>&>(&Keypoints::setKeypoints),
             py::arg("keypoints"),
             py::arg("edges"))
        .def("setEdges", &dai::Keypoints::setEdges, py::arg("edges"))
        .def("getKeypoints", &Keypoints::getKeypoints, DOC(dai, Keypoints, getKeypoints))
        .def(
            "getEdges", [](Keypoints& msg) { return msg.getEdges(); }, DOC(dai, Keypoints, getEdges))
        .def("getCoordinates3f", &Keypoints::getCoordinates3f, DOC(dai, Keypoints, getCoordinates3f))
        .def("getCoordinates2f", &Keypoints::getCoordinates2f, DOC(dai, Keypoints, getCoordinates2f))
        .def("getTimestamp", &Keypoints::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &Keypoints::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &Keypoints::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getTransformation", [](Keypoints& msg) { return msg.transformation; })
        .def(
            "setTransformation",
            [](Keypoints& msg, const std::optional<ImgTransformation>& transformation) { msg.transformation = transformation; },
            py::arg("transformation"));
}
