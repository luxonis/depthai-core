#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/SegmentationMask.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/detail/common.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_segmentationmask(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    // Message

    py::class_<dai::SegmentationMask, dai::Buffer, std::shared_ptr<dai::SegmentationMask>>(m, "SegmentationMask")
        .def(py::init<>(), DOC(dai, SegmentationMask, SegmentationMask))
        .def("__repr__", &SegmentationMask::str)
        .def_readwrite("width", &SegmentationMask::width)
        .def_readwrite("height", &SegmentationMask::height)
        .def("getTimestamp", &dai::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &dai::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getHeight", &SegmentationMask::getHeight, DOC(dai, SegmentationMask, getHeight))
#ifdef DETPHAI_HAVE_OPENCV_SUPPORT
        .def(
            "getMask", [](SegmentationMask& self) { return self.getMask(); }, DOC(dai, SegmentationMask, getMask))
        .def("setMask", &SegmentationMask::setMask, py::arg("mask"), DOC(dai, SegmentationMask, setMask))
        .def(
            "getCvMask", [](SegmentationMask& self) { return self.getCvMask(&g_numpyAllocator); }, DOC(dai, SegmentationMask, getCvMask))
        .def(
            "getCvMaskByIndex",
            [](SegmentationMask& self, uint8_t index) { return self.getCvMaskByIndex(index, &g_numpyAllocator); },
            py::arg("index"),
            DOC(dai, SegmentationMask, getCvMaskByIndex))
#endif
#ifdef DEPTHAI_XTENSOR_SUPPORT
        .def("getTensorMask", &SegmentationMask::getTensorMask, DOC(dai, SegmentationMask, getTensorMask))
        .def("getTensorMaskByIndex", &SegmentationMask::getTensorMaskByIndex, py::arg("index"), DOC(dai, SegmentationMask, getTensorMaskByIndex));
#endif
}
