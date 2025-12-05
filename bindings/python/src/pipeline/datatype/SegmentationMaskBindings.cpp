#include <memory>

#include "DatatypeBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "ndarray_converter.h"
// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/pytypes.h>

void bind_segmentationmask(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<SegmentationMask, Py<SegmentationMask>, Buffer, std::shared_ptr<SegmentationMask>> segmentationMask(
        m, "SegmentationMask", DOC(dai, SegmentationMask));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    // Actual bindings
    /////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    segmentationMask.def(py::init<>())
        .def(py::init<const std::vector<std::uint8_t>&, size_t, size_t>(),
             py::arg("mask"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, SegmentationMask, SegmentationMask))
        .def("__repr__", &SegmentationMask::str)
        .def("getWidth", &SegmentationMask::getWidth, DOC(dai, SegmentationMask, getWidth))
        .def("getHeight", &SegmentationMask::getHeight, DOC(dai, SegmentationMask, getHeight))
        .def("setMask",
             (void (SegmentationMask::*)(const std::vector<std::uint8_t>&, size_t, size_t))&SegmentationMask::setMask,
             py::arg("mask"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, SegmentationMask, setMask))
        .def("setMask", (void (SegmentationMask::*)(dai::ImgFrame&))&SegmentationMask::setMask, py::arg("frame"), DOC(dai, SegmentationMask, setMask, 2))
        .def("getMaskData", &SegmentationMask::getMaskData, DOC(dai, SegmentationMask, getMaskData))
        .def("getMask", &SegmentationMask::getMask, DOC(dai, SegmentationMask, getMask))
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def("setCvMask", &SegmentationMask::setCvMask, py::arg("mask"), DOC(dai, SegmentationMask, setCvMask))
        .def(
            "getCvMask", [](SegmentationMask& msg) { return msg.getCvMask(&g_numpyAllocator); }, DOC(dai, SegmentationMask, getCvMask))
        .def(
            "getCvMaskByIndex",
            [](SegmentationMask& msg, uint8_t index) { return msg.getCvMaskByIndex(index, &g_numpyAllocator); },
            py::arg("index"),
            DOC(dai, SegmentationMask, getCvMaskByIndex))
#endif
        .def("getTimestamp", &SegmentationMask::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &SegmentationMask::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &SegmentationMask::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getTransformation", [](SegmentationMask& msg) { return msg.transformation; })
        .def("setTimestamp", &SegmentationMask::setTimestamp, py::arg("ts"), DOC(dai, Buffer, setTimestamp))
        .def("setTimestampDevice", &SegmentationMask::setTimestampDevice, py::arg("ts"), DOC(dai, Buffer, setTimestampDevice))
        .def("setSequenceNum", &SegmentationMask::setSequenceNum, py::arg("seqNum"), DOC(dai, Buffer, setSequenceNum))
        .def("setTransformation", [](SegmentationMask& msg, const ImgTransformation transformation) { msg.transformation = transformation; });
}
