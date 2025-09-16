#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "ndarray_converter.h"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/SegmentationMask.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/detail/common.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

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
        .def(py::init<size_t, size_t>(), py::arg("width"), py::arg("height"), DOC(dai, SegmentationMask, SegmentationMask, 2))
        .def(py::init([](std::vector<int> mask, size_t width, size_t height) { return std::make_shared<dai::SegmentationMask>(mask, width, height); }),
             py::arg("mask_vector"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, SegmentationMask, SegmentationMask, 3))
        .def("__repr__", &SegmentationMask::str)
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def(py::init<cv::Mat>(), py::arg("mask"), DOC(dai, SegmentationMask, SegmentationMask, 4))
        .def(
            "getMask", [](SegmentationMask& self) { return self.getMask(false); }, DOC(dai, SegmentationMask, getMask))
        .def("setMask", &SegmentationMask::setMask, DOC(dai, SegmentationMask, setMask))
        .def(
            "getCvMask", [](SegmentationMask& self) { return self.getCvMask(&g_numpyAllocator); }, DOC(dai, SegmentationMask, getCvMask))
        .def(
            "getCvMaskByIndex",
            [](SegmentationMask& self, uint8_t index) { return self.getCvMaskByIndex(index, &g_numpyAllocator); },
            py::arg("index"),
            DOC(dai, SegmentationMask, getCvMaskByIndex))
#endif
#ifdef DEPTHAI_XTENSOR_SUPPORT
        .def(
            "getTensorMask", [](SegmentationMask& self) -> py::object { return py::cast(self.getTensorMask()); }, DOC(dai, SegmentationMask, getTensorMask))
        .def("setTensorMask",
             static_cast<SegmentationMask& (SegmentationMask::*)(xt::xarray<uint8_t>&)>(&SegmentationMask::setTensorMask),
             py::arg("mask"),
             DOC(dai, SegmentationMask, setTensorMask))
        .def(
            "getTensorMaskByIndex",
            [](SegmentationMask& self, uint8_t index) -> py::object { return py::cast(self.getTensorMaskByIndex(index)); },
            py::arg("index"),
            DOC(dai, SegmentationMask, getTensorMaskByIndex))
#endif

        .def("getTimestamp", &dai::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &dai::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &dai::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getHeight", &SegmentationMask::getHeight, DOC(dai, SegmentationMask, getHeight))
        .def("getWidth", &SegmentationMask::getWidth, DOC(dai, SegmentationMask, getWidth));
}
