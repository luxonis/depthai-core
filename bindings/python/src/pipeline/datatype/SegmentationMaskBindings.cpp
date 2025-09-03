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
        .def(py::init<std::vector<std::uint8_t>, size_t, size_t>(),
             py::arg("mask"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, SegmentationMask, SegmentationMask, 3))
        .def("__repr__", &SegmentationMask::str)
        .def("getTimestamp", &dai::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &dai::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getHeight", &SegmentationMask::getHeight, DOC(dai, SegmentationMask, getHeight))
        .def("getWidth", &SegmentationMask::getWidth, DOC(dai, SegmentationMask, getWidth))
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def(py::init<const cv::Mat&>(), py::arg("mask"), DOC(dai, SegmentationMask, SegmentationMask, 4))
        .def("getMask", &SegmentationMask::getMask, DOC(dai, SegmentationMask, getMask))
        .def("setMask", &SegmentationMask::setMask, py::arg("mask"), DOC(dai, SegmentationMask, setMask))
        .def("getCvMask", &SegmentationMask::getCvMask, py::arg("allocator") = nullptr, DOC(dai, SegmentationMask, getCvMask))
        .def("getCvMaskByIndex",
             &SegmentationMask::getCvMaskByIndex,
             py::arg("index"),
             py::arg("allocator") = nullptr,
             DOC(dai, SegmentationMask, getCvMaskByIndex))
#endif
#ifdef DEPTHAI_XTENSOR_SUPPORT
        .def(py::init<xt::xtensor<std::uint8_t, 2, xt::layout_type::row_major>>(), py::arg("mask"), DOC(dai, SegmentationMask, SegmentationMask, 5))
        .def("getTensorMask", &SegmentationMask::getTensorMask, DOC(dai, SegmentationMask, getTensorMask))
        .def("getTensorMaskByIndex", &SegmentationMask::getTensorMaskByIndex, py::arg("index"), DOC(dai, SegmentationMask, getTensorMaskByIndex));
#endif
}
