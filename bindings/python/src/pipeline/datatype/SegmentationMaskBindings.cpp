#include <cstring>
#include <memory>

#include "DatatypeBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/SegmentationMask.hpp"
#include "ndarray_converter.h"
// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>

namespace {
py::array_t<std::uint8_t> toNumpyMask(const dai::SegmentationMask& mask, const std::vector<std::uint8_t>& data) {
    if(data.empty()) {
        return py::array_t<std::uint8_t>();
    }

    const auto width = static_cast<py::ssize_t>(mask.getWidth());
    const auto height = static_cast<py::ssize_t>(mask.getHeight());
    const auto count = static_cast<py::ssize_t>(data.size());

    if(width > 0 && height > 0 && width * height == count) {
        py::array_t<std::uint8_t> arr({height, width});
        std::memcpy(arr.mutable_data(), data.data(), data.size());
        return arr;
    }

    py::array_t<std::uint8_t> arr(count);
    std::memcpy(arr.mutable_data(), data.data(), data.size());
    return arr;
}
}  // namespace

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
        .def(
            "setMask",
            [](SegmentationMask& self, const std::vector<int>& mask, size_t width, size_t height) {
                std::vector<std::uint8_t> converted;
                converted.reserve(mask.size());
                for(size_t i = 0; i < mask.size(); ++i) {
                    const int value = mask[i];
                    if(value < 0 || value > 255) {
                        throw py::value_error("SegmentationMask.setMask: mask values must be in [0, 255]");
                    }
                    converted.push_back(static_cast<std::uint8_t>(value));
                }
                self.setMask(converted, width, height);
            },
            py::arg("mask"),
            py::arg("width"),
            py::arg("height"),
            DOC(dai, SegmentationMask, setMask))
        .def("setMask",
             static_cast<void (SegmentationMask::*)(dai::ImgFrame&)>(&SegmentationMask::setMask),
             py::arg("frame"),
             DOC(dai, SegmentationMask, setMask, 2))
        .def(
            "getMaskData", [](SegmentationMask& self) { return toNumpyMask(self, self.getMaskData()); }, DOC(dai, SegmentationMask, getMaskData))
        .def("getFrame", &SegmentationMask::getFrame, DOC(dai, SegmentationMask, getFrame))
        .def("setLabels", &SegmentationMask::setLabels, py::arg("labels"), DOC(dai, SegmentationMask, setLabels))
        .def("getLabels", &SegmentationMask::getLabels, DOC(dai, SegmentationMask, getLabels))
        .def("getArea", &SegmentationMask::getArea, py::arg("index"), DOC(dai, SegmentationMask, getArea))
        .def("getCentroid", &SegmentationMask::getCentroid, py::arg("index"), DOC(dai, SegmentationMask, getCentroid))
        .def("getUniqueIndices",
             [](SegmentationMask& self) {
                 const auto v = self.getUniqueIndices();
                 py::list out;
                 for(auto x : v) out.append(py::int_(x));
                 return out;
             })
        .def(
            "getMaskByIndex",
            [](SegmentationMask& self, uint8_t index) { return toNumpyMask(self, self.getMaskByIndex(index)); },
            py::arg("index"),
            DOC(dai, SegmentationMask, getMaskByIndex))
        .def(
            "getMaskByLabel",
            [](SegmentationMask& self, const std::string& label) { return toNumpyMask(self, self.getMaskByLabel(label)); },
            py::arg("label"),
            DOC(dai, SegmentationMask, getMaskByLabel))
        .def("hasValidMask", &SegmentationMask::hasValidMask, DOC(dai, SegmentationMask, hasValidMask))
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def("setCvMask", &SegmentationMask::setCvMask, py::arg("mask"), DOC(dai, SegmentationMask, setCvMask))
        .def(
            "getCvMask", [](SegmentationMask& msg) { return msg.getCvMask(&g_numpyAllocator); }, DOC(dai, SegmentationMask, getCvMask))
        .def(
            "getCvMaskByIndex",
            [](SegmentationMask& msg, uint8_t index) { return msg.getCvMaskByIndex(index, &g_numpyAllocator); },
            py::arg("index"),
            DOC(dai, SegmentationMask, getCvMaskByIndex))
        .def("getContour", &SegmentationMask::getContour, py::arg("index"), DOC(dai, SegmentationMask, getContour))
        .def("getBoundingBoxes",
             &SegmentationMask::getBoundingBoxes,
             py::arg("index"),
             py::arg("calculateRotation") = false,
             DOC(dai, SegmentationMask, getBoundingBoxes))
#endif
        .def("getTimestamp", &SegmentationMask::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &SegmentationMask::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &SegmentationMask::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def(
            "getTransformation", [](SegmentationMask& msg) { return msg.transformation; }, DOC(dai, ImgFrame, getTransformation))
        .def("setTimestamp", &SegmentationMask::setTimestamp, py::arg("timestamp"), DOC(dai, Buffer, setTimestamp))
        .def("setTimestampDevice", &SegmentationMask::setTimestampDevice, py::arg("timestampDevice"), DOC(dai, Buffer, setTimestampDevice))
        .def("setSequenceNum", &SegmentationMask::setSequenceNum, py::arg("sequenceNum"), DOC(dai, Buffer, setSequenceNum))
        .def(
            "setTransformation",
            [](SegmentationMask& msg, const ImgTransformation& transformation) { msg.transformation = transformation; },
            py::arg("transformation"),
            DOC(dai, ImgFrame, setTransformation));
}
