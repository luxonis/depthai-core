#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "ndarray_converter.h"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/pytypes.h>

// #include "spdlog/spdlog.h"

void bind_imgdetections(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawImgDetections, RawBuffer, std::shared_ptr<RawImgDetections>> rawImgDetections(m, "RawImgDetections", DOC(dai, RawImgDetections));
    py::class_<ImgDetections, Py<ImgDetections>, Buffer, std::shared_ptr<ImgDetections>> imgDetections(m, "ImgDetections", DOC(dai, ImgDetections));
    py::class_<ImgDetection> imgDetection(m, "ImgDetection", DOC(dai, ImgDetection));

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

    // Metadata / raw
    imgDetection.def(py::init<>())
        .def(py::init<const RotatedRect&, float, std::uint32_t>(), py::arg("boundingBox"), py::arg("confidence"), py::arg("label"))
        .def(py::init<const RotatedRect&, std::string, float, std::uint32_t>(),
             py::arg("boundingBox"),
             py::arg("labelName"),
             py::arg("confidence"),
             py::arg("label"))
        .def(py::init<const RotatedRect&, const KeypointsList&, float, std::uint32_t>(),
             py::arg("boundingBox"),
             py::arg("keypoints"),
             py::arg("confidence"),
             py::arg("label"))
        .def(py::init<const RotatedRect&, const KeypointsList&, std::string, float, std::uint32_t>(),
             py::arg("boundingBox"),
             py::arg("keypoints"),
             py::arg("labelName"),
             py::arg("confidence"),
             py::arg("label"))
        .def("__repr__", &ImgDetection::str)
        .def_readwrite("label", &ImgDetection::label)
        .def_readwrite("labelName", &ImgDetection::labelName)
        .def_readwrite("confidence", &ImgDetection::confidence)
        .def_readwrite("xmin", &ImgDetection::xmin)
        .def_readwrite("ymin", &ImgDetection::ymin)
        .def_readwrite("xmax", &ImgDetection::xmax)
        .def_readwrite("ymax", &ImgDetection::ymax)
        .def("setBoundingBox", &ImgDetection::setBoundingBox, py::arg("boundingBox"))
        .def("getBoundingBox", &ImgDetection::getBoundingBox)
        .def("setOuterBoundingBox", &ImgDetection::setOuterBoundingBox, py::arg("xmin"), py::arg("ymin"), py::arg("xmax"), py::arg("ymax"))
        .def("setKeypoints", py::overload_cast<const KeypointsList>(&ImgDetection::setKeypoints), py::arg("keypoints"))
        .def("setKeypoints", py::overload_cast<const std::vector<Keypoint>>(&ImgDetection::setKeypoints), py::arg("keypoints"))
        .def("setKeypoints",
             py::overload_cast<const std::vector<Keypoint>, const std::vector<Edge>>(&ImgDetection::setKeypoints),
             py::arg("keypoints"),
             py::arg("edges"))
        .def("setKeypoints", py::overload_cast<const std::vector<Point3f>>(&ImgDetection::setKeypoints), py::arg("keypoints"))
        .def("setKeypoints", py::overload_cast<const std::vector<Point2f>>(&ImgDetection::setKeypoints), py::arg("keypoints"))
        .def("getKeypoints", &ImgDetection::getKeypoints, DOC(dai, ImgDetection, getKeypoints))
        .def("setEdges", &ImgDetection::setEdges, py::arg("edges"))
        .def("getEdges", &ImgDetection::getEdges, DOC(dai, ImgDetection, getEdges))
        .def("centerX", &dai::ImgDetection::getCenterX)
        .def("centerY", &dai::ImgDetection::getCenterY)
        .def("width", &dai::ImgDetection::getWidth)
        .def("height", &dai::ImgDetection::getHeight)
        .def("angle", &dai::ImgDetection::getAngle);

    // rawImgDetections
    //     .def(py::init<>())
    //     .def_readwrite("detections", &RawImgDetections::detections)
    //     .def_property("ts",
    //         [](const RawImgDetections& o){
    //             double ts = o.ts.sec + o.ts.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawImgDetections& o, double ts){
    //             o.ts.sec = ts;
    //             o.ts.nsec = (ts - o.ts.sec) * 1000000000.0;
    //         }
    //     )
    //     .def_property("tsDevice",
    //         [](const RawImgDetections& o){
    //             double ts = o.tsDevice.sec + o.tsDevice.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawImgDetections& o, double ts){
    //             o.tsDevice.sec = ts;
    //             o.tsDevice.nsec = (ts - o.tsDevice.sec) * 1000000000.0;
    //         }
    //     )
    //     .def_readwrite("sequenceNum", &RawImgDetections::sequenceNum)
    //     ;

    // Message
    imgDetections.def(py::init<>(), DOC(dai, ImgDetections, ImgDetections))
        .def("__repr__", &ImgDetections::str)
        .def_property(
            "detections",
            [](ImgDetections& det) { return &det.detections; },
            [](ImgDetections& det, std::vector<ImgDetection> val) { det.detections = val; },
            DOC(dai, ImgDetections, detections))
        .def("getTimestamp", &ImgDetections::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &ImgDetections::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &ImgDetections::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getTransformation", [](ImgDetections& msg) { return msg.transformation; })
        .def("setTransformation", [](ImgDetections& msg, const std::optional<ImgTransformation>& transformation) { msg.transformation = transformation; })
        .def("getSegmentationMaskWidth", &ImgDetections::getSegmentationMaskWidth, DOC(dai, ImgDetections, getSegmentationMaskWidth))
        .def("getSegmentationMaskHeight", &ImgDetections::getSegmentationMaskHeight, DOC(dai, ImgDetections, getSegmentationMaskHeight))
        .def("setMask", &ImgDetections::setMask, py::arg("mask"), py::arg("width"), py::arg("height"), DOC(dai, ImgDetections, setMask))
        .def("getMaskData", &ImgDetections::getMaskData, DOC(dai, ImgDetections, getMaskData))
        .def("getSegmentationMaskAsImgFrame", &ImgDetections::getSegmentationMaskAsImgFrame, DOC(dai, ImgDetections, getSegmentationMaskAsImgFrame))
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def(
            "getSegmentationMask", [](ImgDetections& self) { return self.getSegmentationMask(false); }, DOC(dai, ImgDetections, getSegmentationMask))
        .def("setSegmentationMask", &ImgDetections::setSegmentationMask, DOC(dai, ImgDetections, setSegmentationMask))
        .def(
            "getCvSegmentationMask",
            [](ImgDetections& self) { return self.getCvSegmentationMask(&g_numpyAllocator); },
            DOC(dai, ImgDetections, getCvSegmentationMask))
        .def(
            "getCvSegmentationMaskByIndex",
            [](ImgDetections& self, uint8_t index) { return self.getCvSegmentationMaskByIndex(index, &g_numpyAllocator); },
            py::arg("index"),
            DOC(dai, ImgDetections, getCvSegmentationMaskByIndex))
        .def(
            "getCvSegmentationMaskByClass",
            [](ImgDetections& self, uint8_t class_index) { return self.getCvSegmentationMaskByClass(class_index, &g_numpyAllocator); },
            py::arg("semantic_class"),
            DOC(dai, ImgDetections, getCvSegmentationMaskByClass));
#endif
}
