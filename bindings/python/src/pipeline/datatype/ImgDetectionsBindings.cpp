#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgDetections.hpp"
#include "depthai/pipeline/datatype/ImgDetectionsT.hpp"
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
        .def("getKeypoints2f", &ImgDetection::getKeypoints2f, DOC(dai, ImgDetection, getKeypoints2f))
        .def("getKeypoints3f", &ImgDetection::getKeypoints3f, DOC(dai, ImgDetection, getKeypoints3f))
        .def("setEdges", &ImgDetection::setEdges, py::arg("edges"))
        .def("getEdges", &ImgDetection::getEdges, DOC(dai, ImgDetection, getEdges))
        .def("getCenterX", &dai::ImgDetection::getCenterX)
        .def("getCenterY", &dai::ImgDetection::getCenterY)
        .def("getWidth", &dai::ImgDetection::getWidth)
        .def("getHeight", &dai::ImgDetection::getHeight)
        .def("getAngle", &dai::ImgDetection::getAngle);

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
    imgDetections.def(py::init<>(), DOC(dai, ImgDetectionsT, ImgDetectionsT))
        .def("__repr__", &ImgDetections::str)
        .def_property(
            "detections",
            [](ImgDetections& det) { return &det.detections; },
            [](ImgDetections& det, std::vector<ImgDetection> val) { det.detections = val; },
            DOC(dai, ImgDetectionsT, detections),
            py::return_value_policy::reference_internal)
        .def_property(
            "segmentationMaskWidth",
            [](ImgDetections& det) { return &det.segmentationMaskWidth; },
            [](ImgDetections& det, size_t val) { det.segmentationMaskWidth = val; },
            DOC(dai, ImgDetectionsT, segmentationMaskWidth),
            py::return_value_policy::reference_internal)
        .def_property(
            "segmentationMaskHeight",
            [](ImgDetections& det) { return &det.segmentationMaskHeight; },
            [](ImgDetections& det, size_t val) { det.segmentationMaskHeight = val; },
            DOC(dai, ImgDetectionsT, segmentationMaskHeight),
            py::return_value_policy::reference_internal)
        .def("getTimestamp", &dai::ImgDetectionsT<dai::ImgDetection>::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &dai::ImgDetectionsT<dai::ImgDetection>::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &dai::ImgDetectionsT<dai::ImgDetection>::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getTransformation", [](ImgDetections& msg) { return msg.transformation; })
        .def("setTransformation", [](ImgDetections& msg, const std::optional<ImgTransformation>& transformation) { msg.transformation = transformation; })
        .def("getSegmentationMaskWidth", &ImgDetections::getSegmentationMaskWidth, DOC(dai, ImgDetectionsT, getSegmentationMaskWidth))
        .def("getSegmentationMaskHeight", &ImgDetections::getSegmentationMaskHeight, DOC(dai, ImgDetectionsT, getSegmentationMaskHeight))
        .def("setSegmentationMask",
             py::overload_cast<dai::ImgFrame&>(&ImgDetections::setSegmentationMask),
             py::arg("frame"),
             DOC(dai, ImgDetectionsT, setSegmentationMask),
             py::return_value_policy::reference_internal)
        .def("getMaskData", &ImgDetections::getMaskData, DOC(dai, ImgDetectionsT, getMaskData))
        .def("getSegmentationMask", &ImgDetections::getSegmentationMask, DOC(dai, ImgDetectionsT, getSegmentationMask))
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def("setCvSegmentationMask", &ImgDetections::setCvSegmentationMask, py::arg("mask"), DOC(dai, ImgDetectionsT, setCvSegmentationMask))
        .def(
            "getCvSegmentationMask",
            [](ImgDetections& self) { return self.getCvSegmentationMask(&g_numpyAllocator); },
            DOC(dai, ImgDetectionsT, getCvSegmentationMask))
        .def(
            "getCvSegmentationMaskByIndex",
            [](ImgDetections& self, uint8_t index) { return self.getCvSegmentationMaskByIndex(index, &g_numpyAllocator); },
            py::arg("index"),
            DOC(dai, ImgDetectionsT, getCvSegmentationMaskByIndex))
        .def(
            "getCvSegmentationMaskByClass",
            [](ImgDetections& self, uint8_t semanticClass) { return self.getCvSegmentationMaskByClass(semanticClass, &g_numpyAllocator); },
            py::arg("semantic_class"),
            DOC(dai, ImgDetectionsT, getCvSegmentationMaskByClass));
#endif
}
