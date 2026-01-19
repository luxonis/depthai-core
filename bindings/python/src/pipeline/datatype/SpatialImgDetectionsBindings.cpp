#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/SpatialImgDetections.hpp"
#include "ndarray_converter.h"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_spatialimgdetections(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawSpatialImgDetections, RawBuffer, std::shared_ptr<RawSpatialImgDetections>> rawSpatialImgDetections(m, "RawSpatialImgDetections", DOC(dai,
    // RawSpatialImgDetections));
    py::class_<SpatialImgDetection> spatialImgDetection(m, "SpatialImgDetection", DOC(dai, SpatialImgDetection));
    py::class_<SpatialImgDetections, Py<SpatialImgDetections>, Buffer, std::shared_ptr<SpatialImgDetections>> spatialImgDetections(
        m, "SpatialImgDetections", DOC(dai, SpatialImgDetections));

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
    spatialImgDetection.def(py::init<>())
        .def(py::init<const RotatedRect&, Point3f, float, std::uint32_t>(),
             py::arg("boundingBox"),
             py::arg("spatialCoordinates"),
             py::arg("confidence"),
             py::arg("label"),
             DOC(dai, SpatialImgDetection, SpatialImgDetection))
        .def(py::init<const RotatedRect&, Point3f, std::string, float, std::uint32_t>(),
             py::arg("boundingBox"),
             py::arg("spatialCoordinates"),
             py::arg("labelName"),
             py::arg("confidence"),
             py::arg("label"),
             DOC(dai, SpatialImgDetection, SpatialImgDetection))
        .def(py::init<const RotatedRect&, Point3f, const SpatialKeypointsList&, float, std::uint32_t>(),
             py::arg("boundingBox"),
             py::arg("spatialCoordinates"),
             py::arg("keypoints"),
             py::arg("confidence"),
             py::arg("label"),
             DOC(dai, SpatialImgDetection, SpatialImgDetection))
        .def(py::init<const RotatedRect&, Point3f, const SpatialKeypointsList&, std::string, float, std::uint32_t>(),
             py::arg("boundingBox"),
             py::arg("spatialCoordinates"),
             py::arg("keypoints"),
             py::arg("labelName"),
             py::arg("confidence"),
             py::arg("label"),
             DOC(dai, SpatialImgDetection, SpatialImgDetection))
        .def("__repr__", &SpatialImgDetection::str)
        .def_readwrite("label", &SpatialImgDetection::label)
        .def_readwrite("labelName", &SpatialImgDetection::labelName)
        .def_readwrite("confidence", &SpatialImgDetection::confidence)
        .def_readwrite("xmin", &SpatialImgDetection::xmin, "Deprecation warning: use boundingBox instead")
        .def_readwrite("ymin", &SpatialImgDetection::ymin, "Deprecation warning: use boundingBox instead")
        .def_readwrite("xmax", &SpatialImgDetection::xmax, "Deprecation warning: use boundingBox instead")
        .def_readwrite("ymax", &SpatialImgDetection::ymax, "Deprecation warning: use boundingBox instead")
        .def_readwrite("spatialCoordinates", &SpatialImgDetection::spatialCoordinates)
        .def_readwrite("boundingBoxMapping", &SpatialImgDetection::boundingBoxMapping)
        .def_readwrite("boundingBox", &SpatialImgDetection::boundingBox)
        .def("setBoundingBox", &SpatialImgDetection::setBoundingBox, py::arg("boundingBox"), DOC(dai, SpatialImgDetection, setBoundingBox))
        .def("getBoundingBox", &SpatialImgDetection::getBoundingBox, DOC(dai, SpatialImgDetection, getBoundingBox))
        .def("setOuterBoundingBox",
             &SpatialImgDetection::setOuterBoundingBox,
             py::arg("xmin"),
             py::arg("ymin"),
             py::arg("xmax"),
             py::arg("ymax"),
             DOC(dai, SpatialImgDetection, setOuterBoundingBox))
        .def("setKeypoints",
             py::overload_cast<const SpatialKeypointsList>(&SpatialImgDetection::setKeypoints),
             py::arg("keypoints"),
             DOC(dai, SpatialImgDetection, setKeypoints))
        .def("setKeypoints",
             py::overload_cast<const std::vector<SpatialKeypoint>>(&SpatialImgDetection::setKeypoints),
             py::arg("keypoints"),
             DOC(dai, SpatialImgDetection, setKeypoints))
        .def("setKeypoints",
             py::overload_cast<const std::vector<SpatialKeypoint>, const std::vector<Edge>>(&SpatialImgDetection::setKeypoints),
             py::arg("keypoints"),
             py::arg("edges"),
             DOC(dai, SpatialImgDetection, setKeypoints))
        .def("setKeypoints",
             py::overload_cast<const std::vector<Point3f>>(&SpatialImgDetection::setKeypoints),
             py::arg("keypoints"),
             DOC(dai, SpatialImgDetection, setKeypoints))
        .def("setSpatialCoordinate",
             &SpatialImgDetection::setSpatialCoordinate,
             py::arg("spatialCoordinates"),
             DOC(dai, SpatialImgDetection, setSpatialCoordinate))
        .def("setEdges", &SpatialImgDetection::setEdges, py::arg("edges"), DOC(dai, SpatialImgDetection, setEdges))
        .def("getImgDetection", &SpatialImgDetection::getImgDetection, DOC(dai, SpatialImgDetection, getImgDetection))
        .def("getKeypoints", &SpatialImgDetection::getKeypoints, DOC(dai, SpatialImgDetection, getKeypoints))
        .def("getKeypointSpatialCoordinates", &SpatialImgDetection::getKeypointSpatialCoordinates, DOC(dai, SpatialImgDetection, getKeypointSpatialCoordinates))
        .def("getEdges", &SpatialImgDetection::getEdges, DOC(dai, SpatialImgDetection, getEdges))
        .def("getCenterX", &SpatialImgDetection::getCenterX, DOC(dai, SpatialImgDetection, getCenterX))
        .def("getCenterY", &SpatialImgDetection::getCenterY, DOC(dai, SpatialImgDetection, getCenterY))
        .def("getWidth", &SpatialImgDetection::getWidth, DOC(dai, SpatialImgDetection, getWidth))
        .def("getHeight", &SpatialImgDetection::getHeight, DOC(dai, SpatialImgDetection, getHeight))
        .def("getAngle", &SpatialImgDetection::getAngle, DOC(dai, SpatialImgDetection, getAngle));

    // Message
    spatialImgDetections.def(py::init<>(), DOC(dai, ImgDetectionsT, ImgDetectionsT))
        .def("__repr__", &SpatialImgDetections::str)
        .def_property(
            "detections",
            [](SpatialImgDetections& det) { return &det.detections; },
            [](SpatialImgDetections& det, std::vector<SpatialImgDetection> val) { det.detections = val; },
            DOC(dai, ImgDetectionsT, detections),
            py::return_value_policy::reference_internal)
        .def("getTimestamp", &SpatialImgDetections::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &SpatialImgDetections::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &SpatialImgDetections::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getTransformation", [](SpatialImgDetections& msg) { return msg.transformation; })
        .def("setTransformation",
             [](SpatialImgDetections& msg, const std::optional<ImgTransformation>& transformation) { msg.transformation = transformation; })
        .def("getSegmentationMaskWidth", &SpatialImgDetections::getSegmentationMaskWidth, DOC(dai, ImgDetectionsT, getSegmentationMaskWidth))
        .def("getSegmentationMaskHeight", &SpatialImgDetections::getSegmentationMaskHeight, DOC(dai, ImgDetectionsT, getSegmentationMaskHeight))
        .def("setSegmentationMask",
             py::overload_cast<dai::ImgFrame&>(&SpatialImgDetections::setSegmentationMask),
             py::arg("frame"),
             DOC(dai, ImgDetectionsT, setSegmentationMask),
             py::return_value_policy::reference_internal)
        .def("getMaskData", &SpatialImgDetections::getMaskData, DOC(dai, ImgDetectionsT, getMaskData))
        .def("getSegmentationMask", &SpatialImgDetections::getSegmentationMask, DOC(dai, ImgDetectionsT, getSegmentationMask))
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        .def("setCvSegmentationMask", &SpatialImgDetections::setCvSegmentationMask, py::arg("mask"), DOC(dai, ImgDetectionsT, setCvSegmentationMask))
        .def(
            "getCvSegmentationMask",
            [](SpatialImgDetections& self) { return self.getCvSegmentationMask(&g_numpyAllocator); },
            DOC(dai, ImgDetectionsT, getCvSegmentationMask))
        .def(
            "getCvSegmentationMaskByIndex",
            [](SpatialImgDetections& self, uint8_t index) { return self.getCvSegmentationMaskByIndex(index, &g_numpyAllocator); },
            py::arg("index"),
            DOC(dai, ImgDetectionsT, getCvSegmentationMaskByIndex))
        .def(
            "getCvSegmentationMaskByClass",
            [](SpatialImgDetections& self, uint8_t semanticClass) { return self.getCvSegmentationMaskByClass(semanticClass, &g_numpyAllocator); },
            py::arg("semantic_class"),
            DOC(dai, ImgDetectionsT, getCvSegmentationMaskByClass))
#endif
        // .def("setTimestamp", &SpatialImgDetections::setTimestamp, DOC(dai, SpatialImgDetections, setTimestamp))
        // .def("setTimestampDevice", &SpatialImgDetections::setTimestampDevice, DOC(dai, SpatialImgDetections, setTimestampDevice))
        // .def("setSequenceNum", &SpatialImgDetections::setSequenceNum, DOC(dai, SpatialImgDetections, setSequenceNum))
        ;
}
