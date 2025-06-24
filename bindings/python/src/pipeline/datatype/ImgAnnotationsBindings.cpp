
#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ImgAnnotations.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>
#include <pybind11/stl_bind.h>

// #include "spdlog/spdlog.h"

PYBIND11_MAKE_OPAQUE(std::vector<dai::Color>);
PYBIND11_MAKE_OPAQUE(std::vector<dai::CircleAnnotation>);
PYBIND11_MAKE_OPAQUE(std::vector<dai::PointsAnnotation>);
PYBIND11_MAKE_OPAQUE(std::vector<dai::TextAnnotation>);
PYBIND11_MAKE_OPAQUE(std::vector<dai::ImgAnnotation>);

void bind_imageannotations(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ImgAnnotations, Py<ImgAnnotations>, Buffer, std::shared_ptr<ImgAnnotations>> imageAnnotations(m, "ImgAnnotations", DOC(dai, ImgAnnotations));
    py::class_<CircleAnnotation> circleAnnotation(m, "CircleAnnotation", DOC(dai, CircleAnnotation));
    py::enum_<PointsAnnotationType> pointsAnnotationType(m, "PointsAnnotationType", DOC(dai, PointsAnnotationType));
    py::class_<PointsAnnotation> pointsAnnotation(m, "PointsAnnotation", DOC(dai, PointsAnnotation));
    py::class_<TextAnnotation> textAnnotation(m, "TextAnnotation", DOC(dai, TextAnnotation));
    py::class_<ImgAnnotation> imageAnnotation(m, "ImgAnnotation", DOC(dai, ImgAnnotation));

    py::bind_vector<std::vector<dai::Color>>(m, "VectorColor");
    py::implicitly_convertible<py::list, std::vector<dai::Color>>();
    py::bind_vector<std::vector<dai::Point2f>>(m, "VectorPoint2f");
    py::implicitly_convertible<py::list, std::vector<dai::Point2f>>();
    py::bind_vector<std::vector<dai::CircleAnnotation>>(m, "VectorCircleAnnotation");
    py::implicitly_convertible<py::list, std::vector<dai::CircleAnnotation>>();
    py::bind_vector<std::vector<dai::PointsAnnotation>>(m, "VectorPointsAnnotation");
    py::implicitly_convertible<py::list, std::vector<dai::PointsAnnotation>>();
    py::bind_vector<std::vector<dai::TextAnnotation>>(m, "VectorTextAnnotation");
    py::implicitly_convertible<py::list, std::vector<dai::TextAnnotation>>();
    py::bind_vector<std::vector<dai::ImgAnnotation>>(m, "VectorImgAnnotation");
    py::implicitly_convertible<py::list, std::vector<dai::ImgAnnotation>>();

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
    circleAnnotation.def(py::init<>())
        .def_readwrite("position", &CircleAnnotation::position)
        .def_readwrite("diameter", &CircleAnnotation::diameter)
        .def_readwrite("thickness", &CircleAnnotation::thickness)
        .def_readwrite("fillColor", &CircleAnnotation::fillColor)
        .def_readwrite("outlineColor", &CircleAnnotation::outlineColor);

    pointsAnnotationType.value("UNKNOWN", PointsAnnotationType::UNKNOWN)
        .value("POINTS", PointsAnnotationType::POINTS)
        .value("LINE_LOOP", PointsAnnotationType::LINE_LOOP)
        .value("LINE_STRIP", PointsAnnotationType::LINE_STRIP)
        .value("LINE_LIST", PointsAnnotationType::LINE_LIST)
        .export_values();
    pointsAnnotation.def(py::init<>())
        .def_readwrite("type", &PointsAnnotation::type)
        .def_readwrite("points", &PointsAnnotation::points)
        .def_readwrite("outlineColor", &PointsAnnotation::outlineColor)
        .def_readwrite("outlineColors", &PointsAnnotation::outlineColors)
        .def_readwrite("fillColor", &PointsAnnotation::fillColor)
        .def_readwrite("thickness", &PointsAnnotation::thickness);

    textAnnotation.def(py::init<>())
        .def_readwrite("position", &TextAnnotation::position)
        .def_readwrite("text", &TextAnnotation::text)
        .def_readwrite("fontSize", &TextAnnotation::fontSize)
        .def_readwrite("textColor", &TextAnnotation::textColor)
        .def_readwrite("backgroundColor", &TextAnnotation::backgroundColor);

    imageAnnotation.def(py::init<>())
        .def_readwrite("circles", &ImgAnnotation::circles)
        .def_readwrite("points", &ImgAnnotation::points)
        .def_readwrite("texts", &ImgAnnotation::texts);

    // Message
    imageAnnotations.def(py::init<>(), DOC(dai, ImgAnnotations, ImgAnnotations))
        .def(py::init<const std::vector<ImgAnnotation>&>(), DOC(dai, ImgAnnotations, ImgAnnotations, 2))
        .def_readwrite("annotations", &ImgAnnotations::annotations)
        .def("getTimestamp", &ImgAnnotations::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &ImgAnnotations::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &ImgAnnotations::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum));
}
