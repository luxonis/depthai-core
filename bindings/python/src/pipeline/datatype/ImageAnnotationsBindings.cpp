
#include "DatatypeBindings.hpp"
#include "pipeline/CommonBindings.hpp"
#include <unordered_map>
#include <memory>

// depthai
#include "depthai/pipeline/datatype/ImageAnnotations.hpp"

//pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_imageannotations(pybind11::module& m, void* pCallstack){

    using namespace dai;

    py::class_<ImageAnnotations, Py<ImageAnnotations>, Buffer, std::shared_ptr<ImageAnnotations>> imageAnnotations(m, "ImageAnnotations", DOC(dai, ImageAnnotations));
	py::class_<CircleAnnotation> circleAnnotation(m, "CircleAnnotation", DOC(dai, CircleAnnotation));
	py::enum_<PointsAnnotationType>pointsAnnotationType (m, "PointsAnnotationType", DOC(dai, PointsAnnotationType));
	py::class_<PointsAnnotation> pointsAnnotation(m, "PointsAnnotation", DOC(dai, PointsAnnotation));
	py::class_<TextAnnotation> textAnnotation(m, "TextAnnotation", DOC(dai, TextAnnotation));
    py::class_<ImageAnnotation> imageAnnotation(m, "ImageAnnotation", DOC(dai, ImageAnnotation));

    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    // Call the rest of the type defines, then perform the actual bindings
    Callstack* callstack = (Callstack*) pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);
    // Actual bindings
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////

    // Metadata / raw
	circleAnnotation
		.def(py::init<>())
		.def_readwrite("position", &CircleAnnotation::position)
		.def_readwrite("diameter", &CircleAnnotation::diameter)
		.def_readwrite("thickness", &CircleAnnotation::thickness)
		.def_readwrite("fillColor", &CircleAnnotation::fillColor)
		.def_readwrite("outlineColor", &CircleAnnotation::outlineColor)
		;

	pointsAnnotationType
		.value("UNKNOWN", PointsAnnotationType::UNKNOWN)
		.value("POINTS", PointsAnnotationType::POINTS)
		.value("LINE_STRIP", PointsAnnotationType::LINE_LOOP)
		.value("LINE_LOOP", PointsAnnotationType::LINE_STRIP)
		.value("POLYGON", PointsAnnotationType::LINE_LIST)
		.export_values();
	pointsAnnotation
		.def(py::init<>())
		.def_readwrite("type", &PointsAnnotation::type)
		.def_readwrite("points", &PointsAnnotation::points)
		.def_readwrite("outlineColor", &PointsAnnotation::outlineColor)
		.def_readwrite("outlineColors", &PointsAnnotation::outlineColors)
		.def_readwrite("fillColor", &PointsAnnotation::fillColor)
		.def_readwrite("thickness", &PointsAnnotation::thickness)
		;

	textAnnotation
		.def(py::init<>())
		.def_readwrite("position", &TextAnnotation::position)
		.def_readwrite("text", &TextAnnotation::text)
		.def_readwrite("fontSize", &TextAnnotation::fontSize)
		.def_readwrite("textColor", &TextAnnotation::textColor)
		.def_readwrite("backgroundColor", &TextAnnotation::backgroundColor)
		;
		
    imageAnnotation
        .def(py::init<>())
		.def_readwrite("circles", &ImageAnnotation::circles)
		.def_readwrite("points", &ImageAnnotation::points)
		.def_readwrite("texts", &ImageAnnotation::texts)
		;	


    // Message
    imageAnnotations
        .def(py::init<>(), DOC(dai, ImageAnnotations, ImageAnnotations))
		.def(py::init<const std::vector<ImageAnnotation>&>(), DOC(dai, ImageAnnotations, ImageAnnotations, 2))
		.def_readwrite("annotations", &ImageAnnotations::annotations)
        .def("getTimestamp", &ImageAnnotations::Buffer::getTimestamp, DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", &ImageAnnotations::Buffer::getTimestampDevice, DOC(dai, Buffer, getTimestampDevice))
        .def("getSequenceNum", &ImageAnnotations::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        ;


}
