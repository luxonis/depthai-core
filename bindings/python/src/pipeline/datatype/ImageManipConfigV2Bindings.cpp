#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "depthai/common/Point2f.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ImageManipConfigV2.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_imagemanipconfigv2(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ImageManipConfigV2, Py<ImageManipConfigV2>, Buffer, std::shared_ptr<ImageManipConfigV2>> imageManipConfig(
        m, "ImageManipConfigV2", DOC(dai, ImageManipConfigV2));
    py::enum_<ImageManipConfigV2::ResizeMode> resizeMode(imageManipConfig, "ResizeMode");

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

    resizeMode.value("NONE", ImageManipConfigV2::ResizeMode::NONE)
        .value("LETTERBOX", ImageManipConfigV2::ResizeMode::LETTERBOX)
        .value("CENTER_CROP", ImageManipConfigV2::ResizeMode::CENTER_CROP)
        .value("STRETCH", ImageManipConfigV2::ResizeMode::STRETCH);

    // Message

    imageManipConfig
        .def(py::init<>())
        // New API Setters
        .def("clearOps", &ImageManipConfigV2::clearOps, DOC(dai, ImageManipConfigV2, clearOps))
        .def("addCrop", static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(uint32_t, uint32_t, uint32_t, uint32_t)>(&ImageManipConfigV2::addCrop), py::arg("x"), py::arg("y"), py::arg("w"), py::arg("h"), DOC(dai, ImageManipConfigV2, addCrop))
        .def("addCrop", static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(dai::Rect, bool)>(&ImageManipConfigV2::addCrop), py::arg("rect"), py::arg("normalizedCoords"), DOC(dai, ImageManipConfigV2, addCrop))
        .def("addCropRotatedRect",
             &ImageManipConfigV2::addCropRotatedRect,
             py::arg("rect"),
             py::arg("normalizedCoords"),
             DOC(dai, ImageManipConfigV2, addCropRotatedRect))
        .def("addResize", &ImageManipConfigV2::addResize, py::arg("w"), py::arg("h"), DOC(dai, ImageManipConfigV2, addResize))
        .def(
            "addScale", [](ImageManipConfigV2& self, float scale) { return self.addScale(scale); }, py::arg("scale"), DOC(dai, ImageManipConfigV2, addScale))
        .def(
            "addScale",
            [](ImageManipConfigV2& self, float scaleX, float scaleY) { return self.addScale(scaleX, scaleY); },
            py::arg("scaleX"),
            py::arg("scaleY"),
            DOC(dai, ImageManipConfigV2, addScale))
        .def("addFlipHorizontal", &ImageManipConfigV2::addFlipHorizontal, DOC(dai, ImageManipConfigV2, addFlipHorizontal))
        .def("addFlipVertical", &ImageManipConfigV2::addFlipVertical, DOC(dai, ImageManipConfigV2, addFlipVertical))
        .def("addRotateDeg",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(float)>(&ImageManipConfigV2::addRotateDeg),
             py::arg("angle"),
             DOC(dai, ImageManipConfigV2, addRotateDeg))
        .def("addRotateDeg",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(float, Point2f)>(&ImageManipConfigV2::addRotateDeg),
             py::arg("angle"),
             py::arg("center"),
             DOC(dai, ImageManipConfigV2, addRotateDeg))
        .def("addTransformAffine", &ImageManipConfigV2::addTransformAffine, py::arg("mat"), DOC(dai, ImageManipConfigV2, addTransformAffine))
        .def("addTransformPerspective", &ImageManipConfigV2::addTransformPerspective, py::arg("mat"), DOC(dai, ImageManipConfigV2, addTransformPerspective))
        .def("addTransformFourPoints",
             &ImageManipConfigV2::addTransformFourPoints,
             py::arg("src"),
             py::arg("dst"),
             py::arg("normalizedCoords"),
             DOC(dai, ImageManipConfigV2, addTransformFourPoints))
        .def("setColormap", &ImageManipConfigV2::setColormap, py::arg("colormap"), DOC(dai, ImageManipConfigV2, setColormap))
        .def("setBackgroundColor",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(uint8_t, uint8_t, uint8_t)>(&ImageManipConfigV2::setBackgroundColor),
             py::arg("r"),
             py::arg("g"),
             py::arg("b"),
             DOC(dai, ImageManipConfigV2, setBackgroundColor))
        .def("setBackgroundColor",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(uint8_t)>(&ImageManipConfigV2::setBackgroundColor),
             py::arg("val"),
             DOC(dai, ImageManipConfigV2, setBackgroundColor))
        .def("setOutputSize", &ImageManipConfigV2::setOutputSize, py::arg("w"), py::arg("h"), py::arg("mode") = ImageManipConfigV2::ResizeMode::STRETCH, DOC(dai, ImageManipConfigV2, setOutputSize))
        .def("setOutputCenter", &ImageManipConfigV2::setOutputCenter, py::arg("c"), DOC(dai, ImageManipConfigV2, setOutputCenter))
        .def("setReusePreviousImage", &ImageManipConfigV2::setReusePreviousImage, py::arg("reuse"), DOC(dai, ImageManipConfigV2, setReusePreviousImage))
        .def("setSkipCurrentImage", &ImageManipConfigV2::setSkipCurrentImage, py::arg("skip"), DOC(dai, ImageManipConfigV2, setSkipCurrentImage))
        .def("setFrameType", &ImageManipConfigV2::setFrameType, py::arg("type"), DOC(dai, ImageManipConfigV2, setFrameType))
        .def("setColormap",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(Colormap)>(&ImageManipConfigV2::setColormap),
             py::arg("colormap"),
             DOC(dai, ImageManipConfigV2, setColormap));
}
