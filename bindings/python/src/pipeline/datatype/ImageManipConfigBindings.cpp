#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "depthai/common/Point2f.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/ImageManipConfig.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

// #include "spdlog/spdlog.h"

void bind_imagemanipconfig(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<ImageManipConfig, Py<ImageManipConfig>, Buffer, std::shared_ptr<ImageManipConfig>> imageManipConfig(
        m, "ImageManipConfig", DOC(dai, ImageManipConfig));
    py::enum_<ImageManipConfig::ResizeMode> resizeMode(imageManipConfig, "ResizeMode");

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

    resizeMode.value("NONE", ImageManipConfig::ResizeMode::NONE)
        .value("LETTERBOX", ImageManipConfig::ResizeMode::LETTERBOX)
        .value("CENTER_CROP", ImageManipConfig::ResizeMode::CENTER_CROP)
        .value("STRETCH", ImageManipConfig::ResizeMode::STRETCH);

    // Message

    imageManipConfig.def(py::init<>())
        .def("__repr__", &ImageManipConfig::str)
        // New API Setters
        .def("clearOps", &ImageManipConfig::clearOps, DOC(dai, ImageManipConfig, clearOps))
        .def("addCrop",
             static_cast<ImageManipConfig& (ImageManipConfig::*)(uint32_t, uint32_t, uint32_t, uint32_t)>(&ImageManipConfig::addCrop),
             py::arg("x"),
             py::arg("y"),
             py::arg("w"),
             py::arg("h"),
             DOC(dai, ImageManipConfig, addCrop))
        .def("addCrop",
             static_cast<ImageManipConfig& (ImageManipConfig::*)(dai::Rect, bool)>(&ImageManipConfig::addCrop),
             py::arg("rect"),
             py::arg("normalizedCoords"),
             DOC(dai, ImageManipConfig, addCrop))
        .def("addCropRotatedRect",
             &ImageManipConfig::addCropRotatedRect,
             py::arg("rect"),
             py::arg("normalizedCoords"),
             DOC(dai, ImageManipConfig, addCropRotatedRect))
        .def(
            "addScale", [](ImageManipConfig& self, float scale) { return self.addScale(scale); }, py::arg("scale"), DOC(dai, ImageManipConfig, addScale))
        .def(
            "addScale",
            [](ImageManipConfig& self, float scaleX, float scaleY) { return self.addScale(scaleX, scaleY); },
            py::arg("scaleX"),
            py::arg("scaleY"),
            DOC(dai, ImageManipConfig, addScale))
        .def("addFlipHorizontal", &ImageManipConfig::addFlipHorizontal, DOC(dai, ImageManipConfig, addFlipHorizontal))
        .def("addFlipVertical", &ImageManipConfig::addFlipVertical, DOC(dai, ImageManipConfig, addFlipVertical))
        .def("addRotateDeg",
             static_cast<ImageManipConfig& (ImageManipConfig::*)(float)>(&ImageManipConfig::addRotateDeg),
             py::arg("angle"),
             DOC(dai, ImageManipConfig, addRotateDeg))
        .def("addRotateDeg",
             static_cast<ImageManipConfig& (ImageManipConfig::*)(float, Point2f)>(&ImageManipConfig::addRotateDeg),
             py::arg("angle"),
             py::arg("center"),
             DOC(dai, ImageManipConfig, addRotateDeg))
        .def("addTransformAffine", &ImageManipConfig::addTransformAffine, py::arg("mat"), DOC(dai, ImageManipConfig, addTransformAffine))
        .def("addTransformPerspective", &ImageManipConfig::addTransformPerspective, py::arg("mat"), DOC(dai, ImageManipConfig, addTransformPerspective))
        .def("addTransformFourPoints",
             &ImageManipConfig::addTransformFourPoints,
             py::arg("src"),
             py::arg("dst"),
             py::arg("normalizedCoords"),
             DOC(dai, ImageManipConfig, addTransformFourPoints))
        .def("setColormap", &ImageManipConfig::setColormap, py::arg("colormap"), DOC(dai, ImageManipConfig, setColormap))
        .def("setBackgroundColor",
             static_cast<ImageManipConfig& (ImageManipConfig::*)(uint32_t, uint32_t, uint32_t)>(&ImageManipConfig::setBackgroundColor),
             py::arg("r"),
             py::arg("g"),
             py::arg("b"),
             DOC(dai, ImageManipConfig, setBackgroundColor))
        .def("setBackgroundColor",
             static_cast<ImageManipConfig& (ImageManipConfig::*)(uint32_t)>(&ImageManipConfig::setBackgroundColor),
             py::arg("val"),
             DOC(dai, ImageManipConfig, setBackgroundColor))
        .def("setOutputSize",
             &ImageManipConfig::setOutputSize,
             py::arg("w"),
             py::arg("h"),
             py::arg("mode") = ImageManipConfig::ResizeMode::STRETCH,
             DOC(dai, ImageManipConfig, setOutputSize))
        .def("setOutputCenter", &ImageManipConfig::setOutputCenter, py::arg("c"), DOC(dai, ImageManipConfig, setOutputCenter))
        .def("setReusePreviousImage", &ImageManipConfig::setReusePreviousImage, py::arg("reuse"), DOC(dai, ImageManipConfig, setReusePreviousImage))
        .def("setSkipCurrentImage", &ImageManipConfig::setSkipCurrentImage, py::arg("skip"), DOC(dai, ImageManipConfig, setSkipCurrentImage))
        .def("setFrameType", &ImageManipConfig::setFrameType, py::arg("type"), DOC(dai, ImageManipConfig, setFrameType))
        .def("setUndistort", &ImageManipConfig::setUndistort, py::arg("undistort"), DOC(dai, ImageManipConfig, setUndistort))
        .def("getUndistort", &ImageManipConfig::getUndistort, DOC(dai, ImageManipConfig, getUndistort))
        .def("setColormap",
             static_cast<ImageManipConfig& (ImageManipConfig::*)(Colormap)>(&ImageManipConfig::setColormap),
             py::arg("colormap"),
             DOC(dai, ImageManipConfig, setColormap));
}
