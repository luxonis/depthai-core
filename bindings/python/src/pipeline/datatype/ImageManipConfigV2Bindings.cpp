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

    // py::class_<RawImageManipConfig, RawBuffer, std::shared_ptr<RawImageManipConfig>> rawImageManipConfig(m, "RawImageManipConfig", DOC(dai,
    // RawImageManipConfig));
    py::class_<ImageManipConfigV2, Py<ImageManipConfigV2>, Buffer, std::shared_ptr<ImageManipConfigV2>> imageManipConfig(
        m, "ImageManipConfigV2", DOC(dai, ImageManipConfigV2));

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

    // Message

    imageManipConfig
        .def(py::init<>())
        // New API Setters
        .def("crop", &ImageManipConfigV2::crop, py::arg("x"), py::arg("y"), py::arg("w"), py::arg("h"), DOC(dai, ImageManipConfigV2, crop))
        .def("resize", &ImageManipConfigV2::resize, py::arg("w"), py::arg("h"), DOC(dai, ImageManipConfigV2, resize))
        .def("scale", &ImageManipConfigV2::scale, py::arg("scaleX"), py::arg("scaleY"), DOC(dai, ImageManipConfigV2, scale))
        .def("flipHorizontal", &ImageManipConfigV2::flipHorizontal, DOC(dai, ImageManipConfigV2, flipHorizontal))
        .def("flipVertical", &ImageManipConfigV2::flipVertical, DOC(dai, ImageManipConfigV2, flipVertical))
        .def("rotateDeg",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(float)>(&ImageManipConfigV2::rotateDeg),
             py::arg("angle"),
             DOC(dai, ImageManipConfigV2, rotateDeg))
        .def("rotateDeg",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(float, Point2f)>(&ImageManipConfigV2::rotateDeg),
             py::arg("angle"),
             py::arg("center"),
             DOC(dai, ImageManipConfigV2, rotateDeg))
        .def(
            "setOutputSize",
            [](ImageManipConfigV2& self, float w, float h, std::string mode) -> ImageManipConfigV2& {
                auto m = ImageManipConfigV2::ResizeMode::NONE;
                if(mode == "LETTERBOX")
                    m = ImageManipConfigV2::ResizeMode::LETTERBOX;
                else if(mode == "CENTER_CROP")
                    m = ImageManipConfigV2::ResizeMode::CENTER_CROP;
                else if(mode == "STRETCH")
                    m = ImageManipConfigV2::ResizeMode::STRETCH;
                self.setOutputSize(w, h, m);
                return self;
            },
            py::arg("w"),
            py::arg("h"),
            py::arg("mode"),
            DOC(dai, ImageManipConfigV2, setOutputSize))
        .def("setFrameType", &ImageManipConfigV2::setFrameType, py::arg("type"), DOC(dai, ImageManipConfigV2, setFrameType))
        .def("setColormap",
             static_cast<ImageManipConfigV2& (ImageManipConfigV2::*)(Colormap)>(&ImageManipConfigV2::setColormap),
             py::arg("colormap"),
             DOC(dai, ImageManipConfigV2, setColormap));
}
