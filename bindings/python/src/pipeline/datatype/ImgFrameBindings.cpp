#include <cstddef>
#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "depthai/common/RotatedRect.hpp"
#include "pipeline/CommonBindings.hpp"
// depthai
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "ndarray_converter.h"
// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_imgframe(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    // py::class_<RawImgFrame, RawBuffer, std::shared_ptr<RawImgFrame>> rawImgFrame(m, "RawImgFrame", DOC(dai, RawImgFrame));
    py::class_<ImgFrame, Py<ImgFrame>, Buffer, std::shared_ptr<ImgFrame>> imgFrame(m, "ImgFrame", DOC(dai, ImgFrame));
    py::enum_<ImgFrame::Type> imgFrameType(imgFrame, "Type");
    py::class_<ImgFrame::Specs> imgFrameSpecs(imgFrame, "Specs", DOC(dai, ImgFrame, Specs));
    py::class_<ImgTransformation> imgTransformation(m, "ImgTransformation", DOC(dai, ImgTransformation));

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

    // rawImgFrame
    //     .def(py::init<>())
    //     .def_readwrite("fb", &RawImgFrame::fb)
    //     .def_readwrite("category", &RawImgFrame::category)
    //     .def_readwrite("instanceNum", &RawImgFrame::instanceNum)
    //     .def_readwrite("sequenceNum", &RawImgFrame::sequenceNum)
    //     .def_property("ts",
    //         [](const RawImgFrame& o){
    //             double ts = o.ts.sec + o.ts.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawImgFrame& o, double ts){
    //             o.ts.sec = ts;
    //             o.ts.nsec = (ts - o.ts.sec) * 1000000000.0;
    //         }
    //     )
    //     .def_property("tsDevice",
    //         [](const RawImgFrame& o){
    //             double ts = o.tsDevice.sec + o.tsDevice.nsec / 1000000000.0;
    //             return ts;
    //         },
    //         [](RawImgFrame& o, double ts){
    //             o.tsDevice.sec = ts;
    //             o.tsDevice.nsec = (ts - o.tsDevice.sec) * 1000000000.0;
    //         }
    //     )
    //     ;

    imgFrameType.value("YUV422i", ImgFrame::Type::YUV422i)
        .value("YUV444p", ImgFrame::Type::YUV444p)
        .value("YUV420p", ImgFrame::Type::YUV420p)
        .value("YUV422p", ImgFrame::Type::YUV422p)
        .value("YUV400p", ImgFrame::Type::YUV400p)
        .value("RGBA8888", ImgFrame::Type::RGBA8888)
        .value("RGB161616", ImgFrame::Type::RGB161616)
        .value("RGB888p", ImgFrame::Type::RGB888p)
        .value("BGR888p", ImgFrame::Type::BGR888p)
        .value("RGB888i", ImgFrame::Type::RGB888i)
        .value("BGR888i", ImgFrame::Type::BGR888i)
        .value("RGBF16F16F16p", ImgFrame::Type::RGBF16F16F16p)
        .value("BGRF16F16F16p", ImgFrame::Type::BGRF16F16F16p)
        .value("RGBF16F16F16i", ImgFrame::Type::RGBF16F16F16i)
        .value("BGRF16F16F16i", ImgFrame::Type::BGRF16F16F16i)
        .value("GRAY8", ImgFrame::Type::GRAY8)
        .value("GRAYF16", ImgFrame::Type::GRAYF16)
        .value("LUT2", ImgFrame::Type::LUT2)
        .value("LUT4", ImgFrame::Type::LUT4)
        .value("LUT16", ImgFrame::Type::LUT16)
        .value("RAW16", ImgFrame::Type::RAW16)
        .value("RAW14", ImgFrame::Type::RAW14)
        .value("RAW12", ImgFrame::Type::RAW12)
        .value("RAW10", ImgFrame::Type::RAW10)
        .value("RAW8", ImgFrame::Type::RAW8)
        .value("PACK10", ImgFrame::Type::PACK10)
        .value("PACK12", ImgFrame::Type::PACK12)
        .value("YUV444i", ImgFrame::Type::YUV444i)
        .value("NV12", ImgFrame::Type::NV12)
        .value("NV21", ImgFrame::Type::NV21)
        .value("BITSTREAM", ImgFrame::Type::BITSTREAM)
        .value("HDR", ImgFrame::Type::HDR)
        .value("RAW32", ImgFrame::Type::RAW32)
        .value("NONE", ImgFrame::Type::NONE);

    imgFrameSpecs.def(py::init<>())
        .def_readwrite("type", &ImgFrame::Specs::type)
        .def_readwrite("width", &ImgFrame::Specs::width)
        .def_readwrite("height", &ImgFrame::Specs::height)
        .def_readwrite("stride", &ImgFrame::Specs::stride)
        .def_readwrite("bytesPP", &ImgFrame::Specs::bytesPP)
        .def_readwrite("p1Offset", &ImgFrame::Specs::p1Offset)
        .def_readwrite("p2Offset", &ImgFrame::Specs::p2Offset)
        .def_readwrite("p3Offset", &ImgFrame::Specs::p3Offset);

    imgTransformation.def(py::init<>(), DOC(dai, ImgTransformation, ImgTransformation))
        .def(py::init<size_t, size_t>(), py::arg("width"), py::arg("height"), DOC(dai, ImgTransformation, ImgTransformation, 2))
        .def(py::init<size_t, size_t, size_t, size_t>(),
             py::arg("srcWidth"),
             py::arg("srcHeight"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, ImgTransformation, ImgTransformation, 3))
        .def(py::init<size_t, size_t, std::array<std::array<float, 3>, 3>>(),
             py::arg("width"),
             py::arg("height"),
             py::arg("sourceIntrinsicMatrix"),
             DOC(dai, ImgTransformation, ImgTransformation, 4))
        .def(py::init<size_t, size_t, std::array<std::array<float, 3>, 3>, CameraModel, std::vector<float>>(),
             py::arg("width"),
             py::arg("height"),
             py::arg("sourceIntrinsicMatrix"),
             py::arg("distortionModel"),
             py::arg("distortionCoefficients"),
             DOC(dai, ImgTransformation, ImgTransformation, 5))
        .def("__repr__", &ImgTransformation::str)
        .def("transformPoint", &ImgTransformation::transformPoint, py::arg("point"), DOC(dai, ImgTransformation, transformPoint))
        .def("transformRect", &ImgTransformation::transformRect, py::arg("rect"), DOC(dai, ImgTransformation, transformRect))
        .def("invTransformPoint", &ImgTransformation::invTransformPoint, py::arg("point"), DOC(dai, ImgTransformation, invTransformPoint))
        .def("invTransformRect", &ImgTransformation::invTransformRect, py::arg("rect"), DOC(dai, ImgTransformation, invTransformRect))
        .def("getSize", &ImgTransformation::getSize, DOC(dai, ImgTransformation, getSize))
        .def("getSourceSize", &ImgTransformation::getSourceSize, DOC(dai, ImgTransformation, getSourceSize))
        // TODO(Morato) - docstrings don't get generated
        .def("getMatrix", &ImgTransformation::getMatrix)
        .def("getMatrixInv", &ImgTransformation::getMatrixInv)
        .def("getSourceIntrinsicMatrix", &ImgTransformation::getSourceIntrinsicMatrix)
        .def("getSourceIntrinsicMatrixInv", &ImgTransformation::getSourceIntrinsicMatrixInv)
        .def("getIntrinsicMatrix", &ImgTransformation::getIntrinsicMatrix)
        .def("getIntrinsicMatrixInv", &ImgTransformation::getIntrinsicMatrixInv)
        .def("getDistortionModel", &ImgTransformation::getDistortionModel, DOC(dai, ImgTransformation, getDistortionModel))
        .def("getDistortionCoefficients", &ImgTransformation::getDistortionCoefficients, DOC(dai, ImgTransformation, getDistortionCoefficients))
        .def("getSrcCrops", &ImgTransformation::getSrcCrops, DOC(dai, ImgTransformation, getSrcCrops))
        .def("getSrcMaskPt", &ImgTransformation::getSrcMaskPt, py::arg("x"), py::arg("y"), DOC(dai, ImgTransformation, getSrcMaskPt))
        .def("getDstMaskPt", &ImgTransformation::getDstMaskPt, py::arg("x"), py::arg("y"), DOC(dai, ImgTransformation, getDstMaskPt))
        .def("getDFov", &ImgTransformation::getDFov, py::arg("source") = false)
        .def("getHFov", &ImgTransformation::getHFov, py::arg("source") = false)
        .def("getVFov", &ImgTransformation::getVFov, py::arg("source") = false)
        .def("setIntrinsicMatrix", &ImgTransformation::setIntrinsicMatrix, py::arg("intrinsicMatrix"), DOC(dai, ImgTransformation, setIntrinsicMatrix))
        .def("setDistortionModel", &ImgTransformation::setDistortionModel, py::arg("model"), DOC(dai, ImgTransformation, setDistortionModel))
        .def("setDistortionCoefficients",
             &ImgTransformation::setDistortionCoefficients,
             py::arg("coefficients"),
             DOC(dai, ImgTransformation, setDistortionCoefficients))
        .def("addTransformation", &ImgTransformation::addTransformation, py::arg("matrix"), DOC(dai, ImgTransformation, addTransformation))
        .def("addCrop", &ImgTransformation::addCrop, py::arg("x"), py::arg("y"), py::arg("width"), py::arg("height"), DOC(dai, ImgTransformation, addCrop))
        .def("addPadding",
             &ImgTransformation::addPadding,
             py::arg("x"),
             py::arg("y"),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, ImgTransformation, addPadding))
        .def("addFlipVertical", &ImgTransformation::addFlipVertical, DOC(dai, ImgTransformation, addFlipVertical))
        .def("addFlipHorizontal", &ImgTransformation::addFlipHorizontal, DOC(dai, ImgTransformation, addFlipHorizontal))
        .def("addRotation", &ImgTransformation::addRotation, py::arg("angle"), py::arg("rotationPoint"), DOC(dai, ImgTransformation, addRotation))
        .def("addScale", &ImgTransformation::addScale, py::arg("scaleX"), py::arg("scaleY"), DOC(dai, ImgTransformation, addScale))
        .def("remapPointTo", &ImgTransformation::remapPointTo, py::arg("to"), py::arg("point"), DOC(dai, ImgTransformation, remapPointTo))
        .def("remapPointFrom", &ImgTransformation::remapPointFrom, py::arg("to"), py::arg("point"), DOC(dai, ImgTransformation, remapPointFrom))
        .def("remapRectTo", &ImgTransformation::remapRectTo, py::arg("to"), py::arg("rect"), DOC(dai, ImgTransformation, remapRectTo))
        .def("remapRectFrom", &ImgTransformation::remapRectFrom, py::arg("to"), py::arg("rect"), DOC(dai, ImgTransformation, remapRectFrom))
        .def("isValid", &ImgTransformation::isValid, DOC(dai, ImgTransformation, isValid));

    // TODO add RawImgFrame::CameraSettings

    // Message
    imgFrame.def(py::init<>())
        .def(py::init<size_t>())
        .def("__repr__", &ImgFrame::str)
        // getters
        .def("getTimestamp", py::overload_cast<>(&ImgFrame::Buffer::getTimestamp, py::const_), DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", py::overload_cast<>(&ImgFrame::Buffer::getTimestampDevice, py::const_), DOC(dai, Buffer, getTimestampDevice))
        .def("getTimestamp", py::overload_cast<CameraExposureOffset>(&ImgFrame::getTimestamp, py::const_), py::arg("offset"), DOC(dai, ImgFrame, getTimestamp))
        .def("getTimestampDevice",
             py::overload_cast<CameraExposureOffset>(&ImgFrame::getTimestampDevice, py::const_),
             py::arg("offset"),
             DOC(dai, ImgFrame, getTimestampDevice))
        .def("getSequenceNum", &ImgFrame::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getInstanceNum", &ImgFrame::getInstanceNum, DOC(dai, ImgFrame, getInstanceNum))
        .def("getCategory", &ImgFrame::getCategory, DOC(dai, ImgFrame, getCategory))
        .def("getWidth", &ImgFrame::getWidth, DOC(dai, ImgFrame, getWidth))
        .def("getStride", &ImgFrame::getStride, DOC(dai, ImgFrame, getStride))
        .def("getHeight", &ImgFrame::getHeight, DOC(dai, ImgFrame, getHeight))
        .def("getPlaneStride", &ImgFrame::getPlaneStride, DOC(dai, ImgFrame, getPlaneStride))
        .def("getPlaneHeight", &ImgFrame::getPlaneHeight, DOC(dai, ImgFrame, getPlaneHeight))
        .def("getType", &ImgFrame::getType, DOC(dai, ImgFrame, getType))
        .def("getBytesPerPixel", &ImgFrame::getBytesPerPixel, DOC(dai, ImgFrame, getBytesPerPixel))
        .def("getExposureTime", &ImgFrame::getExposureTime, DOC(dai, ImgFrame, getExposureTime))
        .def("getSensitivity", &ImgFrame::getSensitivity, DOC(dai, ImgFrame, getSensitivity))
        .def("getColorTemperature", &ImgFrame::getColorTemperature, DOC(dai, ImgFrame, getColorTemperature))
        .def("getLensPosition", &ImgFrame::getLensPosition, DOC(dai, ImgFrame, getLensPosition))
        .def("getLensPositionRaw", &ImgFrame::getLensPositionRaw, DOC(dai, ImgFrame, getLensPositionRaw))
        .def("getSourceHFov", &ImgFrame::getSourceHFov, DOC(dai, ImgFrame, getSourceHFov))
        .def("getSourceVFov", &ImgFrame::getSourceVFov, DOC(dai, ImgFrame, getSourceVFov))
        .def("getSourceDFov", &ImgFrame::getSourceDFov, DOC(dai, ImgFrame, getSourceDFov))
        .def("getSourceWidth", &ImgFrame::getSourceWidth, DOC(dai, ImgFrame, getSourceWidth))
        .def("getSourceHeight", &ImgFrame::getSourceHeight, DOC(dai, ImgFrame, getSourceHeight))
        .def("getTransformation", [](ImgFrame& msg) { return msg.transformation; })
        .def("validateTransformations", &ImgFrame::validateTransformations, DOC(dai, ImgFrame, validateTransformations))

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
        // The cast function itself does a copy, so we can avoid two copies by always not copying
        .def(
            "getFrame", [](ImgFrame& self) { return self.getFrame(false); }, DOC(dai, ImgFrame, getFrame))
        .def("setFrame", &ImgFrame::setFrame, DOC(dai, ImgFrame, setFrame))
        .def(
            "getCvFrame", [](ImgFrame& self) { return self.getCvFrame(&g_numpyAllocator); }, DOC(dai, ImgFrame, getCvFrame))
        .def("setCvFrame", &ImgFrame::setCvFrame, DOC(dai, ImgFrame, setCvFrame))
#else
        .def(
            "getFrame",
            [](ImgFrame& self, bool copy) {
                throw std::runtime_error("OpenCV support is not available. Please recompile with OpenCV support to use this function.");
            },
            py::arg("copy") = true,
            DOC(dai, ImgFrame, getFrame))
        .def(
            "setFrame",
            [](ImgFrame& self, cv::Mat frame) {
                throw std::runtime_error("OpenCV support is not available. Please recompile with OpenCV support to use this function.");
            },
            DOC(dai, ImgFrame, setFrame))
        .def(
            "getCvFrame",
            []() { throw std::runtime_error("OpenCV support is not available. Please recompile with OpenCV support to use this function."); },
            DOC(dai, ImgFrame, getCvFrame))
        .def(
            "setCvFrame",
            [](cv::Mat frame) { throw std::runtime_error("OpenCV support is not available. Please recompile with OpenCV support to use this function."); },
            DOC(dai, ImgFrame, setCvFrame))
#endif
        // setters
        // .def("setTimestamp", &ImgFrame::setTimestamp, py::arg("timestamp"), DOC(dai, ImgFrame, setTimestamp))
        // .def("setTimestampDevice", &ImgFrame::setTimestampDevice, DOC(dai, ImgFrame, setTimestampDevice))
        .def("setInstanceNum", &ImgFrame::setInstanceNum, py::arg("instance"), DOC(dai, ImgFrame, setInstanceNum))
        .def("setCategory", &ImgFrame::setCategory, py::arg("category"), DOC(dai, ImgFrame, setCategory))
        // .def("setSequenceNum", &ImgFrame::setSequenceNum, py::arg("seq"), DOC(dai, ImgFrame, setSequenceNum))
        .def("setWidth", &ImgFrame::setWidth, py::arg("width"), DOC(dai, ImgFrame, setWidth))
        .def("setStride", &ImgFrame::setStride, py::arg("stride"), DOC(dai, ImgFrame, setStride))
        .def("setHeight", &ImgFrame::setHeight, py::arg("height"), DOC(dai, ImgFrame, setHeight))
        .def("setSize",
             static_cast<ImgFrame& (ImgFrame::*)(unsigned int, unsigned int)>(&ImgFrame::setSize),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, ImgFrame, setSize))
        .def("setSize",
             static_cast<ImgFrame& (ImgFrame::*)(std::tuple<unsigned int, unsigned int>)>(&ImgFrame::setSize),
             py::arg("sizer"),
             DOC(dai, ImgFrame, setSize, 2))
        .def("setType", &ImgFrame::setType, py::arg("type"), DOC(dai, ImgFrame, setType))
        .def("setTransformation", [](ImgFrame& msg, const ImgTransformation& transformation) { msg.transformation = transformation; })
        // .def("set", &ImgFrame::set, py::arg("type"), DOC(dai, ImgFrame, set))
        ;
    // add aliases dai.ImgFrame.Type and dai.ImgFrame.Specs
    // m.attr("ImgFrame").attr("Type") = m.attr("RawImgFrame").attr("Type");
    // m.attr("ImgFrame").attr("Specs") = m.attr("RawImgFrame").attr("Specs");
}
