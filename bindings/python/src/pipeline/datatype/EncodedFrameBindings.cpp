#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "pipeline/CommonBindings.hpp"

#include "depthai/pipeline/datatype/EncodedFrame.hpp"

#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_encodedframe(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<EncodedFrame, Py<EncodedFrame>, Buffer, ProtoSerializable, py::smart_holder> encodedFrame(
        m, "EncodedFrame", DOC(dai, EncodedFrame));
    py::enum_<EncodedFrame::Profile> encodedFrameProfile(encodedFrame, "Profile");
    py::enum_<EncodedFrame::FrameType> encodedFrameType(encodedFrame, "FrameType", DOC(dai, EncodedFrame, FrameType));

    Callstack* callstack = (Callstack*)pCallstack;
    auto cb = callstack->top();
    callstack->pop();
    cb(m, pCallstack);

    encodedFrameProfile.value("JPEG", EncodedFrame::Profile::JPEG).value("AVC", EncodedFrame::Profile::AVC).value("HEVC", EncodedFrame::Profile::HEVC);

    encodedFrameType.value("I", EncodedFrame::FrameType::I)
        .value("P", EncodedFrame::FrameType::P)
        .value("B", EncodedFrame::FrameType::B)
        .value("Unknown", EncodedFrame::FrameType::Unknown);

    encodedFrame.def(py::init<>())
        .def_readwrite("cam", &dai::EncodedFrame::cam, DOC(dai, EncodedFrame, cam))
        .def_readwrite("frameOffset", &dai::EncodedFrame::frameOffset, DOC(dai, EncodedFrame, frameOffset))
        .def_readwrite("frameSize", &dai::EncodedFrame::frameSize, DOC(dai, EncodedFrame, frameSize))
        .def_readwrite("width", &dai::EncodedFrame::width, DOC(dai, EncodedFrame, width))
        .def_readwrite("height", &dai::EncodedFrame::height, DOC(dai, EncodedFrame, height))
        .def_readwrite("quality", &dai::EncodedFrame::quality, DOC(dai, EncodedFrame, quality))
        .def_readwrite("bitrate", &dai::EncodedFrame::bitrate, DOC(dai, EncodedFrame, bitrate))
        .def_readwrite("profile", &dai::EncodedFrame::profile, DOC(dai, EncodedFrame, profile))
        .def_readwrite("lossless", &dai::EncodedFrame::lossless, DOC(dai, EncodedFrame, lossless))
        .def_readwrite("type", &dai::EncodedFrame::type, DOC(dai, EncodedFrame, type))
        .def_readwrite("instanceNum", &dai::EncodedFrame::instanceNum, DOC(dai, EncodedFrame, instanceNum))
        .def_readwrite("transformation", &dai::EncodedFrame::transformation, DOC(dai, EncodedFrame, transformation))
        .def("setInstanceNum",
             &dai::EncodedFrame::setInstanceNum,
             py::arg("instance"),
             py::return_value_policy::reference_internal,
             DOC(dai, EncodedFrame, setInstanceNum))
        .def("getImgFrameMeta", &dai::EncodedFrame::getImgFrameMeta, DOC(dai, EncodedFrame, getImgFrameMeta));

    encodedFrame.def("__repr__", &EncodedFrame::str)
        .def("getInstanceNum", &EncodedFrame::getInstanceNum, DOC(dai, EncodedFrame, getInstanceNum))
        .def("getWidth", &EncodedFrame::getWidth, DOC(dai, EncodedFrame, getWidth))
        .def("getHeight", &EncodedFrame::getHeight, DOC(dai, EncodedFrame, getHeight))
        .def("getExposureTime", &EncodedFrame::getExposureTime, DOC(dai, EncodedFrame, getExposureTime))
        .def("getSensitivity", &EncodedFrame::getSensitivity, DOC(dai, EncodedFrame, getSensitivity))
        .def("getColorTemperature", &EncodedFrame::getColorTemperature, DOC(dai, EncodedFrame, getColorTemperature))
        .def("getLensPosition", &EncodedFrame::getLensPosition, DOC(dai, EncodedFrame, getLensPosition))
        .def("getLensPositionRaw", &EncodedFrame::getLensPositionRaw, DOC(dai, EncodedFrame, getLensPositionRaw))
        .def("getFsync", &EncodedFrame::getFsync, DOC(dai, EncodedFrame, getFsync))
        .def("getSensorMode", &EncodedFrame::getSensorMode, DOC(dai, EncodedFrame, getSensorMode))
        .def("getFps", &EncodedFrame::getFps, DOC(dai, EncodedFrame, getFps))
        .def("getSensorTemperature", &EncodedFrame::getSensorTemperature, DOC(dai, EncodedFrame, getSensorTemperature))
        .def("getQuality", &EncodedFrame::getQuality, DOC(dai, EncodedFrame, getQuality))
        .def("getBitrate", &EncodedFrame::getBitrate, DOC(dai, EncodedFrame, getBitrate))
        .def("getFrameType", &EncodedFrame::getFrameType, DOC(dai, EncodedFrame, getFrameType))
        .def("getLossless", &EncodedFrame::getLossless, DOC(dai, EncodedFrame, getLossless))
        .def("getProfile", &EncodedFrame::getProfile, DOC(dai, EncodedFrame, getProfile))
        .def("getTransformation", [](EncodedFrame& msg) { return msg.transformation; })
        .def("setWidth", &EncodedFrame::setWidth, py::arg("width"), DOC(dai, EncodedFrame, setWidth))
        .def("setHeight", &EncodedFrame::setHeight, py::arg("height"), DOC(dai, EncodedFrame, setHeight))
        .def("setSize",
             static_cast<EncodedFrame& (EncodedFrame::*)(unsigned int, unsigned int)>(&EncodedFrame::setSize),
             py::arg("width"),
             py::arg("height"),
             DOC(dai, EncodedFrame, setSize))
        .def("setSize",
             static_cast<EncodedFrame& (EncodedFrame::*)(std::tuple<unsigned int, unsigned int>)>(&EncodedFrame::setSize),
             py::arg("sizer"),
             DOC(dai, EncodedFrame, setSize, 2))
        .def("setQuality", &EncodedFrame::setQuality, py::arg("quality"), DOC(dai, EncodedFrame, setQuality))
        .def("setBitrate", &EncodedFrame::setBitrate, py::arg("bitrate"), DOC(dai, EncodedFrame, setBitrate))
        .def("setFrameType", &EncodedFrame::setFrameType, py::arg("frameType"), DOC(dai, EncodedFrame, setFrameType))
        .def("setLossless", &EncodedFrame::setLossless, py::arg("lossless"), DOC(dai, EncodedFrame, setLossless))
        .def("setProfile", &EncodedFrame::setProfile, py::arg("profile"), DOC(dai, EncodedFrame, setProfile))
        .def("setTransformation", [](EncodedFrame& msg, const ImgTransformation& transformation) { msg.transformation = transformation; });
}
