#include <memory>
#include <unordered_map>

#include "DatatypeBindings.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "pipeline/CommonBindings.hpp"

// depthai
#include "depthai/pipeline/datatype/EncodedFrame.hpp"

// pybind
#include <pybind11/chrono.h>
#include <pybind11/numpy.h>

void bind_encodedframe(pybind11::module& m, void* pCallstack) {
    using namespace dai;

    py::class_<EncodedFrame, Py<EncodedFrame>, Buffer, std::shared_ptr<EncodedFrame>> encodedFrame(m, "EncodedFrame", DOC(dai, EncodedFrame));
    py::enum_<EncodedFrame::Profile> encodedFrameProfile(encodedFrame, "Profile");
    py::enum_<EncodedFrame::FrameType> encodedFrameType(encodedFrame, "FrameType", DOC(dai, EncodedFrame, FrameType));

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

    //   rawEncodedFrame.def(py::init<>())
    //       .def_readwrite("quality", &RawEncodedFrame::quality)
    //       .def_readwrite("bitrate", &RawEncodedFrame::bitrate)
    //       .def_readwrite("profile", &RawEncodedFrame::profile)
    //       .def_readwrite("lossless", &RawEncodedFrame::lossless)
    //       .def_readwrite("type", &RawEncodedFrame::type)
    //       .def_readwrite("instanceNum", &RawEncodedFrame::instanceNum)
    //       .def_readwrite("sequenceNum", &RawEncodedFrame::sequenceNum)
    //       .def_property(
    //           "ts",
    //           [](const RawEncodedFrame &o) {
    //             double ts = o.ts.sec + o.ts.nsec / 1000000000.0;
    //             return ts;
    //           },
    //           [](RawEncodedFrame &o, double ts) {
    //             o.ts.sec = ts;
    //             o.ts.nsec = (ts - o.ts.sec) * 1000000000.0;
    //           })
    //       .def_property(
    //           "tsDevice",
    //           [](const RawEncodedFrame &o) {
    //             double ts = o.tsDevice.sec + o.tsDevice.nsec / 1000000000.0;
    //             return ts;
    //           },
    //           [](RawEncodedFrame &o, double ts) {
    //             o.tsDevice.sec = ts;
    //             o.tsDevice.nsec = (ts - o.tsDevice.sec) * 1000000000.0;
    //           });

    encodedFrameProfile.value("JPEG", EncodedFrame::Profile::JPEG).value("AVC", EncodedFrame::Profile::AVC).value("HEVC", EncodedFrame::Profile::HEVC);

    encodedFrameType.value("I", EncodedFrame::FrameType::I)
        .value("P", EncodedFrame::FrameType::P)
        .value("B", EncodedFrame::FrameType::B)
        .value("Unknown", EncodedFrame::FrameType::Unknown);

    // Message
    encodedFrame.def(py::init<>())
        .def("__repr__", &EncodedFrame::str)
        // getters
        .def("getTimestamp", py::overload_cast<>(&EncodedFrame::Buffer::getTimestamp, py::const_), DOC(dai, Buffer, getTimestamp))
        .def("getTimestampDevice", py::overload_cast<>(&EncodedFrame::Buffer::getTimestampDevice, py::const_), DOC(dai, Buffer, getTimestampDevice))
        .def("getInstanceNum", &EncodedFrame::getInstanceNum, DOC(dai, EncodedFrame, getInstanceNum))
        .def("getWidth", &EncodedFrame::getWidth, DOC(dai, EncodedFrame, getWidth))
        .def("getHeight", &EncodedFrame::getHeight, DOC(dai, EncodedFrame, getHeight))
        .def("getSequenceNum", &EncodedFrame::Buffer::getSequenceNum, DOC(dai, Buffer, getSequenceNum))
        .def("getExposureTime", &EncodedFrame::getExposureTime, DOC(dai, EncodedFrame, getExposureTime))
        .def("getSensitivity", &EncodedFrame::getSensitivity, DOC(dai, EncodedFrame, getSensitivity))
        .def("getColorTemperature", &EncodedFrame::getColorTemperature, DOC(dai, EncodedFrame, getColorTemperature))
        .def("getLensPosition", &EncodedFrame::getLensPosition, DOC(dai, EncodedFrame, getLensPosition))
        .def("getLensPositionRaw", &EncodedFrame::getLensPositionRaw, DOC(dai, EncodedFrame, getLensPositionRaw))
        .def("getQuality", &EncodedFrame::getQuality, DOC(dai, EncodedFrame, getQuality))
        .def("getBitrate", &EncodedFrame::getBitrate, DOC(dai, EncodedFrame, getBitrate))
        .def("getFrameType", &EncodedFrame::getFrameType, DOC(dai, EncodedFrame, getFrameType))
        .def("getLossless", &EncodedFrame::getLossless, DOC(dai, EncodedFrame, getLossless))
        .def("getProfile", &EncodedFrame::getProfile, DOC(dai, EncodedFrame, getProfile))
        .def("getTransformation", [](EncodedFrame& msg) { return msg.transformation; })
        //  // setters
        //  .def("setTimestamp", &EncodedFrame::setTimestamp,
        //       DOC(dai, EncodedFrame, setTimestamp))
        //  .def("setTimestampDevice", &EncodedFrame::setTimestampDevice,
        //       DOC(dai, EncodedFrame, setTimestampDevice))
        //  .def("setSequenceNum", &EncodedFrame::setSequenceNum,
        //       DOC(dai, EncodedFrame, setSequenceNum))
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
        .def("setQuality", &EncodedFrame::setQuality, DOC(dai, EncodedFrame, getQuality))
        .def("setBitrate", &EncodedFrame::setBitrate, DOC(dai, EncodedFrame, getBitrate))
        .def("setFrameType", &EncodedFrame::setFrameType, DOC(dai, EncodedFrame, getFrameType))
        .def("setLossless", &EncodedFrame::setLossless, DOC(dai, EncodedFrame, getLossless))
        .def("setProfile", &EncodedFrame::setProfile, DOC(dai, EncodedFrame, getProfile))
        .def("setTransformation", [](EncodedFrame& msg, const ImgTransformation& transformation) { msg.transformation = transformation; });
    //   // add aliases dai.ImgFrame.Type and dai.ImgFrame.Specs
    //   m.attr("EncodedFrame").attr("FrameType") =
    //       m.attr("RawEncodedFrame").attr("FrameType");
    //   m.attr("EncodedFrame").attr("Profile") =
    //       m.attr("RawEncodedFrame").attr("Profile");
}
