#include "Common.hpp"
#include "NodeBindings.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"

void bind_videoencoder(pybind11::module& m, void* pCallstack) {
    using namespace dai;
    using namespace dai::node;

    // Node and Properties declare upfront
    py::class_<VideoEncoderProperties> videoEncoderProperties(m, "VideoEncoderProperties", DOC(dai, VideoEncoderProperties));
    py::enum_<VideoEncoderProperties::Profile> videoEncoderPropertiesProfile(videoEncoderProperties, "Profile", DOC(dai, VideoEncoderProperties, Profile));
    py::enum_<VideoEncoderProperties::RateControlMode> videoEncoderPropertiesProfileRateControlMode(
        videoEncoderProperties, "RateControlMode", DOC(dai, VideoEncoderProperties, RateControlMode));
    auto videoEncoder = ADD_NODE(VideoEncoder);

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

    // Properties
    videoEncoderPropertiesProfile.value("H264_BASELINE", VideoEncoderProperties::Profile::H264_BASELINE)
        .value("H264_HIGH", VideoEncoderProperties::Profile::H264_HIGH)
        .value("H264_MAIN", VideoEncoderProperties::Profile::H264_MAIN)
        .value("H265_MAIN", VideoEncoderProperties::Profile::H265_MAIN)
        .value("MJPEG", VideoEncoderProperties::Profile::MJPEG);

    videoEncoderPropertiesProfileRateControlMode.value("CBR", VideoEncoderProperties::RateControlMode::CBR)
        .value("VBR", VideoEncoderProperties::RateControlMode::VBR);

    videoEncoderProperties.def_readwrite("bitrate", &VideoEncoderProperties::bitrate)
        .def_readwrite("keyframeFrequency", &VideoEncoderProperties::keyframeFrequency)
        .def_readwrite("maxBitrate", &VideoEncoderProperties::maxBitrate)
        .def_readwrite("numBFrames", &VideoEncoderProperties::numBFrames)
        .def_readwrite("numFramesPool", &VideoEncoderProperties::numFramesPool)
        .def_readwrite("profile", &VideoEncoderProperties::profile)
        .def_readwrite("quality", &VideoEncoderProperties::quality)
        .def_readwrite("rateCtrlMode", &VideoEncoderProperties::rateCtrlMode)
        .def_readwrite("outputFrameSize", &VideoEncoderProperties::outputFrameSize);

    // Node
    videoEncoder
#define VIDEO_ENCODER_BUILD_ARGS Node::Output& input
#define VIDEO_ENCODER_BUILD_PYARGS py::arg("input")
#define VIDEO_ENCODER_ARGS float bitrate, float frameRate, VideoEncoderProperties::Profile profile, int keyframeFrequency, bool lossless, int quality
#define VIDEO_ENCODER_PYARGS                                                                                                          \
    py::arg("bitrate") = VideoEncoderProperties().bitrate, py::arg("frameRate") = VideoEncoderProperties().frameRate,                 \
    py::arg("profile") = VideoEncoderProperties().profile, py::arg("keyframeFrequency") = VideoEncoderProperties().keyframeFrequency, \
    py::arg("lossless") = VideoEncoderProperties().lossless, py::arg("quality") = VideoEncoderProperties().quality
    // TODO (Zimamazim) Automatically fetch default arguments to avoid duplicity
#define VIDEO_ENCODER_CODE(OP)                       \
    self OP setBitrate(bitrate);                     \
    self OP setFrameRate(frameRate);                 \
    self OP setProfile(profile);                     \
    self OP setKeyframeFrequency(keyframeFrequency); \
    self OP setLossless(lossless);                   \
    self OP setQuality(quality);
        .def(
            "build",
            [](VideoEncoder& self, VIDEO_ENCODER_BUILD_ARGS, VIDEO_ENCODER_ARGS) {
                self.build(input);
                VIDEO_ENCODER_CODE(.)
                return std::static_pointer_cast<VideoEncoder>(self.shared_from_this());
            },
            VIDEO_ENCODER_BUILD_PYARGS,
            VIDEO_ENCODER_PYARGS)
        .def(py::init([](VIDEO_ENCODER_BUILD_ARGS, VIDEO_ENCODER_ARGS) {
                 auto self = getImplicitPipeline()->create<VideoEncoder>();
                 self->build(input);
                 VIDEO_ENCODER_CODE(->)
                 return self;
             }),
             VIDEO_ENCODER_BUILD_PYARGS,
             VIDEO_ENCODER_PYARGS)
        .def_readonly("input", &VideoEncoder::input, DOC(dai, node, VideoEncoder, input), DOC(dai, node, VideoEncoder, input))
        .def_readonly("bitstream", &VideoEncoder::bitstream, DOC(dai, node, VideoEncoder, bitstream), DOC(dai, node, VideoEncoder, bitstream))
        .def_readonly("out", &VideoEncoder::out, DOC(dai, node, VideoEncoder, out), DOC(dai, node, VideoEncoder, out))
        .def("setDefaultProfilePreset",
             static_cast<void (VideoEncoder::*)(float, VideoEncoderProperties::Profile)>(&VideoEncoder::setDefaultProfilePreset),
             py::arg("fps"),
             py::arg("profile"),
             DOC(dai, node, VideoEncoder, setDefaultProfilePreset))
        .def("setNumFramesPool", &VideoEncoder::setNumFramesPool, py::arg("frames"), DOC(dai, node, VideoEncoder, setNumFramesPool))
        .def("getNumFramesPool", &VideoEncoder::getNumFramesPool, DOC(dai, node, VideoEncoder, getNumFramesPool))
        .def("setRateControlMode", &VideoEncoder::setRateControlMode, py::arg("mode"), DOC(dai, node, VideoEncoder, setRateControlMode))
        .def("setProfile",
             static_cast<void (VideoEncoder::*)(VideoEncoder::Properties::Profile)>(&VideoEncoder::setProfile),
             py::arg("profile"),
             DOC(dai, node, VideoEncoder, setProfile))
        .def("setBitrate", &VideoEncoder::setBitrate, py::arg("bitrate"), DOC(dai, node, VideoEncoder, setBitrate))
        .def("setBitrateKbps", &VideoEncoder::setBitrateKbps, py::arg("bitrateKbps"), DOC(dai, node, VideoEncoder, setBitrateKbps))
        .def("setKeyframeFrequency", &VideoEncoder::setKeyframeFrequency, py::arg("freq"), DOC(dai, node, VideoEncoder, setKeyframeFrequency))
        .def("setMaxOutputFrameSize", &VideoEncoder::setMaxOutputFrameSize, py::arg("maxFrameSize"), DOC(dai, node, VideoEncoder, setMaxOutputFrameSize))
        //.def("setMaxBitrate", &VideoEncoder::setMaxBitrate)
        .def("setNumBFrames", &VideoEncoder::setNumBFrames, py::arg("numBFrames"), DOC(dai, node, VideoEncoder, setNumBFrames))
        .def("setQuality", &VideoEncoder::setQuality, py::arg("quality"), DOC(dai, node, VideoEncoder, setQuality))
        .def("setLossless", &VideoEncoder::setLossless, DOC(dai, node, VideoEncoder, setLossless))
        .def("setFrameRate", &VideoEncoder::setFrameRate, py::arg("frameRate"), DOC(dai, node, VideoEncoder, setFrameRate))
        .def("getRateControlMode", &VideoEncoder::getRateControlMode, DOC(dai, node, VideoEncoder, getRateControlMode))
        .def("getProfile", &VideoEncoder::getProfile, DOC(dai, node, VideoEncoder, getProfile))
        .def("getBitrate", &VideoEncoder::getBitrate, DOC(dai, node, VideoEncoder, getBitrate))
        .def("getBitrateKbps", &VideoEncoder::getBitrateKbps, DOC(dai, node, VideoEncoder, getBitrateKbps))
        .def("getKeyframeFrequency", &VideoEncoder::getKeyframeFrequency, DOC(dai, node, VideoEncoder, getKeyframeFrequency))
        //.def("getMaxBitrate", &VideoEncoder::getMaxBitrate)
        .def("getNumBFrames", &VideoEncoder::getNumBFrames, DOC(dai, node, VideoEncoder, getNumBFrames))
        .def("getQuality", &VideoEncoder::getQuality, DOC(dai, node, VideoEncoder, getQuality))
        .def("getFrameRate", &VideoEncoder::getFrameRate, DOC(dai, node, VideoEncoder, getFrameRate))
        .def("getLossless", &VideoEncoder::getLossless, DOC(dai, node, VideoEncoder, getLossless))
        .def("getMaxOutputFrameSize", &VideoEncoder::getMaxOutputFrameSize, DOC(dai, node, VideoEncoder, getMaxOutputFrameSize));
    // ALIAS
    daiNodeModule.attr("VideoEncoder").attr("Properties") = videoEncoderProperties;
}
