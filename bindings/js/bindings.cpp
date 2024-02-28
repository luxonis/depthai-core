#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

// include depthai library
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/DatatypeEnum.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>

EMSCRIPTEN_DECLARE_VAL_TYPE(Uint8Array);

template <typename T>
std::shared_ptr<T> daiDeserializeFromJS(std::vector<std::uint8_t> data) {
    // T daiDeserializeFromJS(Uint8Array data) {
    streamPacketDesc_t packet;
    packet.data = data.data();
    packet.length = data.size();
    const auto res = dai::StreamMessageParser::parseMessage(&packet);
    const auto typedRes = std::dynamic_pointer_cast<T>(res);
    if(typedRes == nullptr) {
        throw std::invalid_argument(
            "Deserialization error. Passed in message data is for message of other type. "
            "Please use getMessageType() first and call the correct decoding function for the returned type");
    }
    return typedRes;
}

dai::DatatypeEnum daiGetMessageType(std::vector<std::uint8_t> data) {
    streamPacketDesc_t packet;
    packet.data = data.data();
    packet.length = data.size();
    dai::DatatypeEnum objectType;
    size_t serializedObjectSize;
    size_t bufferLength;
    std::tie(objectType, serializedObjectSize, bufferLength) = dai::StreamMessageParser::parseHeader(&packet);
    return objectType;
}

using namespace emscripten;

EMSCRIPTEN_BINDINGS(depthai_js) {
    // emscripten::val types using EMSCRIPTEN_DECLARE_VAL_TYPE() macro
    register_type<Uint8Array>("Uint8Array");

    // vector types
    register_vector<std::uint8_t>("Uint8Vector");
    register_vector<dai::ImgDetection>("ImgDetectionVector");

    // enums
    enum_<dai::DatatypeEnum>("DatatypeEnum")
        .value("ADatatype", dai::DatatypeEnum::ADatatype)
        .value("Buffer", dai::DatatypeEnum::Buffer)
        .value("ImgFrame", dai::DatatypeEnum::ImgFrame)
        .value("EncodedFrame", dai::DatatypeEnum::EncodedFrame)
        .value("NNData", dai::DatatypeEnum::NNData)
        .value("ImageManipConfig", dai::DatatypeEnum::ImageManipConfig)
        .value("CameraControl", dai::DatatypeEnum::CameraControl)
        .value("ImgDetections", dai::DatatypeEnum::ImgDetections)
        .value("SpatialImgDetections", dai::DatatypeEnum::SpatialImgDetections)
        .value("SystemInformation", dai::DatatypeEnum::SystemInformation)
        .value("SystemInformationS3", dai::DatatypeEnum::SystemInformationS3)
        .value("SpatialLocationCalculatorConfig", dai::DatatypeEnum::SpatialLocationCalculatorConfig)
        .value("SpatialLocationCalculatorData", dai::DatatypeEnum::SpatialLocationCalculatorData)
        .value("EdgeDetectorConfig", dai::DatatypeEnum::EdgeDetectorConfig)
        .value("AprilTagConfig", dai::DatatypeEnum::AprilTagConfig)
        .value("AprilTags", dai::DatatypeEnum::AprilTags)
        .value("Tracklets", dai::DatatypeEnum::Tracklets)
        .value("IMUData", dai::DatatypeEnum::IMUData)
        .value("StereoDepthConfig", dai::DatatypeEnum::StereoDepthConfig)
        .value("FeatureTrackerConfig", dai::DatatypeEnum::FeatureTrackerConfig)
        .value("ToFConfig", dai::DatatypeEnum::ToFConfig)
        .value("TrackedFeatures", dai::DatatypeEnum::TrackedFeatures)
        .value("BenchmarkReport", dai::DatatypeEnum::BenchmarkReport)
        .value("MessageGroup", dai::DatatypeEnum::MessageGroup);

    // structs
    value_object<dai::ImgDetection>("ImgDetection")
        .field("xmin", &dai::ImgDetection::xmin)
        .field("xmax", &dai::ImgDetection::xmax)
        .field("label", &dai::ImgDetection::label);

    // classes
    class_<dai::ImgFrame>("ImgFrame")
        .smart_ptr_constructor("ImgFrame", &std::make_shared<dai::ImgFrame>)
        .property("width", &dai::ImgFrame::getWidth)
        .property("height", &dai::ImgFrame::getHeight);
    class_<dai::ImgDetections>("ImgDetections")
        .smart_ptr_constructor("ImgDetections", &std::make_shared<dai::ImgDetections>)
        .property("detections", &dai::ImgDetections::detections);

    // module global functions
    function("deserializeImgFrame", &daiDeserializeFromJS<dai::ImgFrame>);
    function("deserializeImgDetections", &daiDeserializeFromJS<dai::ImgDetections>);
    function("getMessageType", &daiGetMessageType);
}

