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

inline int readIntLE(std::uint8_t* data) {
    return data[0] + data[1] * 256 + data[2] * 256 * 256 + data[3] * 256 * 256 * 256;
}

template <typename T>
std::shared_ptr<T> daiDeserializeFromJS(std::vector<std::uint8_t> data) {
    auto imgFrame = std::make_shared<T>();
    const auto rc = dai::utility::deserialize(data, *imgFrame);
    if(!rc) {
        // this should never happen as exceptions are enabled !!!
        std::cout << "Deserialization failed" << std::endl;
    }
    return imgFrame;
}

struct MessageHeader {
    dai::DatatypeEnum objectType;
    int serializedObjectSize;
};

MessageHeader daiGetMessageType(std::vector<std::uint8_t> data) {
    if(data.size() < 8) {
        throw std::invalid_argument("Message header data should be at least 8 bytes long.");
    }
    MessageHeader header;
    header.serializedObjectSize = readIntLE(data.data() + data.size() - 4);
    header.objectType = static_cast<dai::DatatypeEnum>(readIntLE(data.data() + data.size() - 8));
    return header;
}

using namespace emscripten;

EMSCRIPTEN_BINDINGS(depthai_js) {
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
    value_object<MessageHeader>("MessageHeader")
        .field("objectType", &MessageHeader::objectType)
        .field("serializedObjectSize", &MessageHeader::serializedObjectSize);
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
