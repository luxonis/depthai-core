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

template <typename T>
T daiDeserializeFromJS(std::vector<std::uint8_t> data) {
    T imgFrame;
    std::cout << "input bytes array length: " << data.size() << std::endl;
    std::cout << "starting deserialize" << std::endl;
    const auto rc = dai::utility::deserialize(data, imgFrame);
    std::cout << "after deserialize" << std::endl;
    if(!rc) {
        // this should never happen as exceptions are enabled !!!
        std::cout << "Deserialization failed" << std::endl;
    }
    return imgFrame;
}

using namespace emscripten;

EMSCRIPTEN_BINDINGS(depthai_js) {
    class_<dai::ImgFrame>("ImgFrame").property("width", &dai::ImgFrame::getWidth).property("height", &dai::ImgFrame::getHeight);
    value_object<dai::ImgDetection>("ImgDetection")
        .field("xmin", &dai::ImgDetection::xmin)
        .field("xmax", &dai::ImgDetection::xmax)
        .field("label", &dai::ImgDetection::label);
    class_<dai::ImgDetections>("ImgDetections").property("detections", &dai::ImgDetections::detections);
    function("deserializeImgFrame", &daiDeserializeFromJS<dai::ImgFrame>);
    function("deserializeImgDetections", &daiDeserializeFromJS<dai::ImgDetections>);
    register_vector<std::uint8_t>("Uint8Vector");
    register_vector<dai::ImgDetection>("ImgDetectionVector");
}

