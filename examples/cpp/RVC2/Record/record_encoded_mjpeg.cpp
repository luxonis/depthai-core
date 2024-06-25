#include <cassert>
#include <depthai/depthai.hpp>
#include <fstream>
#include <nlohmann/json.hpp>

#include "depthai/pipeline/node/host/Record.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
    #error This example needs OpenCV support, which is not available on your system
#endif

#include "mcap/reader.hpp"

constexpr auto RECORDING_PATH = "recording";

int main() {
    {
        std::cout << "Press q to stop recording" << std::endl;
        dai::Pipeline pipeline(true);
        auto cam = pipeline.create<dai::node::ColorCamera>();
        auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
        auto record = pipeline.create<dai::node::Record>();
        auto display = pipeline.create<dai::node::Display>();

        record->setRecordFile(RECORDING_PATH);

        cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
        cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
        cam->setFps(30);

        videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);

        cam->video.link(videoEncoder->input);
        cam->video.link(display->input);
        videoEncoder->out.link(record->input);

        pipeline.run();  // Let the display node stop the pipeline
    }
    // mcap::McapReader reader;
    // {
    //     const auto res = reader.open(std::string(RECORDING_PATH) + ".mcap");
    //     if(!res.ok()) {
    //         std::cerr << "Failed to open " << RECORDING_PATH << " for reading: " << res.message << std::endl;
    //         return 1;
    //     }
    // }
    // auto messageView = reader.readMessages();
    // auto message = *messageView.begin();
    // assert(message.channel->messageEncoding == "json");
    // std::string_view asString(reinterpret_cast<const char*>(message.message.data), message.message.dataSize);
    // std::cout << "Message: " << asString << std::endl;
    // nlohmann::json j = nlohmann::json::parse(asString);
    // std::ofstream out(std::string(RECORDING_PATH) + ".json");
    // out << j.dump(4) << std::endl;

    // std::string_view asString2(reinterpret_cast<const char*>(message.schema->data.data()), message.schema->data.size());
    // std::cout << "Schema: " << asString2 << std::endl;
    return 0;
}
