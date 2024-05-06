#include <cstdio>
#include <iostream>

#include "depthai/pipeline/node/host/Record.hpp"
#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;

    {  // Create pipeline
        dai::Pipeline pipeline;

        // Define sources and outputs
        auto replay = pipeline.create<dai::node::Replay>();
        auto imu = pipeline.create<dai::node::IMU>();
        auto record = pipeline.create<dai::node::Record>();

        replay->setReplayFile("imu_recording.mcap");

        // enable ACCELEROMETER_RAW at 500 hz rate
        imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
        // enable GYROSCOPE_RAW at 400 hz rate
        imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);

        record->setRecordFile("imu_recording2");

        replay->out.link(imu->mockIn);
        imu->out.link(record->input);

        pipeline.start();

        std::this_thread::sleep_for(std::chrono::seconds(10));

        pipeline.stop();
    }

    std::ofstream out1("imu_recording1.json");
    std::ofstream out2("imu_recording2.json");
    {
        mcap::McapReader reader;
        {
            const auto res = reader.open("imu_recording.mcap");
            if(!res.ok()) {
                std::cerr << "Failed to open "
                    << "imu_recording"
                    << " for reading: " << res.message << std::endl;
                return 1;
            }
        }
        auto messageView = reader.readMessages();
        for(const auto& msg : messageView) {
            assert(msg.channel->messageEncoding == "json");
            std::string_view asString(reinterpret_cast<const char*>(msg.message.data), msg.message.dataSize);
            nlohmann::json j = nlohmann::json::parse(asString);
            out1 << j.dump(4) << std::endl;
        }
    }
    {
        mcap::McapReader reader;
        {
            const auto res = reader.open("imu_recording2.mcap");
            if(!res.ok()) {
                std::cerr << "Failed to open "
                    << "imu_recording"
                    << " for reading: " << res.message << std::endl;
                return 1;
            }
        }
        auto messageView = reader.readMessages();
        for(const auto& msg : messageView) {
            assert(msg.channel->messageEncoding == "json");
            std::string_view asString(reinterpret_cast<const char*>(msg.message.data), msg.message.dataSize);
            nlohmann::json j = nlohmann::json::parse(asString);
            out2 << j.dump(4) << std::endl;
        }
    }
    return 0;
}
