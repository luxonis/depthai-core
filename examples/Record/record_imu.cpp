
#include <cstdio>
#include <iostream>

#include "depthai/pipeline/node/host/Record.hpp"
#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto imu = pipeline.create<dai::node::IMU>();
    auto record = pipeline.create<dai::node::RecordMessage>();

    // enable ACCELEROMETER_RAW at 500 hz rate
    imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
    // enable GYROSCOPE_RAW at 400 hz rate
    imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);

    std::string recordFile = argc > 1 ? argv[1] : "imu_recording";
    record->setRecordFile(recordFile + ".mcap");

    imu->out.link(record->input);

    pipeline.start();

    std::this_thread::sleep_for(std::chrono::seconds(10));

    pipeline.stop();
}
