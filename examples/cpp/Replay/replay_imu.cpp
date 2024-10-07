#include <cstdio>
#include <iostream>

#include "depthai/pipeline/node/host/Record.hpp"
#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    {  // Create pipeline
        dai::Pipeline pipeline;

        // Define sources and outputs
        auto replay = pipeline.create<dai::node::ReplayMetadataOnly>();
        auto imu = pipeline.create<dai::node::IMU>();

        std::string filePath = "imu_recording.mcap";
        if(argc > 1) {
            filePath = argv[1];
        }

        replay->setReplayFile(filePath);

        // enable ACCELEROMETER_RAW at 500 hz rate
        imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
        // enable GYROSCOPE_RAW at 400 hz rate
        imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);

        replay->out.link(imu->mockIn);

        auto imuQ = imu->out.createOutputQueue();

        pipeline.start();

        while(true) {
            auto imuData = imuQ->get<dai::IMUData>();
            std::cout << "Packet seqNo: " << (int)imuData->getSequenceNum() << "\n";
            std::cout << "Packet ts: " << imuData->tsDevice.sec << "s " << imuData->tsDevice.nsec << "ns\n";
            for(auto& imuPacket : imuData->packets) {
                std::cout << "\tAccelerometer: \n";
                std::cout << "\t\tts: " << imuPacket.acceleroMeter.tsDevice.sec << "s " << imuPacket.acceleroMeter.tsDevice.nsec << "ns\n";
                std::cout << "\t\tseqNo: " << imuPacket.acceleroMeter.sequence << "\n";
                std::cout << "\t\tx: " << imuPacket.acceleroMeter.x << "\n";
                std::cout << "\t\ty: " << imuPacket.acceleroMeter.y << "\n";
                std::cout << "\t\tz: " << imuPacket.acceleroMeter.z << "\n";

                std::cout << "\tGyroscope: \n";
                std::cout << "\t\tts: " << imuPacket.gyroscope.tsDevice.sec << "s " << imuPacket.gyroscope.tsDevice.nsec << "ns\n";
                std::cout << "\t\tseqNo: " << imuPacket.gyroscope.sequence << "\n";
                std::cout << "\t\tx: " << imuPacket.gyroscope.x << "\n";
                std::cout << "\t\ty: " << imuPacket.gyroscope.y << "\n";
                std::cout << "\t\tz: " << imuPacket.gyroscope.z << "\n";
            }
        }

        pipeline.stop();
    }
}
