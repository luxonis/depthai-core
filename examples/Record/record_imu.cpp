#include <depthai/depthai.hpp>
#include "depthai/pipeline/node/MonoCamera.hpp"
#include "depthai/pipeline/node/host/Record.hpp"
#include "depthai/properties/MonoCameraProperties.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
	#error This example needs OpenCV support, which is not available on your system
#endif

constexpr auto RECORDING_PATH = "/home/work/workspaces/lib/depthai-python/depthai-core/recording_imu";

int main() {
	{
		dai::Pipeline pipeline;
		auto imu = pipeline.create<dai::node::IMU>();
		auto record = pipeline.create<dai::node::Record>();

		record->setRecordFile(RECORDING_PATH);

		imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
		// enable GYROSCOPE_RAW at 400 hz rate
		imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);

		imu->out.link(record->in);

		pipeline.start();

		std::this_thread::sleep_for(std::chrono::seconds(10));

		pipeline.stop();
	}

    mcap::McapReader reader;
    {
        const auto res = reader.open(std::string(RECORDING_PATH) + ".mcap");
        if(!res.ok()) {
            std::cerr << "Failed to open " << RECORDING_PATH << " for reading: " << res.message << std::endl;
            return 1;
        }
    }
	auto messageView = reader.readMessages();
	auto message = *messageView.begin();
	assert(message.channel->messageEncoding == "json");
	std::string_view asString(reinterpret_cast<const char*>(message.message.data),
                              message.message.dataSize);
	std::cout << "Message: " << asString << std::endl;
	nlohmann::json j = nlohmann::json::parse(asString);
	std::ofstream out(std::string(RECORDING_PATH) + ".json");
	out << j.dump(4) << std::endl;

	std::string_view asString2(reinterpret_cast<const char*>(message.schema->data.data()),
                              message.schema->data.size());
	std::cout << "Schema: " << asString2 << std::endl;
	return 0;
}
