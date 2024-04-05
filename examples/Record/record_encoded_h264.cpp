#include <depthai/depthai.hpp>
#include "depthai/pipeline/node/Record.hpp"

#ifndef DEPTHAI_HAVE_OPENCV_SUPPORT
	#error This example needs OpenCV support, which is not available on your system
#endif

int main() {
	dai::Pipeline pipeline;
	auto cam = pipeline.create<dai::node::ColorCamera>();
	auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
	auto record = pipeline.create<dai::node::Record>();

	record->setRecordFile("/home/work/workspaces/lib/depthai-python/depthai-core/recording_h264.mp4");

	cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
	cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
	cam->setFps(30);

	videoEncoder->setProfile(dai::VideoEncoderProperties::Profile::H264_MAIN);

	cam->video.link(videoEncoder->input);
	videoEncoder->out.link(record->in);

	pipeline.start();

	std::this_thread::sleep_for(std::chrono::seconds(10));
	
	pipeline.stop();
}
