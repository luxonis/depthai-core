#include "depthai/depthai.hpp"

class Display : public dai::NodeCRTP<dai::ThreadedNode, Display> {
   public:
    constexpr static const char* NAME = "Display";

   public:
    void build() {
        hostNode = true;
    }

    /**
     * Input for any ImgFrame messages to be displayed
     * Default queue is blocking with size 8
     */
    Input input{true, *this, "in", Input::Type::SReceiver, true, 8, true, {{dai::DatatypeEnum::Buffer, true}}};

    void run() override {
        while(isRunning()) {
            std::shared_ptr<dai::ImgFrame> imgFrame = input.queue.get<dai::ImgFrame>();
            if(imgFrame != nullptr) {
                cv::imshow("MyConsumer", imgFrame->getCvFrame());
                auto key = cv::waitKey(1);
                if(key == 'q') {
                    stop();
                }            }
        }
        fmt::print("Display node stopped\n");
    }
};

int main() {
	dai::Pipeline pipeline;

	auto replay = pipeline.create<dai::node::Replay>();
	auto cam = pipeline.create<dai::node::Camera>();
	auto display = pipeline.create<Display>();

	cam->setBoardSocket(dai::CameraBoardSocket::CAM_A);

	replay->setReplayVideo("video.mp4");
	replay->setReplayFile("video.mcap");

	replay->out.link(cam->mockIsp);
	cam->video.link(display->input);

	pipeline.start();

	std::this_thread::sleep_for(std::chrono::seconds(10));

	pipeline.stop();
}
