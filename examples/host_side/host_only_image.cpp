#include <depthai/depthai.hpp>
#include <opencv2/highgui.hpp>
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/ImageManip.hpp"

class TestSource : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestSource> {
   public:
    Output output = dai::Node::Output{*this, {}};

    void run() override {
        int64_t seqNum = 0;

        // Generate a cv::Mat frame with a gradient
        cv::Mat frame(768, 1920, CV_8UC3);
        for(int i = 0; i < frame.rows; i++) {
            for(int j = 0; j < frame.cols; j++) {
                frame.at<cv::Vec3b>(i, j) = cv::Vec3b(i % 256, j % 256, (i + j) % 256);
            }
        }
        if(frame.empty()) {
            throw std::runtime_error("Couldn't capture frame");
        }

        cv::Mat frameResized;
        cv::resize(frame, frameResized, cv::Size(1920, 768));

        while(isRunning()) {
            auto imgFrame = std::make_shared<dai::ImgFrame>();
            imgFrame->setFrame(frameResized);
            imgFrame->setSequenceNum(seqNum++);
            imgFrame->setType(dai::ImgFrame::Type::NV12);
            imgFrame->setWidth(frameResized.cols);
            imgFrame->setHeight(frameResized.rows);
            imgFrame->setTimestamp(std::chrono::steady_clock::now());
            output.send(imgFrame);
        }
    }
};

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);
    auto camRgb = pipeline.create<TestSource>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    camRgb->output.link(manip->inputImage);
    manip->initialConfig.setResize(1920, 768);
    manip->setMaxOutputFrameSize(1920 * 768 * 3);
    auto queue = manip->out.createOutputQueue();
    pipeline.start();

    std::cout << "Pipeline running" << std::endl;
    std::size_t framesRecieved = 0;
    std::size_t totalTime = 0;
    while(pipeline.isRunning()) {
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        auto img = queue->get<dai::ImgFrame>();
        auto cvFrame = img->getCvFrame();

	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	++framesRecieved;
	totalTime += std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();

	if (framesRecieved % 1024 == 0) {
	    std::cout << "Mean time: " << totalTime / framesRecieved << "µs" << std::endl;
	}
    }

    pipeline.wait();
    return 0;
}
