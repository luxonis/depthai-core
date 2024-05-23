#include <memory>
#include "depthai/depthai.hpp"

class Display : public dai::NodeCRTP<dai::node::HostNode, Display> {
   public:
    Input& input = inputs["in"];

    std::shared_ptr<Display> build(Output &out) {
        out.link(input);
        return std::static_pointer_cast<Display>(this->shared_from_this());
    }
    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto frame = in->get<dai::ImgFrame>("in");
        cv::Mat frameCv = frame->getCvFrame();
        cv::imshow("Display", frameCv);
        auto key = cv::waitKey(1);
        if(key == 'q') {
            stopPipeline();
        }
        return nullptr;
    }
};


class StreamMerger : public dai::NodeCRTP<dai::node::HostNode, StreamMerger> {
   public:
    Input& inputRgb = inputs["rgb"];
    Input& inputMono = inputs["mono"];

    std::shared_ptr<StreamMerger> build(Output &outRgb, Output &outMono) {
        outRgb.link(inputRgb);
        outMono.link(inputMono);
        return std::static_pointer_cast<StreamMerger>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto mono = in->get<dai::ImgFrame>("mono"); // Matches the name passed in creating the input
        auto rgb = in->get<dai::ImgFrame>("rgb"); // Matches the name passed in creating the input
        // Show the frames side by side
        auto monoFrame = mono->getCvFrame();
        auto rgbFrame = rgb->getCvFrame();
        if(monoFrame.channels() == 1) {
            cv::Mat tempFrame;
            cv::cvtColor(monoFrame, tempFrame, cv::COLOR_GRAY2BGR);
            monoFrame = tempFrame;
        }
        // Resize both to 640x640
        cv::resize(monoFrame, monoFrame, cv::Size(640, 640));
        cv::resize(rgbFrame, rgbFrame, cv::Size(640, 640));
        cv::Mat displayFrame(640, 640 * 2, CV_8UC3);
        monoFrame.copyTo(displayFrame(cv::Rect(0, 0, 640, 640)));
        rgbFrame.copyTo(displayFrame(cv::Rect(640, 0, 640, 640)));
        // Create an ImgFrame from the displayFrame
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(displayFrame, dai::ImgFrame::Type::BGR888i);
        return frame;
    }
};

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setVideoSize(640, 480);

    auto camMono = pipeline.create<dai::node::MonoCamera>();
    camMono->setBoardSocket(dai::CameraBoardSocket::CAM_B);

    auto streamMerger = pipeline.create<StreamMerger>()->build(camRgb->video, camMono->out);
    auto display = pipeline.create<Display>()->build(streamMerger->out);

    pipeline.start();
    pipeline.wait();
    return 0;
}