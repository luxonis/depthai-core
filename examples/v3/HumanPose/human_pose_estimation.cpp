// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

class writeFPS : public dai::NodeCRTP<dai::node::HostNode, writeFPS> {
   private:
    std::chrono::steady_clock::time_point startTime;
    float fps = 0;
    int frames = 0;

   public:
    Input& input = inputs["in"];

    std::shared_ptr<writeFPS> build(Output& out) {
        startTime = std::chrono::steady_clock::now();
        frames = 0;
        fps = 0;

        out.link(input);
        return std::static_pointer_cast<writeFPS>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        frames++;
        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime).count();
        if(elapsed > 1000) {
            fps = frames * 1000.0f / elapsed;
            frames = 0;
            startTime = currentTime;
        }

        auto inValue = in->get<dai::ImgFrame>("in");
        auto inFrame = inValue->getCvFrame();
        cv::putText(inFrame, "FPS: " + std::to_string(fps), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(inFrame, dai::ImgFrame::Type::BGR888p);
        return frame;
    }
};

class Resize : public dai::NodeCRTP<dai::node::HostNode, Resize> {
   public:
    Input& input = inputs["in"];
    int windowWidth = 0;
    int windowHeight = 0;

    std::shared_ptr<Resize> build(Output& out, int width = 640, int height = 480) {
        windowWidth = width;
        windowHeight = height;
        out.link(input);
        return std::static_pointer_cast<Resize>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto inValue = in->get<dai::ImgFrame>("in");
        auto inFrame = inValue->getCvFrame();
        cv::resize(inFrame, inFrame, cv::Size(windowWidth, windowHeight));

        cv::Mat displayFrame(windowHeight, windowWidth, CV_8UC3);
        inFrame.copyTo(displayFrame);

        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(displayFrame, dai::ImgFrame::Type::BGR888p);
        return frame;
    }
};

class AddRedTint : public dai::NodeCRTP<dai::node::HostNode, AddRedTint> {
   public:
    Input& input = inputs["in"];

    std::shared_ptr<AddRedTint> build(Output& out) {
        out.link(input);
        return std::static_pointer_cast<AddRedTint>(this->shared_from_this());
    }

    std::shared_ptr<dai::Buffer> processGroup(std::shared_ptr<dai::MessageGroup> in) override {
        auto inValue = in->get<dai::ImgFrame>("in");
        auto inFrame = inValue->getCvFrame();
        cv::Mat redTint = cv::Mat::zeros(inFrame.size(), inFrame.type());
        redTint = cv::Scalar(0, 0, 255);
        cv::addWeighted(inFrame, 0.5, redTint, 0.5, 0, inFrame);

        auto frame = std::make_shared<dai::ImgFrame>();
        frame->setCvFrame(inFrame, dai::ImgFrame::Type::BGR888p);
        return frame;
    }
};

class Display : public dai::NodeCRTP<dai::node::HostNode, Display> {
   public:
    Input& input = inputs["in"];

    std::shared_ptr<Display> build(Output& out) {
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




int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(37);
    camRgb->setVideoSize(1920, 1080);

    // Create a node that will resize the frame
    auto resize = pipeline.create<Resize>()->build(camRgb->video);
    auto addRedTint = pipeline.create<AddRedTint>()->build(resize->out);
    auto writeFps = pipeline.create<writeFPS>()->build(addRedTint->out);
    auto display = pipeline.create<Display>()->build(writeFps->out);

    pipeline.start();
    pipeline.wait();
    return 0;


}
