#include <depthai/depthai.hpp>

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);
    auto camRgb = pipeline.create<dai::node::HostCamera>();
    auto displayDevice = pipeline.create<dai::node::Display>(std::string{"Device Display"});

    camRgb->out.link(displayDevice->input);
    auto queue = camRgb->out.getQueue();
    pipeline.start();
    while(pipeline.isRunning()) {
        auto img = queue->get<dai::ImgFrame>();
        cv::imshow("from-queue", img->getCvFrame());
    }
    pipeline.wait();
    return 0;
}