#include <depthai/depthai.hpp>

int main() {
    // Create pipeline
    dai::Pipeline pipeline(false);
    auto camRgb = pipeline.create<dai::node::HostCamera>();
    auto displayDevice = pipeline.create<dai::node::Display>(std::string{"Device Display"});

    camRgb->out.link(displayDevice->input);

    pipeline.start();
    pipeline.wait();
    return 0;
}