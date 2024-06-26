#include <chrono>
#include <iostream>

#include "depthai/depthai.hpp"

int main() {
    dai::Pipeline pipeline;

    auto script1 = pipeline.create<dai::node::Script>();
    script1->setScript(
        R"SCRPT(
from time import sleep

while True:
    sleep(1)
    b = Buffer(512)
    b.setData(bytes(4 * [i for i in range(0, 128)]))
    b.setTimestamp(Clock.now())
    node.io['out'].send(b)
)SCRPT");

    auto script2 = pipeline.create<dai::node::Script>();
    script2->setScript(
        R"SCRPT(
from time import sleep

while True:
    sleep(0.3)
    b = Buffer(512)
    b.setData(bytes(4 * [i for i in range(128, 256)]))
    b.setTimestamp(Clock.now())
    node.io['out'].send(b)
)SCRPT");

    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::milliseconds(100));

    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("xout");

    sync->out.link(xout->input);
    script1->outputs["out"].link(sync->inputs["s1"]);
    script2->outputs["out"].link(sync->inputs["s2"]);

    dai::Device device(pipeline);
    std::cout << "Start" << std::endl;
    auto queue = device.getOutputQueue("xout", 10, true);
    while(true) {
        auto grp = queue->get<dai::MessageGroup>();
        std::cout << "Buffer 1 timestamp: " << grp->get<dai::Buffer>("s1")->getTimestamp().time_since_epoch().count() << std::endl;
        std::cout << "Buffer 2 timestamp: " << grp->get<dai::Buffer>("s2")->getTimestamp().time_since_epoch().count() << std::endl;
        std::cout << "Time interval between messages: " << static_cast<double>(grp->getIntervalNs()) / 1e6 << "ms" << std::endl;
        std::cout << "----------" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
