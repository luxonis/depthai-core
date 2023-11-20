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

    auto demux = pipeline.create<dai::node::MessageDemux>();

    auto xout1 = pipeline.create<dai::node::XLinkOut>();
    xout1->setStreamName("xout1");
    auto xout2 = pipeline.create<dai::node::XLinkOut>();
    xout2->setStreamName("xout2");

    script1->outputs["out"].link(sync->inputs["s1"]);
    script2->outputs["out"].link(sync->inputs["s2"]);
    sync->out.link(demux->input);
    demux->outputs["s1"].link(xout1->input);
    demux->outputs["s2"].link(xout2->input);

    dai::Device device(pipeline);
    std::cout << "Start" << std::endl;
    auto queue1 = device.getOutputQueue("xout1", 10, true);
    auto queue2 = device.getOutputQueue("xout2", 10, true);
    while(true) {
        auto bufS1 = queue1->get<dai::Buffer>();
        auto bufS2 = queue2->get<dai::Buffer>();
        std::cout << "Buffer 1 timestamp: " << bufS1->getTimestamp().time_since_epoch().count() << std::endl;
        std::cout << "Buffer 2 timestamp: " << bufS2->getTimestamp().time_since_epoch().count() << std::endl;
        std::cout << "----------" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}
