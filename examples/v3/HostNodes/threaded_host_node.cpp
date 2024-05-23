#include<iostream>
#include <chrono>
#include <thread>

#include "depthai/depthai.hpp"

class TestPassthrough : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestPassthrough> {
   public:
    Input input = dai::Node::Input{*this, {}};
    Output output = dai::Node::Output{*this, {}};

    void run() override {
        while(isRunning()) {
            auto buffer = input.get<dai::Buffer>();
            if(buffer) {
                std::cout << "The passthrough node received a buffer!" << std::endl;
                output.send(buffer);
            }
        }
    }
};

class TestSink : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestSink> {
   public:
    Input input = dai::Node::Input{*this, {}};

    void run() override {
        while(isRunning()) {
            auto buffer = input.get<dai::Buffer>();
            if(buffer) {
                std::cout << "The sink node received a buffer!" << std::endl;
            }
        }
    }
};

class TestSource : public dai::NodeCRTP<dai::node::ThreadedHostNode, TestSource> {
   public:
    Output output = dai::Node::Output{*this, {}};

    void run() override {
        while(isRunning()) {
            auto buffer = std::make_shared<dai::Buffer>();
            std::cout << "The source node is sending a buffer!" << std::endl;
            output.send(buffer);
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
};

int main() {
    dai::Pipeline pipeline(false);

    auto source = pipeline.create<TestSource>();
    auto passthrough = pipeline.create<TestPassthrough>();
    auto sink = pipeline.create<TestSink>();

    source->output.link(passthrough->input);
    passthrough->output.link(sink->input);

    pipeline.start();

    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        std::cout << "Pipeline is running..." << std::endl;
    }

    return 0;
}
