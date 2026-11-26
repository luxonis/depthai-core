#pragma once

#include <spdlog/async_logger.h>

#include <thread>

#include "depthai/pipeline/MessageQueue.hpp"
#include "depthai/pipeline/Node.hpp"

namespace dai {
namespace gate {

class FpsRegulator {
   public:
    MessageQueue inQueue;

    FpsRegulator(float inputFps, float outputFps, Node::Output* output, std::shared_ptr<spdlog::async_logger> logger);
    FpsRegulator(float inputFps, float outputFps, MessageQueue* output, std::shared_ptr<spdlog::async_logger> logger);

    void run();
    void mainLoop();
    void stop();

   private:
    float inputFps = 0;
    float outputFps = 0;
    bool isAlive{true};
    Node::Output* output{nullptr};
    MessageQueue* outputQueue{nullptr};
    std::shared_ptr<spdlog::async_logger> logger;
    std::thread mainThread;
};

}  // namespace gate
}  // namespace dai
