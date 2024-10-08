#include "depthai/pipeline/node/host/Display.hpp"

#include <chrono>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_map>

#include "depthai/pipeline/Pipeline.hpp"
#include "pipeline/datatype/MessageGroup.hpp"
namespace dai {
namespace node {

// Create a window of 30 frames and calculate the FPS
class FPSCounter {
   public:
    void update() {
        if(frames.size() >= 30) {
            frames.pop_front();
        }
        frames.push_back(std::chrono::steady_clock::now());
    }

    float getFPS() {
        if(frames.size() < 2) {
            return 0;
        }
        auto first = frames.front();
        auto last = frames.back();
        auto duration = last - first;
        return frames.size() / std::chrono::duration_cast<std::chrono::duration<float>>(duration).count();
    }

   private:
    std::deque<std::chrono::steady_clock::time_point> frames;
};

Display::Display(std::string name) : name(std::move(name)) {}

void Display::run() {
    std::unordered_map<std::string, FPSCounter> fpsCounters;
    fpsCounters["default"] = FPSCounter();
    while(isRunning()) {
        auto msg = input.get<dai::Buffer>();
        auto imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(msg);
        auto msgGroup = std::dynamic_pointer_cast<dai::MessageGroup>(msg);
        if(imgFrame != nullptr) {
            fpsCounters["default"].update();
            auto fps = fpsCounters["default"].getFPS();
            using namespace std::chrono;
            auto latencyMs = duration_cast<milliseconds>(steady_clock::now() - imgFrame->getTimestamp());
            auto frame = imgFrame->getCvFrame();
            cv::putText(frame, fmt::format("FPS: {:.2f}", fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame, fmt::format("Latency: {}ms", latencyMs.count()), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
            cv::imshow(name, frame);
            auto key = cv::waitKey(1);
            if(key == 'q') {
                // Get the parent pipeline and stop it
                // TODO(Morato) - add a convience stop method directly to the pipeline
                auto parentPipeline = getParentPipeline();
                parentPipeline.stop();
            }
        } else if(msgGroup != nullptr) {
            for(const auto& [k, v] : *msgGroup) {
                auto imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(v);
                if(imgFrame != nullptr) {
                    fpsCounters[k].update();
                    auto fps = fpsCounters[k].getFPS();
                    using namespace std::chrono;
                    auto latencyMs = duration_cast<milliseconds>(steady_clock::now() - imgFrame->getTimestamp());
                    auto frame = imgFrame->getCvFrame();
                    cv::putText(frame, fmt::format("FPS: {:.2f}", fps), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                    cv::putText(
                        frame, fmt::format("Latency: {}ms", latencyMs.count()), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                    cv::imshow(name + " - " + k, frame);
                }
            }
            auto key = cv::waitKey(1);
            if(key == 'q') {
                // Get the parent pipeline and stop it
                // TODO(Morato) - add a convience stop method directly to the pipeline
                auto parentPipeline = getParentPipeline();
                parentPipeline.stop();
            }
        }
    }
    fmt::print("Display node stopped\n");
}
}  // namespace node
}  // namespace dai
