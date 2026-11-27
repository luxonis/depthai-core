#include "../../../../../../src/utility/DeblurValidation.hpp"
#include "Deblur.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utilities/Deblur.hpp"
#include "utilities/DeblurOpenCL.hpp"
#include "utilities/DeblurSync.hpp"
namespace dai {
namespace gate {
namespace {
std::shared_ptr<ImgFrame> makeImage(const ImgFrame& target, const cv::Mat& data) {
    auto result = std::make_shared<ImgFrame>();
    result->setMetadata(target);
    result->setData(std::vector<std::uint8_t>(data.data, data.data + data.total()));
    return result;
}
}  // namespace
void Deblur::run() {
    const auto& logger = ThreadedNode::pimpl->logger;
    try {
        dai::utility::validateDeblurConfig(properties);
        deblur::Synchronizer sync(properties.cacheSize, std::chrono::milliseconds(properties.maxWaitMs));
        std::unique_ptr<deblur::OpenCLFilter> gpu;
        const auto emitReady = [&]() {
            while(true) {
                auto ready = sync.take(deblur::Synchronizer::Clock::now());
                if(!ready.target) break;
                auto restored = ready.target;
                cv::Mat mask = cv::Mat::zeros(ready.target->getHeight(), ready.target->getWidth(), CV_8U);
                if(ready.flow && ready.reference) {
                    const auto field = deblur::prepare(*ready.reference, *ready.target, *ready.flow, properties);
                    if(!field.kernels.empty()) {
                        cv::Mat filtered;
                        if(properties.useOpenCL) {
                            if(!gpu) gpu = std::make_unique<deblur::OpenCLFilter>();
                            filtered = gpu->apply(field);
                        } else
                            filtered = deblur::filterCpu(field);
                        restored = makeImage(*ready.target, deblur::finish(field, filtered, properties));
                        // Ceil positive weights, so a zero mask always means exact passthrough.
                        for(int y = 0; y < mask.rows; ++y)
                            for(int x = 0; x < mask.cols; ++x)
                                mask.at<std::uint8_t>(y, x) = static_cast<std::uint8_t>(std::ceil(field.blend.at<float>(y, x) * 255));
                    }
                }
                auto event = outputBlockEvent();
                output.send(restored);
                passthroughInputImage.send(ready.target);
                if(!outputMask.getConnections().empty() || !outputMask.getQueueConnections().empty()) outputMask.send(makeImage(*ready.target, mask));
            }
        };
        while(mainLoop()) {
            {
                auto event = inputBlockEvent();
                MessageQueue::waitAny({inputImage, inputFlow}, std::chrono::milliseconds(2));
            }
            // One of each per iteration avoids starvation and bounds work under continuous input.
            if(auto message = inputFlow.tryGet()) {
                auto packet = std::dynamic_pointer_cast<MessageGroup>(message);
                if(packet)
                    sync.addFlow(std::move(packet));
                else
                    logger->warn("Deblur ignores inputFlow messages that are not MessageGroup");
            }
            if(auto message = inputImage.tryGet()) {
                auto frame = std::dynamic_pointer_cast<ImgFrame>(message);
                if(!frame) throw std::invalid_argument("Deblur inputImage requires ImgFrame messages");
                deblur::validateImage(*frame);
                // Camera frames can come from a pool smaller than our history cache.
                // Retaining them until cache eviction can exhaust that pool and
                // deadlock acquisition. Detach payload ownership before caching.
                sync.addImage(frame->clone(), deblur::Synchronizer::Clock::now());
            }
            emitReady();
        }
    } catch(const std::exception& error) {
        if(isRunning()) logger->error("Deblur stopped: {}", error.what());
        stop();
    }
}
}  // namespace gate
}  // namespace dai
