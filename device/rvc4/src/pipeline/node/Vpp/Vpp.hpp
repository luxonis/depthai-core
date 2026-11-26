#pragma once

#include <atomic>
#include <memory>
#include <thread>
#include <vector>

// libraries
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "depthai/properties/VppProperties.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"

// shared
#include "VppImpl.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/VppConfig.hpp"
#include "depthai/pipeline/node/Vpp.hpp"

namespace dai {
namespace gate {

class VppTestable;
class Vpp : public ::dai::node::Vpp {
   private:
    std::shared_ptr<spdlog::async_logger>& logger = ThreadedNode::pimpl->logger;
    std::unique_ptr<VppImpl> vppImpl;

    bool loadData(cv::Mat& leftImg,
                  cv::Mat& rightImg,
                  cv::Mat& disparity,
                  cv::Mat& confidence,
                  std::shared_ptr<dai::ImgFrame>& leftOutFrame,
                  std::shared_ptr<dai::ImgFrame>& rightOutFrame);

    cv::Mat buildDisparityPrior(const cv::Mat& depth, float focalLength, float baselineMm);

    void updateDepthToDisparityLut(double scale);

    std::vector<float> depthToDisparityLut;
    double cachedDepthToDisparityScale = 0.0;

    float resolveBaselineMm(const std::shared_ptr<dai::ImgFrame>& leftIn, const std::shared_ptr<dai::ImgFrame>& rightIn);

    float baselineMm = 0.0f;

    static cv::Mat convertUintMatToFloat(const cv::Mat& matUint);

    static void copyFrameToGray(cv::Mat& dst, const cv::Mat& src);

    static cv::Mat prepareGrayOutputFrame(std::shared_ptr<dai::ImgFrame>& outputFrame, const std::shared_ptr<dai::ImgFrame>& inputFrame);

    void sendData(const std::shared_ptr<dai::ImgFrame>& leftOutFrame, const std::shared_ptr<dai::ImgFrame>& rightOutFrame);

    virtual std::shared_ptr<MessageGroup> getMessageGroup();

    virtual void sendLeft(const std::shared_ptr<ImgFrame>& msg) {
        leftOut.send(msg);
    }

    virtual void sendRight(const std::shared_ptr<ImgFrame>& msg) {
        rightOut.send(msg);
    }

   public:
    constexpr static const char* NAME = node::Vpp::NAME;

    Vpp() : Vpp(std::make_unique<dai::node::Vpp::Properties>()) {}

    Vpp(std::unique_ptr<VppProperties> props) : dai::node::Vpp(std::move(props)) {}

    bool runOnHost() const override {
        return true;
    }

    void run() override;

    int maxFPS = 90;

    friend class VppTestable;
};

}  // namespace gate
}  // namespace dai
