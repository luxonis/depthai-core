#pragma once
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "kompute/Kompute.hpp"

namespace dai {
namespace node {

/**
 * @brief RGBD node. Combines depth and color frames into a single point cloud.
 */
class RGBD : public NodeCRTP<ThreadedHostNode, RGBD> {
   public:
    constexpr static const char* NAME = "RGBD";

    Subnode<node::Sync> sync{*this, "sync"};
    std::shared_ptr<node::ImageAlign> align;
    InputMap& inputs = sync->inputs;

    /**
     * Input color frame.
     */
    Input inColor{*this, {"inColor", DEFAULT_GROUP, false, 0, {{DatatypeEnum::ImgFrame, true}}}};
    /**
     * Input depth frame.
     */
    Input inDepth{*this, {"inDepth", DEFAULT_GROUP, false, 0, {{DatatypeEnum::ImgFrame, true}}}};

    /**
     * Output point cloud.
     */
    Output pcl{*this, {"pcl", DEFAULT_GROUP, {{DatatypeEnum::PointCloudData, true}}}};

    std::shared_ptr<RGBD> build();
    std::shared_ptr<RGBD> build(bool autocreate, std::pair<int, int> size);
    void setOutputMeters(bool outputMeters) {
        this->outputMeters = outputMeters;
    }
    void useCPU() {
        computeMethod = ComputeMethod::CPU;
    }
    void useCpuMt() {
        computeMethod = ComputeMethod::CPU_MT;
    }
    void useGPU() {
        computeMethod = ComputeMethod::GPU;
    }
    void setGPUDevice(uint32_t deviceIndex) {
        this->deviceIndex = deviceIndex;
    }

   private:
    void computePointCloudGPU(const cv::Mat& depthMat, const cv::Mat& colorMat, std::vector<float>& xyzOut, std::vector<uint8_t>& rgbOut);
    void computePointCloudCPU(const cv::Mat& depthMat, const cv::Mat& colorMat, std::vector<Point3fRGB>& points);
    void computePointCloudCPUMT(const cv::Mat& depthMat, const cv::Mat& colorMat, std::vector<Point3fRGB>& points);
    enum class ComputeMethod { CPU, CPU_MT, GPU };
    std::string colorInputName = "inColorSync";
    std::string depthInputName = "inDepthSync";
    Input& inColorSync = inputs[colorInputName];
    Input& inDepthSync = inputs[depthInputName];
    Output colorMux{*this, {"colorMux", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, true}}}};
    Output depthPT{*this, {"depthPT", DEFAULT_GROUP, {{DatatypeEnum::ImgFrame, true}}}};
    void run() override;
    void initialize(std::vector<std::shared_ptr<ImgFrame>> frames);
    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};
    bool initialized = false;
    float fx, fy, cx, cy;
    bool outputMeters = false;
    std::shared_ptr<kp::Manager> mgr;
    ComputeMethod computeMethod = ComputeMethod::CPU;
    uint32_t deviceIndex = 0;
};

}  // namespace node
}  // namespace dai
