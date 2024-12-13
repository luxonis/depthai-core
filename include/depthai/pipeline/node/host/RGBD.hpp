#pragma once
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/node/ImageAlign.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

namespace dai {
namespace node {

/**
 * @brief RGBD node. Combines depth and color frames into a single point cloud.
 */
class RGBD : public NodeCRTP<ThreadedHostNode, RGBD> {
   public:
    constexpr static const char* NAME = "RGBD";

    RGBD();
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
    void setOutputMeters(bool outputMeters);
    void useCPU();
    void useCpuMt();
    void useGPU();
    void setGPUDevice(uint32_t deviceIndex);
    void printDevices();
   private:
    class Impl;
    Pimpl<Impl> pimpl;
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
};

}  // namespace node
}  // namespace dai
