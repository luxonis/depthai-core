#pragma once
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"

namespace dai {
namespace node {

/**
 * @brief RGBD node. Combines depth and color frames into a single point cloud.
 */
class RGBD : public NodeCRTP<ThreadedHostNode, RGBD> {
   public:
    constexpr static const char* NAME = "RGBD";

    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    std::string colorInputName = "color";
    std::string depthInputName = "depth";

    /**
     * Input color frame.
     */
    Input& inColor = inputs[colorInputName];
    /**
     * Input depth frame.
     */
    Input& inDepth = inputs[depthInputName];

    /**
     * Output point cloud.
     */
    Output pcl{*this, {"pcl", DEFAULT_GROUP, {{DatatypeEnum::PointCloudData, true}}}};

    std::shared_ptr<RGBD> build();
private:
    void run() override;
    void initialize(std::vector<std::shared_ptr<ImgFrame>> frames);
    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};
    bool initialized = false;
    float fx, fy, cx, cy;
};

}  // namespace node
}  // namespace dai

