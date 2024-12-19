#pragma once
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
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
    ~RGBD();
    Subnode<node::Sync> sync{*this, "sync"};
    InputMap& inputs = sync->inputs;

    std::string colorInputName = "inColorSync";
    std::string depthInputName = "inDepthSync";
    Input& inColor = inputs[colorInputName];
    Input& inDepth = inputs[depthInputName];

    /**
     * Output point cloud.
     */
    Output pcl{*this, {"pcl", DEFAULT_GROUP, {{DatatypeEnum::PointCloudData, true}}}};

    std::shared_ptr<RGBD> build();
    /**
     * @brief Build RGBD node with specified size
     * @param autocreate If true, will create color and depth frames if they don't exist
     * @param size Size of the frames
     */
    std::shared_ptr<RGBD> build(bool autocreate, std::pair<int, int> size);
    void setOutputMeters(bool outputMeters);
    /**
    * @brief Use single-threaded CPU for processing
    */
    void useCPU();
    /**
    * @brief Use multi-threaded CPU for processing
    */
    void useCPUMT();
    /**
    * @brief Use GPU for processing (needs to be compiled with Kompute support)
    */
    void useGPU();
    /**
    * @brief Set GPU device index
    */
    void setGPUDevice(uint32_t deviceIndex);
    /**
    * @brief Set number of threads for CPU processing
    */
    void setCPUThreadNum(uint32_t numThreads);
    /**
    * @brief Print available GPU devices
    */
    void printDevices();
   private:
    class Impl;
    Pimpl<Impl> pimpl;
    void run() override;
    void initialize(std::vector<std::shared_ptr<ImgFrame>> frames);
    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};
    bool initialized = false;
};

}  // namespace node
}  // namespace dai
