#pragma once
#include "depthai/pipeline/Subnode.hpp"
#include "depthai/pipeline/ThreadedHostNode.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/Sync.hpp"
#include "depthai/utility/Pimpl.hpp"

// depth map source nodes
#include "depthai/pipeline/node/NeuralDepth.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/ToF.hpp"

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
    /**
     * Output RGBD frames.
     */
    Output rgbd{*this, {"rgbd", DEFAULT_GROUP, {{DatatypeEnum::RGBDData, true}}}};

    std::shared_ptr<RGBD> build();

    /**
     * @brief Build RGBD node with specified size. Note that this API is global and if used autocreated cameras can't be reused.
     * @param autocreate If true, will create color and depth nodes if they don't exist.
     * @param frameSize Size of the frames
     */
    std::shared_ptr<RGBD> build(bool autocreate,
                                StereoDepth::PresetMode mode = StereoDepth::PresetMode::DEFAULT,
                                std::pair<int, int> frameSize = std::make_pair(640, 400),
                                std::optional<float> fps = std::nullopt);

    /**
     * @brief Build RGBD node with camera and StereoDepth node
     * @param camera Camera node to use for color frames
     * @param stereo StereoDepth node to use for depth frames
     * @param frameSize Size of the frames
     * @param fps FPS of the frames
     */
    std::shared_ptr<RGBD> build(const std::shared_ptr<Camera>& camera,
                                const std::shared_ptr<StereoDepth>& stereo,
                                std::pair<int, int> frameSize = std::make_pair(640, 400),
                                std::optional<float> fps = std::nullopt);

    /**
     * @brief Build RGBD node with camera and NeuralDepth node
     * @param camera Camera node to use for color frames
     * @param neuralDepth NeuralDepth node to use for depth frames
     * @param frameSize Size of the frames
     * @param fps FPS of the frames
     */
    std::shared_ptr<RGBD> build(const std::shared_ptr<Camera>& camera,
                                const std::shared_ptr<NeuralDepth>& neuralDepth,
                                std::pair<int, int> frameSize = std::make_pair(640, 400),
                                std::optional<float> fps = std::nullopt);

    /**
     * @brief Build RGBD node with camera and ToF node
     * @param camera Camera node to use for color frames
     * @param tof ToF node to use for depth frames
     * @param frameSize Size of the frames
     * @param fps FPS of the frames
     */
    std::shared_ptr<RGBD> build(const std::shared_ptr<Camera>& camera,
                                const std::shared_ptr<ToF>& tof,
                                std::pair<int, int> frameSize = std::make_pair(640, 400),
                                std::optional<float> fps = std::nullopt);

    void setDepthUnit(StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit);
    /**
     * @brief Use single-threaded CPU for processing
     */
    void useCPU();
    /**
     * @brief Use multi-threaded CPU for processing
     * @param numThreads Number of threads to use
     */
    void useCPUMT(uint32_t numThreads = 2);
    /**
     * @brief Use GPU for processing (needs to be compiled with Kompute support)
     * @param device GPU device index
     */
    void useGPU(uint32_t device = 0);
    /**
     * @brief Print available GPU devices
     */
    void printDevices();
    void buildInternal() override;

   private:
    class Impl;
    Pimpl<Impl> pimpl;
    void run() override;
    void initialize(std::shared_ptr<MessageGroup> frames);

    // Helpers, same API, different depth map source nodes
    void alignDepth(const std::shared_ptr<StereoDepth>& stereo, const std::shared_ptr<Camera>& camera, std::pair<int, int> frameSize, std::optional<float> fps);
    void alignDepth(const std::shared_ptr<NeuralDepth>& neuralDepth,
                    const std::shared_ptr<Camera>& camera,
                    std::pair<int, int> frameSize,
                    std::optional<float> fps);
    void alignDepth(const std::shared_ptr<ToF>& tof, const std::shared_ptr<Camera>& camera, std::pair<int, int> frameSize, std::optional<float> fps);

    Input inSync{*this, {"inSync", DEFAULT_GROUP, false, 0, {{DatatypeEnum::MessageGroup, true}}}};
    bool initialized = false;
};

}  // namespace node
}  // namespace dai
