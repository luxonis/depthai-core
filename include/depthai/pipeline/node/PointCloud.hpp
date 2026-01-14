#pragma once

#include <depthai/pipeline/DeviceNode.hpp>

// shared
#include <depthai/properties/PointCloudProperties.hpp>
#include <memory>

#include "depthai/pipeline/datatype/PointCloudConfig.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/utility/Pimpl.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/HousingCoordinateSystem.hpp"
#include "depthai/common/DepthUnit.hpp"

namespace dai {
namespace node {

/**
 * @brief PointCloud node. Computes point cloud from depth frames.
 */
class PointCloud : public DeviceNodeCRTP<DeviceNode, PointCloud, PointCloudProperties>, public HostRunnable {
   public:
    constexpr static const char* NAME = "PointCloud";

   protected:
    Properties& getProperties() override;
    using DeviceNodeCRTP::DeviceNodeCRTP;

   public:
    PointCloud();
    ~PointCloud();

    /**
     * Initial config to use when computing the point cloud.
     */
    std::shared_ptr<PointCloudConfig> initialConfig = std::make_shared<PointCloudConfig>();

    /**
     * Input PointCloudConfig message with ability to modify parameters in runtime.
     * Default queue is non-blocking with size 4.
     */
    Input inputConfig{*this, {"inputConfig", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::PointCloudConfig, false}}}, DEFAULT_WAIT_FOR_MESSAGE}};

    /**
     * Input message with depth data used to create the point cloud.
     * Default queue is non-blocking with size 4.
     */
    Input inputDepth{*this, {"inputDepth", DEFAULT_GROUP, false, 4, {{{DatatypeEnum::ImgFrame, false}}}, true}};

    /**
     * Outputs PointCloudData message
     */
    Output outputPointCloud{*this, {"outputPointCloud", DEFAULT_GROUP, {{{DatatypeEnum::PointCloudData, false}}}}};

    /**
     * Passthrough depth from which the point cloud was calculated.
     * Suitable for when input queue is set to non-blocking behavior.
     */
    Output passthroughDepth{*this, {"passthroughDepth", DEFAULT_GROUP, {{{DatatypeEnum::ImgFrame, false}}}}};

    /**
     * Specify number of frames in pool.
     * @param numFramesPool How many frames should the pool have
     */
    void setNumFramesPool(int numFramesPool);

    /**
     * Specify whether to run on host or device
     * By default, the node will run on host.
     */
    void setRunOnHost(bool runOnHost);

    /**
     * Set depth unit for point cloud computation
     * @param depthUnit Depth unit (METER, CENTIMETER, MILLIMETER, INCH, FOOT, CUSTOM)
     */
    void setDepthUnit(DepthUnit depthUnit);

    /**
     * Use single-threaded CPU for processing
     */
    void useCPU();

    /**
     * Use multi-threaded CPU for processing
     */
    void useCPUMT(uint32_t numThreads = 2);

    /**
     * Use GPU for processing (needs Kompute support)
     */
    void useGPU(uint32_t device = 0);

    /**
     * Set target coordinate system to a camera socket
     * Point cloud will be transformed to this camera's coordinate system
     * @param targetCamera Target camera socket
     * @param useSpecTranslation Whether to use spec translation (default: false)
     */
    void setTargetCoordinateSystem(CameraBoardSocket targetCamera, bool useSpecTranslation = false);

    /**
     * Set target coordinate system to housing coordinate system
     * Point cloud will be transformed to this housing coordinate system
     * @param housingCS Target housing coordinate system
     * @param useSpecTranslation Whether to use spec translation (default: false)
     */
    void setTargetCoordinateSystem(HousingCoordinateSystem housingCS, bool useSpecTranslation = false);

    bool runOnHost() const override;

   private:
    class Impl;
    Pimpl<Impl> pimplPointCloud;  // Our custom Impl for point cloud computation
    
    void run() override;
    void initialize(std::shared_ptr<ImgFrame> depthFrame);
    
    bool runOnHostVar = true;
    bool initialized = false;
    
    // Coordinate system transformation settings
    enum class CoordinateSystemType {
        NONE,           // No transformation (camera coordinates)
        CAMERA_SOCKET,  // Transform to another camera
        HOUSING         // Transform to housing coordinate system
    };
    
    CoordinateSystemType coordSystemType = CoordinateSystemType::NONE;
    CameraBoardSocket targetCameraSocket = CameraBoardSocket::AUTO;
    HousingCoordinateSystem targetHousingCS = HousingCoordinateSystem::AUTO;
    bool useSpecTranslation = false;
};

}  // namespace node
}  // namespace dai
