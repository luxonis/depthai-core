#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/utility/Serialization.hpp"
#include "depthai/pipeline/node/ToF.hpp"

namespace dai {

enum class DepthSource {
Stereo,
ToF
};

class DepthFrame : public ImgFrame {
   public:
    DepthFrame();
    DepthFrame(long fd);
    DepthFrame(size_t size);
    DepthFrame(long fd, size_t size);

    virtual ~DepthFrame() = default;

    void save(const std::string& path);
    float getMedian() ;
    float getMin() ;
    float getMax() ;
    DepthSource getSource() const;
    float getMaxDisparity();
    dai::CameraBoardSocket getLeftCamera() const;
    dai::CameraBoardSocket getRightCamera() const;
    std::shared_ptr<PointCloudData> getPointCloud();

    DepthSource source = DepthSource::Stereo;
    dai::CameraBoardSocket leftCamera = dai::CameraBoardSocket::CAM_B;
    dai::CameraBoardSocket rightCamera = dai::CameraBoardSocket::CAM_C;
    std::variant<StereoDepthConfig, ToFConfig> config;
private:
    float getScale();
    float median = 0;
    float min = 0;
    float max = 0;
    float scale = 0;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DepthFrame;
    };
    DEPTHAI_SERIALIZE(DepthFrame, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, config, leftCamera, rightCamera, source);
};

}  // namespace dai
