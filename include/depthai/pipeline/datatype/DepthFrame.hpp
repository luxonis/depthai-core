#pragma once

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/optional.hpp"
#include "depthai/common/variant.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "depthai/utility/Serialization.hpp"

namespace dai {

class DepthFrame : public ImgFrame {
   public:
    enum class DepthSource { Stereo, ToF };
    DepthFrame();
    DepthFrame(long fd);
    DepthFrame(size_t size);
    DepthFrame(long fd, size_t size);

    virtual ~DepthFrame() = default;

    void save(const std::string& path);
    float getMedian();
    float getMin();
    float getMax();
    DepthSource getSource() const;
    std::optional<CameraBoardSocket> getLeftBoardSocket();
    std::shared_ptr<PointCloudData> getPointCloud();

    std::optional<dai::CameraBoardSocket> leftCamera;
    DepthSource source = DepthSource::Stereo;
    std::variant<StereoDepthConfig, ToFConfig> config;
    StereoDepthConfig::AlgorithmControl::DepthUnit getDepthUnit();

   private:
    StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit;
    float getScale();
    float median = 0;
    float min = 0;
    float max = 0;
    float scale = 0;
    void serialize(std::vector<std::uint8_t>& metadata, DatatypeEnum& datatype) const override {
        metadata = utility::serialize(*this);
        datatype = DatatypeEnum::DepthFrame;
    };
    DEPTHAI_SERIALIZE(DepthFrame, Buffer::ts, Buffer::tsDevice, Buffer::sequenceNum, config, source, leftCamera);
};

}  // namespace dai
