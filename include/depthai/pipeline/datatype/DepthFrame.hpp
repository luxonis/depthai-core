#pragma once

#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/PointCloudData.hpp"
#include "depthai/pipeline/datatype/ToFConfig.hpp"
#include "pipeline/node/ToF.hpp"

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

    void save();
    uint16_t getMedian() const;
    uint16_t getMin() const;
    uint16_t getMax() const;
    StereoDepthConfig::AlgorithmControl::DepthUnit getDepthUnit() const;
    DepthSource getSource() const;
    uint16_t getMaxDisparity() const;
    std::shared_ptr<ImgFrame> getLeft() const;
    std::shared_ptr<ImgFrame> getRight() const;
    std::shared_ptr<PointCloudData> getPointCloud() const;

private:
    uint16_t median = 0;
    uint16_t min = 0;
    uint16_t max = 0;
    StereoDepthConfig::AlgorithmControl::DepthUnit depthUnit = StereoDepthConfig::AlgorithmControl::DepthUnit::MILLIMETER;
    DepthSource source = DepthSource::Stereo;
    uint16_t maxDisparity = 0;
    std::shared_ptr<ImgFrame> left;
    std::shared_ptr<ImgFrame> right;

};

}  // namespace dai
