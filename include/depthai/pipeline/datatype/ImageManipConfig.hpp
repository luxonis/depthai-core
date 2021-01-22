#pragma once

#include <unordered_map>
#include <vector>

#include "depthai-shared/datatype/RawImageManipConfig.hpp"
#include "depthai/pipeline/datatype/Buffer.hpp"

namespace dai {

// protected inheritance, so serialize isn't visible to users
class ImageManipConfig : public Buffer {
    std::shared_ptr<RawBuffer> serialize() const override;
    RawImageManipConfig& cfg;

   public:
    ImageManipConfig();
    explicit ImageManipConfig(std::shared_ptr<RawImageManipConfig> ptr);
    virtual ~ImageManipConfig() = default;

    // Functions to set properties
    void setCropRect(float xmin, float ymin, float xmax, float ymax);
    void setCropRotatedRect(RawImageManipConfig::RotatedRect rr, bool normalizedCoords = true);
    void setCropQuadrilateral(std::vector<RawImageManipConfig::Point2f> pt, RawImageManipConfig::Size2f outSize, bool normalizedCoords = true);
    void setCenterCrop(float ratio, float whRatio = 1.0f);
    void setResize(int w, int h);
    void setResizeThumbnail(int w, int h, int bgRed = 0, int bgGreen = 0, int bgBlue = 0);
    void setFrameType(dai::RawImgFrame::Type name);
    void setHorizontalFlip(bool flip);
    void setReusePreviousImage(bool reuse);

    // Functions to retrieve properties
    float getCropXMin() const;
    float getCropYMin() const;
    float getCropXMax() const;
    float getCropYMax() const;

    int getResizeWidth() const;
    int getResizeHeight() const;
    bool isResizeThumbnail() const;
};

}  // namespace dai
