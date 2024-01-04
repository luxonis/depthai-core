#include "depthai/pipeline/node/ImageManip.hpp"
namespace dai {
namespace node {

ImageManip::ImageManip(std::unique_ptr<Properties> props) : NodeCRTP<DeviceNode, ImageManip, ImageManipProperties>(std::move(props)) {}

ImageManip::Properties& ImageManip::getProperties() {
    properties.initialConfig = initialConfig;
    return properties;
}

// Initial ImageManipConfig
void ImageManip::setCropRect(float xmin, float ymin, float xmax, float ymax) {
    initialConfig.setCropRect(xmin, ymin, xmax, ymax);
    properties.initialConfig = initialConfig;
}

void ImageManip::setCenterCrop(float ratio, float whRatio) {
    initialConfig.setCenterCrop(ratio, whRatio);
    properties.initialConfig = initialConfig;
}

void ImageManip::setResize(int w, int h) {
    initialConfig.setResize(w, h);
    properties.initialConfig = initialConfig;
}

void ImageManip::setResizeThumbnail(int w, int h, int bgRed, int bgGreen, int bgBlue) {
    initialConfig.setResizeThumbnail(w, h, bgRed, bgGreen, bgBlue);
    properties.initialConfig = initialConfig;
}

void ImageManip::setFrameType(dai::ImgFrame::Type type) {
    initialConfig.setFrameType(type);
    properties.initialConfig = initialConfig;
}

void ImageManip::setHorizontalFlip(bool flip) {
    initialConfig.setHorizontalFlip(flip);
    properties.initialConfig = initialConfig;
}

void ImageManip::setKeepAspectRatio(bool keep) {
    initialConfig.setKeepAspectRatio(keep);
    properties.initialConfig = initialConfig;
}

// Node properties configuration
void ImageManip::setWaitForConfigInput(bool wait) {
    inputConfig.setWaitForMessage(wait);
}

bool ImageManip::getWaitForConfigInput() const {
    return inputConfig.getWaitForMessage();
}

void ImageManip::setNumFramesPool(int numFramesPool) {
    properties.numFramesPool = numFramesPool;
}

void ImageManip::setMaxOutputFrameSize(int maxFrameSize) {
    properties.outputFrameSize = maxFrameSize;
}

void ImageManip::setWarpMesh(const float* meshData, int numMeshPoints, int width, int height) {
    if(numMeshPoints < width * height) {
        throw std::invalid_argument("Not enough points provided for specified width and height");
    }

    // TODO(themarpe) - optimize
    Asset asset("mesh");
    asset.alignment = 64;

    // Align stride to 16B
    constexpr auto ALIGNMENT = 16;
    size_t meshStride = ((size_t)((sizeof(Point2f) * width)) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1);
    // Specify final mesh size
    size_t meshSize = meshStride * height;

    // Create mesh data
    asset.data = std::vector<uint8_t>(meshSize);

    // Fill out mesh points with stride
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            // get location in meshData
            size_t inputMeshIndex = (i * width + j) * 2;  // 2 float values per point

            // get output offset
            size_t outputMeshOffset = (meshStride * i) + (j * sizeof(Point2f));
            auto& point = reinterpret_cast<Point2f&>(asset.data.data()[outputMeshOffset]);

            // Asign reversed mesh coordinates (HW specified)
            point.x = meshData[inputMeshIndex + 1];
            point.y = meshData[inputMeshIndex + 0];
        }
    }

    properties.meshUri = assetManager.set(asset)->getRelativeUri();
    properties.meshWidth = width;
    properties.meshHeight = height;
}

void ImageManip::setWarpMesh(const std::vector<Point2f>& meshData, int width, int height) {
    setWarpMesh(reinterpret_cast<const float*>(meshData.data()), static_cast<int>(meshData.size()), width, height);
}

void ImageManip::setWarpMesh(const std::vector<std::pair<float, float>>& meshData, int width, int height) {
    setWarpMesh(reinterpret_cast<const float*>(meshData.data()), static_cast<int>(meshData.size()), width, height);
}

}  // namespace node
}  // namespace dai
