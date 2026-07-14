#include "depthai/pipeline/node/TestNode.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <random>

namespace dai {
namespace node {

TestNode::TestNode(std::unique_ptr<Properties> props) : DeviceNodeCRTP<DeviceNode, TestNode, TestNodeProperties>(std::move(props)) {}

void TestNode::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool TestNode::runOnHost() const {
    return runOnHostVar;
}

namespace {

bool shouldSkipThirdItem() {
    static thread_local std::mt19937 rng(std::random_device{}());
    static thread_local std::bernoulli_distribution skipDistribution(0.5);
    return skipDistribution(rng);
}

std::shared_ptr<ImgFrame> cropTopLeft10x10(const std::shared_ptr<ImgFrame>& img) {
    constexpr std::uint32_t targetSize = 10;

    const auto cropWidth = std::min<std::uint32_t>(targetSize, img->getWidth());
    const auto cropHeight = std::min<std::uint32_t>(targetSize, img->getHeight());
    const auto sourceData = img->getData();
    const auto sourceStride = std::max<unsigned int>(img->getStride(), img->getWidth());
    const auto rowBytes = std::min<unsigned int>(sourceStride, static_cast<unsigned int>(std::ceil(cropWidth * img->getBytesPerPixel())));
    const bool semiplanar = img->getType() == ImgFrame::Type::NV12 || img->getType() == ImgFrame::Type::NV21;
    const auto firstPlaneSize = rowBytes * cropHeight;

    auto cropped = std::make_shared<ImgFrame>();
    cropped->setMetadata(img);
    cropped->setSize(cropWidth, cropHeight);
    cropped->setSourceSize(cropWidth, cropHeight);
    cropped->setStride(rowBytes);
    cropped->fb.p1Offset = 0;
    cropped->fb.p2Offset = semiplanar ? firstPlaneSize : 0;
    cropped->fb.p3Offset = semiplanar ? firstPlaneSize + rowBytes * (cropHeight / 2) : 0;

    std::vector<std::uint8_t> croppedData(cropped->fb.p3Offset ? cropped->fb.p3Offset : firstPlaneSize);
    auto copyRows = [&](std::size_t sourceOffset, std::size_t targetOffset, std::uint32_t rows) {
        for(std::uint32_t row = 0; row < rows; row++) {
            const auto sourceRow = sourceOffset + row * sourceStride;
            const auto targetRow = targetOffset + row * rowBytes;
            if(sourceRow + rowBytes > sourceData.size()) break;
            std::memcpy(croppedData.data() + targetRow, sourceData.data() + sourceRow, rowBytes);
        }
    };
    copyRows(img->fb.p1Offset, cropped->fb.p1Offset, cropHeight);
    if(semiplanar) copyRows(img->fb.p2Offset, cropped->fb.p2Offset, cropHeight / 2);
    cropped->setData(std::move(croppedData));

    return cropped;
}

}  // namespace

void TestNode::run() {
    std::uint32_t batchIndex = 0;
    while(mainLoop()) {
        std::shared_ptr<ImgFrame> img;
        {
            auto blockEvent = inputBlockEvent();
            img = input.get<ImgFrame>();
        }

        {
            auto blockEvent = outputBlockEvent();
            constexpr std::uint32_t kBatchSize = 10;
            const bool skipThirdItem = shouldSkipThirdItem();
            for(std::uint32_t i = 0; i < 10; i++) {
                if(skipThirdItem && i == 3) {
                    continue;
                }
                output.sendAsBatchItem(cropTopLeft10x10(img), kBatchSize, batchIndex, i);
            }
            batchIndex++;
        }
    }
}

}  // namespace node
}  // namespace dai
