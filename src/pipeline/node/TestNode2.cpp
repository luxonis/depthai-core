#include "depthai/pipeline/node/TestNode2.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace node {

namespace {

std::shared_ptr<ImgFrame> withBlackPixelAt5x5(const std::shared_ptr<ImgFrame>& img) {
    auto modified = std::make_shared<ImgFrame>();
    modified->setMetadata(img);
    std::vector<std::uint8_t> data(img->getData().begin(), img->getData().end());

    constexpr std::uint32_t kPixelX = 5;
    constexpr std::uint32_t kPixelY = 5;
    if(img->getWidth() <= kPixelX || img->getHeight() <= kPixelY) {
        modified->setData(std::move(data));
        return modified;
    }

    const auto stride = std::max<unsigned int>(img->getStride(), img->getWidth());
    const auto bytesPerPixel = static_cast<unsigned int>(std::ceil(img->getBytesPerPixel()));
    const auto pixelOffset = static_cast<std::size_t>(kPixelY) * stride + static_cast<std::size_t>(kPixelX) * bytesPerPixel;

    for(unsigned int channel = 0; channel < bytesPerPixel && pixelOffset + channel < data.size(); channel++) {
        data[pixelOffset + channel] = 0;
    }
    modified->setData(std::move(data));
    return modified;
}

}  // namespace

void TestNode2::run() {
    while(mainLoop()) {
        std::shared_ptr<BatchItem> iterable;
        {
            auto blockEvent = inputBlockEvent();
            iterable = input.get<BatchItem>();
        }

        std::shared_ptr<ADatatype> outputPayload = iterable ? iterable->getPayload() : nullptr;
        if(auto img = iterable ? iterable->getPayload<ImgFrame>() : nullptr) {
            outputPayload = withBlackPixelAt5x5(img);
        }

        {
            auto blockEvent = outputBlockEvent();
            output.sendAsBatchItem(outputPayload, iterable->batchSize, iterable->batchIndex, iterable->itemIndex);
        }
    }
}

}  // namespace node
}  // namespace dai
