#include "depthai/beta/node/ImageOutputParser.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "depthai/common/ModelType.hpp"
#include "depthai/nn_archive/NNArchiveVersionedConfig.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "utility/ErrorMacros.hpp"

namespace dai {
namespace beta {

ImageOutputParserProperties::~ImageOutputParserProperties() = default;

namespace node {

namespace {

/**
 * Tensor values in the tensor's stored order together with the tensor shape.
 */
struct TensorData {
    /// Tensor values in row-major order over dims.
    std::vector<float> values;
    /// Tensor shape in the tensor's stored order.
    std::vector<std::size_t> dims;
};

/**
 * Retrieve a tensor from NNData as dequantized FP32 values in the tensor's stored order, matching
 * the source parser's output.getTensor(name, dequantize=True).
 */
TensorData getTensorData(dai::NNData& nnData, const std::string& tensorName) {
#ifdef DEPTHAI_XTENSOR_SUPPORT
    xt::xarray<float> tensor = nnData.getTensor<float>(tensorName, true);
    TensorData result;
    result.dims.assign(tensor.shape().begin(), tensor.shape().end());
    result.values.assign(tensor.begin(), tensor.end());
    return result;
#else
    (void)nnData;
    (void)tensorName;
    throw std::runtime_error("ImageOutputParser requires xtensor support to retrieve the output image tensor, but xtensor support is not available.");
#endif
}

/**
 * 8-bit image values together with the image shape in CHW or HWC orientation, as produced by the
 * model.
 */
struct Uint8Image {
    /// Image values in row-major order over dims.
    std::vector<std::uint8_t> values;
    /// Image shape, 3D in CHW or HWC orientation.
    std::vector<std::size_t> dims;
};

/**
 * Convert a model image output tensor into an 8-bit image, mirroring the source
 * compute_image_output() and unnormalize_image() utilities. A leading batch dimension of 1 is
 * squeezed and the tensor must then be 3D. The values are min-max normalized to [0, 1], or only
 * shifted by the minimum when all values are equal, then scaled to [0, 255], clipped and truncated
 * to uint8.
 */
Uint8Image computeImageOutput(TensorData tensor) {
    if(tensor.dims.size() == 4 && tensor.dims[0] == 1) {
        tensor.dims.erase(tensor.dims.begin());
    }
    DAI_CHECK_V(tensor.dims.size() == 3, "Expected 3D output tensor, got {}D.", tensor.dims.size());
    DAI_CHECK(!tensor.values.empty(), "Expected a non-empty output tensor.");

    const auto [minIt, maxIt] = std::minmax_element(tensor.values.begin(), tensor.values.end());
    const float minValue = *minIt;
    const float maxValue = *maxIt;
    const bool scaleByRange = maxValue != minValue;
    const float range = maxValue - minValue;

    Uint8Image image;
    image.dims = std::move(tensor.dims);
    image.values.resize(tensor.values.size());
    for(std::size_t i = 0; i < tensor.values.size(); i++) {
        float value = tensor.values[i] - minValue;
        if(scaleByRange) {
            value /= range;
        }
        value *= 255.0f;
        value = std::clamp(value, 0.0f, 255.0f);
        image.values[i] = static_cast<std::uint8_t>(value);
    }
    return image;
}

/**
 * Create the output ImgFrame message from an 8-bit image, mirroring the source
 * create_image_message() creator and the ImgFrame::setCvFrame() data packing for the BGR888i,
 * BGR888p and GRAY8 frame types. The image orientation is detected as CHW when the first dimension
 * is 1 or 3 (checked first) and as HWC when the third dimension is 1 or 3. A single-channel image
 * is emitted as a GRAY8 frame; a 3-channel image is emitted with the requested BGR frame type,
 * swapping the channels from RGB to BGR unless the image is marked as already BGR.
 */
std::shared_ptr<dai::ImgFrame> createImageMessage(const Uint8Image& image, bool isBGR, dai::ImgFrame::Type requestedFrameType) {
    bool hwc = false;
    if(image.dims[0] == 1 || image.dims[0] == 3) {
        hwc = false;
    } else if(image.dims[2] == 1 || image.dims[2] == 3) {
        hwc = true;
    } else {
        DAI_CHECK_V(false, "Unexpected image shape. Expected CHW or HWC, got ({}, {}, {}).", image.dims[0], image.dims[1], image.dims[2]);
    }

    const std::size_t height = hwc ? image.dims[0] : image.dims[1];
    const std::size_t width = hwc ? image.dims[1] : image.dims[2];
    const std::size_t channels = hwc ? image.dims[2] : image.dims[0];
    const std::size_t planeSize = height * width;

    const auto valueAt = [&](std::size_t y, std::size_t x, std::size_t channel) {
        return hwc ? image.values[(y * width + x) * channels + channel] : image.values[(channel * height + y) * width + x];
    };

    auto frame = std::make_shared<dai::ImgFrame>();
    std::vector<std::uint8_t> data;

    if(channels == 1) {
        // Grayscale: squeeze to a single plane. The source creator forces GRAY8 unless a RAW*
        // or GRAY* frame type is requested; this parser only ever requests BGR888p or BGR888i,
        // so a grayscale image is always emitted as GRAY8.
        data.resize(planeSize);
        for(std::size_t y = 0; y < height; y++) {
            for(std::size_t x = 0; x < width; x++) {
                data[y * width + x] = valueAt(y, x, 0);
            }
        }
        frame->setType(dai::ImgFrame::Type::GRAY8);
        frame->setStride(static_cast<unsigned int>(width));
    } else {
        // Color: when the image is RGB (isBGR == false) the red and blue channels are swapped,
        // mirroring the source cv2.cvtColor(image, cv2.COLOR_RGB2BGR) conversion.
        const std::size_t blueChannel = isBGR ? 0 : 2;
        const std::size_t greenChannel = 1;
        const std::size_t redChannel = isBGR ? 2 : 0;
        data.resize(planeSize * 3);
        if(requestedFrameType == dai::ImgFrame::Type::BGR888p) {
            // Planar packing: B plane first, then G and R planes.
            for(std::size_t y = 0; y < height; y++) {
                for(std::size_t x = 0; x < width; x++) {
                    data[y * width + x] = valueAt(y, x, blueChannel);
                    data[planeSize + y * width + x] = valueAt(y, x, greenChannel);
                    data[2 * planeSize + y * width + x] = valueAt(y, x, redChannel);
                }
            }
            frame->setType(dai::ImgFrame::Type::BGR888p);
            frame->setStride(static_cast<unsigned int>(width));
            frame->fb.p1Offset = 0;
            frame->fb.p2Offset = static_cast<unsigned int>(planeSize);
            frame->fb.p3Offset = static_cast<unsigned int>(2 * planeSize);
        } else {
            // Interleaved packing: B, G, R values per pixel.
            for(std::size_t y = 0; y < height; y++) {
                for(std::size_t x = 0; x < width; x++) {
                    data[(y * width + x) * 3] = valueAt(y, x, blueChannel);
                    data[(y * width + x) * 3 + 1] = valueAt(y, x, greenChannel);
                    data[(y * width + x) * 3 + 2] = valueAt(y, x, redChannel);
                }
            }
            frame->setType(dai::ImgFrame::Type::BGR888i);
            frame->setStride(static_cast<unsigned int>(width * 3));
        }
    }

    frame->setSize(static_cast<unsigned int>(width), static_cast<unsigned int>(height));
    frame->setData(std::move(data));
    return frame;
}

}  // namespace

NNArchive ImageOutputParser::createNNArchive(NNModelDescription& modelDesc) {
    // Download model from zoo
    if(modelDesc.platform.empty()) {
        auto device = getParentPipeline().getDefaultDevice();
        DAI_CHECK(device != nullptr, "Device is not set. Specify the platform in the NNModelDescription when building without a device.");
        modelDesc.platform = device->getPlatformAsString();
    }
    auto path = getModelFromZoo(modelDesc);
    auto modelType = model::readModelType(path);
    DAI_CHECK(modelType == model::ModelType::NNARCHIVE, "Model from zoo must be an NNArchive to use the build function");
    auto nnArchive = NNArchive(path);
    return nnArchive;
}

NNArchive ImageOutputParser::decodeModel(const Model& model) {
    std::optional<NNArchive> nnArchive;

    if(const auto* s = std::get_if<std::string>(&model)) {
        NNModelDescription description;
        description.model = *s;
        nnArchive = createNNArchive(description);
    } else if(const auto* desc = std::get_if<NNModelDescription>(&model)) {
        NNModelDescription tmpDesc = *desc;
        nnArchive = createNNArchive(tmpDesc);
    } else if(const auto* archive = std::get_if<NNArchive>(&model)) {
        nnArchive = *archive;
    }

    DAI_CHECK_V(nnArchive.has_value(), "Unsupported model type passed to ImageOutputParser::build");
    return *nnArchive;
}

std::shared_ptr<ImageOutputParser> ImageOutputParser::build(Node::Output& nnInput, const Model& model) {
    auto nnArchive = decodeModel(model);
    setConfig(nnArchive.getVersionedConfig());
    nnInput.link(input);
    return std::static_pointer_cast<ImageOutputParser>(shared_from_this());
}

std::shared_ptr<ImageOutputParser> ImageOutputParser::build(Node::Output& nnInput, const dai::nn_archive::v1::Head& head) {
    setConfig(head);
    nnInput.link(input);
    return std::static_pointer_cast<ImageOutputParser>(shared_from_this());
}

void ImageOutputParser::setNNArchive(const NNArchive& nnArchive) {
    setConfig(nnArchive.getVersionedConfig());
}

void ImageOutputParser::setNNArchiveHead(const dai::nn_archive::v1::Head& head) {
    setConfig(head);
}

void ImageOutputParser::setConfig(const dai::NNArchiveVersionedConfig& config) {
    DAI_CHECK_V(config.getVersion() == NNArchiveConfigVersion::V1, "Only NNArchive config V1 is supported.");
    const auto& configV1 = config.getConfig<nn_archive::v1::Config>();
    DAI_CHECK(configV1.model.heads, "Heads array is not defined in the NN Archive config file.");

    int imageOutputHeads = 0;
    auto imageOutputHead = dai::nn_archive::v1::Head{};
    for(const auto& head : *configV1.model.heads) {
        if(head.parser == NAME) {
            imageOutputHeads++;
            imageOutputHead = head;
        }
    }

    DAI_CHECK_V(imageOutputHeads > 0, "NNArchive does not contain an ImageOutputParser head.");
    DAI_CHECK_V(imageOutputHeads == 1, "NNArchive contains {} ImageOutputParser heads. Please build with a specific head.", imageOutputHeads);

    setConfig(imageOutputHead);
}

void ImageOutputParser::setConfig(const dai::nn_archive::v1::Head& head) {
    DAI_CHECK_V(head.parser == NAME, "The provided head is not an ImageOutputParser head, got '{}'.", head.parser);

    const std::size_t numOutputs = head.outputs.has_value() ? head.outputs->size() : 0;
    DAI_CHECK_V(numOutputs == 1, "ImageOutputParser expects exactly 1 output layer, got {} layers.", numOutputs);
    setOutputLayerName(head.outputs->front());

    // The output_is_bgr key without a typed metadata field lives in the head metadata extra
    // parameters.
    const auto& extraParams = head.metadata.extraParams;

    if(extraParams.is_object() && extraParams.contains("output_is_bgr") && !extraParams.at("output_is_bgr").is_null()) {
        const auto& outputIsBGRJson = extraParams.at("output_is_bgr");
        DAI_CHECK(outputIsBGRJson.is_boolean(), "output_is_bgr must be a boolean.");
        setBGROutput(outputIsBGRJson.get<bool>());
    }
}

void ImageOutputParser::setOutputLayerName(const std::string& outputLayerName) {
    properties.outputLayerName = outputLayerName;
}

std::string ImageOutputParser::getOutputLayerName() const {
    return properties.outputLayerName;
}

void ImageOutputParser::setBGROutput(bool outputIsBGR) {
    properties.outputIsBGR = outputIsBGR;
}

bool ImageOutputParser::getBGROutput() const {
    return properties.outputIsBGR;
}

void ImageOutputParser::setRunOnHost(bool runOnHost) {
    runOnHostVar = runOnHost;
}

bool ImageOutputParser::runOnHost() const {
    return getDevice() == nullptr || runOnHostVar;
}

void ImageOutputParser::run() {
    auto& logger = ThreadedNode::pimpl->logger;
    logger->debug("ImageOutputParser started");

    // The color output frame type is resolved from the pipeline's default device platform once at
    // startup, mirroring the source parser: BGR888p on RVC2, BGR888i otherwise. A device-less
    // pipeline has no platform to query, so the source creator's default BGR888i is used.
    ImgFrame::Type requestedFrameType = ImgFrame::Type::BGR888i;
    auto device = getParentPipeline().getDefaultDevice();
    if(device != nullptr && device->getPlatformAsString() == "RVC2") {
        requestedFrameType = ImgFrame::Type::BGR888p;
    }

    // The resolved layer name persists across messages once auto-selected from a single-tensor
    // NNData, mirroring the source parser behavior.
    std::string resolvedOutputLayerName = properties.outputLayerName;

    while(mainLoop()) {
        std::shared_ptr<dai::NNData> nnData;
        {
            auto blockEvent = this->inputBlockEvent();
            nnData = input.get<dai::NNData>();
            if(!nnData) {
                continue;
            }
        }

        // Extract
        const auto layerNames = nnData->getAllLayerNames();
        if(resolvedOutputLayerName.empty()) {
            DAI_CHECK_V(
                layerNames.size() == 1, "ImageOutputParser: expected 1 output layer, got {} layers. Please provide the output layer name.", layerNames.size());
            resolvedOutputLayerName = layerNames.front();
        }

        auto tensor = getTensorData(*nnData, resolvedOutputLayerName);

        // Compute. The image dimensions and channel count are derived from the tensor shape.
        auto image = computeImageOutput(std::move(tensor));

        // Emit
        auto message = createImageMessage(image, properties.outputIsBGR, requestedFrameType);
        if(nnData->transformation.has_value()) {
            message->setTransformation(*nnData->transformation);
        }
        message->setBufferMetadataFrom(nnData);

        logger->debug("ImageOutputParser created image message with size {}x{}", message->getWidth(), message->getHeight());
        {
            auto blockEvent = this->outputBlockEvent();
            out.send(message);
        }
    }
}

}  // namespace node
}  // namespace beta
}  // namespace dai
