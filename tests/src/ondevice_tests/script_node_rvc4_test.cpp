
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"

const std::string kNNDataRoundtripScript = R"(
while True:
    src = node.inputs["in"].get()
    if src is None:
        break

    node.outputs["out"].send(src)
    break
)";

void setCommonMessageFields(dai::NNData& msg, int seqNo, int timestampMs) {
    auto timestamp = std::chrono::steady_clock::now() + std::chrono::milliseconds{timestampMs};
    msg.setSequenceNum(seqNo);
    msg.setTimestamp(timestamp);
    msg.setTimestampDevice(timestamp + std::chrono::milliseconds{7});
    msg.batchSize = 3;

    dai::ImgTransformation transform(640, 400);
    transform.addCrop(14, 18, 320, 220).addScale(0.65F, 0.85F);
    msg.transformation = transform;
}

template <typename T>
std::shared_ptr<T> runTypedScriptRoundtrip(const std::shared_ptr<T>& input, const std::string& scriptText) {
    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(scriptText);

    auto in = script->inputs["in"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.start();
    in->send(input);

    bool hasTimedOut = false;
    auto output = out->get<T>(std::chrono::seconds(1), hasTimedOut);
    REQUIRE_FALSE(hasTimedOut);
    REQUIRE(output != nullptr);
    return output;
}

void requireTensorInfoEqual(const dai::TensorInfo& expected, const dai::TensorInfo& actual) {
    REQUIRE(actual.order == expected.order);
    REQUIRE(actual.dataType == expected.dataType);
    REQUIRE(actual.numDimensions == expected.numDimensions);
    REQUIRE(actual.dims == expected.dims);
    REQUIRE(actual.strides == expected.strides);
    REQUIRE(actual.name == expected.name);
    REQUIRE(actual.offset == expected.offset);
    REQUIRE(actual.quantization == expected.quantization);
    REQUIRE(actual.qpScale == Catch::Approx(expected.qpScale));
    REQUIRE(actual.qpZp == Catch::Approx(expected.qpZp));
}

TEST_CASE("NNData roundtrip in Script node") {
    auto input = std::make_shared<dai::NNData>();
    setCommonMessageFields(*input, 301, 1434);

    xt::xarray<int> intTensor = {{1, -2, 3}, {4, 5, -6}};
    xt::xarray<uint16_t> u16Tensor = {{{1000U, 2049U}, {4097U, 65535U}}, {{7U, 8U}, {9U, 10U}}};
    xt::xarray<float> fp16Tensor = {0.25F, -1.5F, 3.75F, 9.0F};
    xt::xarray<float> fp32Tensor = {{1.25F, -2.5F}, {3.0F, 4.125F}};
    xt::xarray<double> fp64Tensor = {1.0, -2.25, 3.5, 8.75};
    xt::xarray<std::int8_t> i8Tensor = {{-128, -2, 3}, {4, 5, 127}};
    xt::xarray<std::uint8_t> u8Tensor = {0U, 1U, 127U, 255U};

    const std::vector<std::pair<std::string, dai::TensorInfo::DataType>> expectedDatatypes = {
        {"int_tensor", dai::TensorInfo::DataType::INT},
        {"u16_tensor", dai::TensorInfo::DataType::U16F},
        {"fp16_tensor", dai::TensorInfo::DataType::FP16},
        {"fp32_tensor", dai::TensorInfo::DataType::FP32},
        {"fp64_tensor", dai::TensorInfo::DataType::FP64},
        {"i8_tensor", dai::TensorInfo::DataType::I8},
        {"u8_tensor", dai::TensorInfo::DataType::U8F},
    };

    input->addTensor("int_tensor", intTensor, dai::TensorInfo::DataType::INT, dai::TensorInfo::StorageOrder::NC);
    input->addTensor("u16_tensor", u16Tensor, dai::TensorInfo::DataType::U16F, dai::TensorInfo::StorageOrder::HCW);
    input->addTensor("fp16_tensor", fp16Tensor, dai::TensorInfo::DataType::FP16, dai::TensorInfo::StorageOrder::C);
    input->addTensor("fp32_tensor", fp32Tensor, dai::TensorInfo::DataType::FP32, dai::TensorInfo::StorageOrder::NC);
    input->addTensor("fp64_tensor", fp64Tensor, dai::TensorInfo::DataType::FP64, dai::TensorInfo::StorageOrder::C);
    input->addTensor("i8_tensor", i8Tensor, dai::TensorInfo::DataType::I8, dai::TensorInfo::StorageOrder::NC);
    input->addTensor("u8_tensor", u8Tensor, dai::TensorInfo::DataType::U8F, dai::TensorInfo::StorageOrder::C);

    auto u16InfoIt = std::find_if(input->tensors.begin(), input->tensors.end(), [](const dai::TensorInfo& tensor) { return tensor.name == "u16_tensor"; });
    REQUIRE(u16InfoIt != input->tensors.end());
    u16InfoIt->quantization = true;
    u16InfoIt->qpScale = 0.5F;
    u16InfoIt->qpZp = 11.0F;

    auto output = runTypedScriptRoundtrip(input, kNNDataRoundtripScript);

    REQUIRE(output->getSequenceNum() == input->getSequenceNum());
    REQUIRE(output->getTimestamp() == input->getTimestamp());
    REQUIRE(output->getTimestampDevice() == input->getTimestampDevice());
    REQUIRE(output->batchSize == input->batchSize);

    REQUIRE(expectedDatatypes.size() == input->tensors.size());

    const auto expectedLayers = input->getAllLayers();
    const auto actualLayers = output->getAllLayers();
    REQUIRE(actualLayers.size() == expectedLayers.size());
    REQUIRE(actualLayers.size() == expectedDatatypes.size());
    for(const auto& [tensorName, expectedDatatype] : expectedDatatypes) {
        auto expectedInfo = input->getTensorInfo(tensorName);
        auto actualInfo = output->getTensorInfo(tensorName);
        REQUIRE(expectedInfo.has_value());
        REQUIRE(actualInfo.has_value());
        REQUIRE(actualInfo->dataType == expectedDatatype);
        requireTensorInfoEqual(*expectedInfo, *actualInfo);
    }
}