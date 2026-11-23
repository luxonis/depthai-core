#include <catch2/catch_test_macros.hpp>
#include <chrono>

#include "depthai/depthai.hpp"

TEST_CASE("Singleton and custom tensor strides and values survive a device roundtrip") {
    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();
    script->setScript("while True:\n    node.io['out'].send(node.io['in'].get())\n");
    auto input = script->inputs["in"].createInputQueue();
    auto output = script->outputs["out"].createOutputQueue();
    pipeline.start();

    xt::xarray<float> tensor = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    tensor.reshape({1, 3, 1, 4});
    auto message = std::make_shared<dai::NNData>();
    message->addTensor("singleton", tensor, dai::TensorInfo::DataType::FP32, dai::TensorInfo::StorageOrder::NCHW);
    using Tensor = xt::xarray<float>;
    const Tensor strided(Tensor::storage_type{0, 1, 2, 3, 4, 5, 6, 7}, Tensor::inner_shape_type{2, 2, 2}, Tensor::inner_strides_type{1, 2, 4});
    const Tensor expected = {{{0, 4}, {2, 6}}, {{1, 5}, {3, 7}}};
    message->addTensor("strided", strided, dai::TensorInfo::DataType::FP32, dai::TensorInfo::StorageOrder::CHW);
    input->send(message);
    bool timedOut = false;
    const auto received = output->get<dai::NNData>(std::chrono::seconds(5), timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(received != nullptr);
    const auto& info = received->tensors.at(0);
    REQUIRE(info.dims == std::vector<unsigned>{1, 3, 1, 4});
    REQUIRE(info.strides == std::vector<unsigned>{48, 16, 16, 4});
    REQUIRE(received->getDatatype() == dai::DatatypeEnum::NNData);
    REQUIRE((received->getTensor<float>("singleton") == tensor));
    const auto& stridedInfo = received->tensors.at(1);
    REQUIRE(stridedInfo.dims == std::vector<unsigned>{2, 2, 2});
    REQUIRE(stridedInfo.strides == std::vector<unsigned>{16, 8, 4});
    REQUIRE((received->getTensor<float>("strided") == expected));
    REQUIRE(received->getSequenceNum() == message->getSequenceNum());
    // RVC2 fills missing transport timestamps. Compare tensor serialization independently.
    received->setTimestamp(message->getTimestamp());
    received->setTimestampDevice(message->getTimestampDevice());
    received->setTimestampSystem(message->getTimestampSystem());
    REQUIRE(dai::utility::serialize(*received) == dai::utility::serialize(*message));
}
