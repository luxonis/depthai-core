#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>
#include <limits>

#include "depthai/common/TensorInfo.hpp"
#include "xtensor/misc/xmanipulation.hpp"

TEST_CASE("NNData describes contiguous byte strides for singleton dimensions") {
    xt::xarray<float> tensor = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    tensor.reshape({1, 3, 1, 4});
    for(const auto dataType : {dai::TensorInfo::DataType::FP16,
                               dai::TensorInfo::DataType::FP32,
                               dai::TensorInfo::DataType::FP64,
                               dai::TensorInfo::DataType::INT,
                               dai::TensorInfo::DataType::I8,
                               dai::TensorInfo::DataType::U8F,
                               dai::TensorInfo::DataType::U16F}) {
        CAPTURE(dataType);
        dai::NNData message;
        message.addTensor("singleton", tensor, dataType, dai::TensorInfo::StorageOrder::NCHW);
        const auto& info = message.tensors.at(0);
        const auto bytes = static_cast<unsigned>(info.getDataTypeSize());
        REQUIRE(info.dims == std::vector<unsigned>{1, 3, 1, 4});
        REQUIRE(info.strides == std::vector<unsigned>{12 * bytes, 4 * bytes, 4 * bytes, bytes});
        REQUIRE((message.getTensor<float>("singleton") == tensor));
    }
}

TEST_CASE("NNData copies custom-strided tensors in logical row-major order") {
    using Tensor = xt::xarray<float>;
    Tensor::storage_type storage = {0, 1, 2, 3, 4, 5, 6, 7, 8};
    Tensor::inner_strides_type strides = {1, 2, 4};
    Tensor expected = {{{0, 4}, {2, 6}}, {{1, 5}, {3, 7}}};
    SECTION("Dense storage") {
        storage = Tensor::storage_type{0, 1, 2, 3, 4, 5, 6, 7};
    }
    SECTION("Padded storage") {
        strides = {1, 2, 5};
        expected = {{{0, 5}, {2, 7}}, {{1, 6}, {3, 8}}};
    }
    SECTION("Broadcast storage") {
        storage = Tensor::storage_type{3};
        strides = {0, 0, 0};
        expected.fill(3);
    }
    const Tensor tensor(std::move(storage), Tensor::inner_shape_type{2, 2, 2}, std::move(strides));
    for(const auto dataType : {dai::TensorInfo::DataType::FP16,
                               dai::TensorInfo::DataType::FP32,
                               dai::TensorInfo::DataType::FP64,
                               dai::TensorInfo::DataType::INT,
                               dai::TensorInfo::DataType::I8,
                               dai::TensorInfo::DataType::U8F,
                               dai::TensorInfo::DataType::U16F}) {
        CAPTURE(dataType);
        CAPTURE(tensor.strides());
        dai::NNData message;
        message.addTensor("strided", tensor, dataType, dai::TensorInfo::StorageOrder::CHW);
        const auto& info = message.tensors.at(0);
        const auto bytes = static_cast<unsigned>(info.getDataTypeSize());
        REQUIRE(info.dims == std::vector<unsigned>{2, 2, 2});
        REQUIRE(info.strides == std::vector<unsigned>{4 * bytes, 2 * bytes, bytes});
        REQUIRE(message.getData().size() == 8 * bytes);
        REQUIRE((message.getTensor<float>("strided") == expected));
    }
}

TEST_CASE("NNData rejects tensors exceeding stride capacity without modifying existing data") {
    using Tensor = xt::xarray<float>;
    dai::NNData message;
    message.addTensor("existing", std::vector<std::uint8_t>{7});
    const std::vector<std::uint8_t> originalData(message.getData().begin(), message.getData().end());
    for(const auto& shape : {Tensor::inner_shape_type{1U << 21, 1U << 20, 1U << 20}, Tensor::inner_shape_type{1U << 22, 1U << 21, 1U << 21}}) {
        CAPTURE(shape);
        const Tensor tensor(Tensor::storage_type{1}, Tensor::inner_shape_type(shape), Tensor::inner_strides_type{0, 0, 0});
        REQUIRE_THROWS_AS(message.addTensor("oversized", tensor, dai::TensorInfo::DataType::FP64), std::overflow_error);
        const std::vector<std::uint8_t> remainingData(message.getData().begin(), message.getData().end());
        REQUIRE(remainingData == originalData);
        REQUIRE(message.getAllLayerNames() == std::vector<std::string>{"existing"});
    }
    message.addTensor("next", std::vector<std::uint8_t>{8});
    REQUIRE(message.getTensor<std::uint8_t>("existing")(0, 0) == 7);
    REQUIRE(message.getTensor<std::uint8_t>("next")(0, 0) == 8);
}

TEST_CASE("NNData rejects tensors exceeding buffer capacity without modifying existing data") {
    using Tensor = xt::xarray<float>;
    dai::NNData message;
    message.addTensor("existing", std::vector<std::uint8_t>{7});
    const std::vector<std::uint8_t> originalData(message.getData().begin(), message.getData().end());
    const auto outerDimension = std::numeric_limits<unsigned int>::max();
    // Keep the strides representable while the total byte count exceeds buffer capacity.
    const auto innerDimension = std::vector<std::uint8_t>().max_size() / outerDimension + 1;
    const Tensor tensor(Tensor::storage_type{1}, Tensor::inner_shape_type{outerDimension, innerDimension}, Tensor::inner_strides_type{0, 0});
    REQUIRE_THROWS_AS(message.addTensor("oversized", tensor, dai::TensorInfo::DataType::U8F), std::length_error);
    const std::vector<std::uint8_t> remainingData(message.getData().begin(), message.getData().end());
    REQUIRE(remainingData == originalData);
    REQUIRE(message.getAllLayerNames() == std::vector<std::string>{"existing"});
}

TEST_CASE("a") {
    dai::NNData nndata;

    xt::xarray<int, xt::layout_type::dynamic> tensorA1 = {{1, 2, 3}, {4, 5, 7}};
    xt::xarray<double, xt::layout_type::dynamic> tensorB1 = {{1, 2, 3}, {4, 5, 7}};
    std::vector<int> tensorC1 = {1, 2, 3, 4, 5};
    std::vector<double> tensorD1 = {1, 2, 3, 4, 5};

    nndata.addTensor<int>("a", tensorA1);
    nndata.addTensor<double>("b", tensorB1);
    nndata.addTensor<int>("c", tensorC1);
    nndata.addTensor<double>("d", tensorD1);

    const auto tensorA2 = nndata.getTensor<int>("a");
    const auto tensorB2 = nndata.getTensor<double>("b");
    const auto tensorC2 = nndata.getTensor<int>("c");
    const auto tensorD2 = nndata.getTensor<double>("d");

    REQUIRE((tensorA1 == tensorA2));
    REQUIRE((tensorB1 == tensorB2));
    REQUIRE([&]() {
        for(uint32_t i = 0; i < tensorC1.size(); i++) {
            if(tensorC1[i] != tensorC2[i]) {
                return 0;
            }
        }
        return 1;
    }());
    REQUIRE([&]() {
        for(uint32_t i = 0; i < tensorD1.size(); i++) {
            if(tensorD1[i] != tensorD2[i]) {
                return 0;
            }
        }
        return 1;
    }());
}

TEST_CASE("NNData uint16 tensors use U16F storage") {
    dai::NNData nndata;

    xt::xarray<uint16_t> tensor = {{0U, 2049U, 4097U}, {42U, 511U, 8191U}};
    std::vector<uint16_t> vector = {0U, 2049U, 4097U, 42U, 511U, 8191U};

    nndata.addTensor("tensor_u16", tensor, dai::TensorInfo::DataType::U16F);
    nndata.addTensor("vector_u16", vector, dai::TensorInfo::DataType::U16F);

    REQUIRE(nndata.getTensorDatatype("tensor_u16") == dai::TensorInfo::DataType::U16F);
    REQUIRE(nndata.getTensorDatatype("vector_u16") == dai::TensorInfo::DataType::U16F);

    REQUIRE((nndata.getTensor<uint16_t>("tensor_u16") == tensor));

    const auto vectorRoundTrip = nndata.getTensor<uint16_t>("vector_u16");
    REQUIRE(vectorRoundTrip.shape() == std::vector<size_t>{1, vector.size()});
    REQUIRE([&]() {
        for(uint32_t i = 0; i < vector.size(); i++) {
            if(vector[i] != vectorRoundTrip[i]) {
                return 0;
            }
        }
        return 1;
    }());
}

TEST_CASE("NNData int storage conversions") {
    dai::NNData nndata;
    xt::xarray<int, xt::layout_type::dynamic> tensorA1 = {{1, 2, 3}, {4, 5, 7}};  // 2x3

    // Add wrong storage order (NCHW = 4 dims) vs (2 dims)
    REQUIRE_THROWS(nndata.addTensor<int>("a", tensorA1, dai::TensorInfo::StorageOrder::NCHW));

    // Add data
    REQUIRE_NOTHROW(nndata.addTensor<int>("a1", tensorA1, dai::TensorInfo::StorageOrder::NC));

    // Get data and check shape
    REQUIRE(nndata.getTensor<int>("a1", dai::TensorInfo::StorageOrder::NC).shape() == std::vector<size_t>{2, 3});
    REQUIRE(nndata.getTensor<int>("a1", dai::TensorInfo::StorageOrder::CN).shape() == std::vector<size_t>{3, 2});
    REQUIRE(nndata.getTensor<int>("a1", dai::TensorInfo::StorageOrder::NCHW).shape() == std::vector<size_t>{2, 3, 1, 1});

    // Check content
    REQUIRE((nndata.getTensor<int>("a1", dai::TensorInfo::StorageOrder::NC) == tensorA1));
    REQUIRE((nndata.getTensor<int>("a1", dai::TensorInfo::StorageOrder::CN) == xt::transpose(tensorA1)));
}

TEST_CASE("NNData float storage conversions") {
    dai::NNData nndata;
    xt::xarray<double, xt::layout_type::dynamic> tensorB1 = {1, 2, 3, 4, 5, 7};  // 6

    REQUIRE_THROWS(nndata.addTensor<double>("b", tensorB1, dai::TensorInfo::StorageOrder::CHW));
    REQUIRE_NOTHROW(nndata.addTensor<float>("b1", tensorB1, dai::TensorInfo::StorageOrder::C));

    REQUIRE(nndata.getTensor<double>("b1", dai::TensorInfo::StorageOrder::C).shape() == std::vector<size_t>{6});
    REQUIRE(nndata.getTensor<double>("b1", dai::TensorInfo::StorageOrder::NC).shape() == std::vector<size_t>{1, 6});
    REQUIRE(nndata.getTensor<double>("b1", dai::TensorInfo::StorageOrder::CN).shape() == std::vector<size_t>{6, 1});
    REQUIRE(nndata.getTensor<double>("b1", dai::TensorInfo::StorageOrder::NCHW).shape() == std::vector<size_t>{1, 6, 1, 1});

    REQUIRE(nndata.getTensor<double>("b1", dai::TensorInfo::StorageOrder::C) == tensorB1);
}

TEST_CASE("NNData double storage conversions") {
    dai::NNData nndata;
    xt::xarray<float, xt::layout_type::dynamic> tensorC1 = {{1, 2, 3, 4, 5, 7}};  // 1x6

    REQUIRE_THROWS(nndata.addTensor<float>("cc1", tensorC1, dai::TensorInfo::StorageOrder::CHW));
    REQUIRE_THROWS(nndata.addTensor<float>("cc1", tensorC1, dai::TensorInfo::StorageOrder::NHCW));

    REQUIRE_NOTHROW(nndata.addTensor<float>("c1", tensorC1, dai::TensorInfo::StorageOrder::NC));

    REQUIRE(nndata.getTensor<float>("c1", dai::TensorInfo::StorageOrder::NC).shape() == std::vector<size_t>{1, 6});
    REQUIRE(nndata.getTensor<float>("c1", dai::TensorInfo::StorageOrder::CN).shape() == std::vector<size_t>{6, 1});
    REQUIRE(nndata.getTensor<float>("c1", dai::TensorInfo::StorageOrder::C).shape() == std::vector<size_t>{6});
    REQUIRE(nndata.getTensor<float>("c1", dai::TensorInfo::StorageOrder::NCHW).shape() == std::vector<size_t>{1, 6, 1, 1});

    REQUIRE_THROWS_AS(nndata.getTensor<float>("c1", dai::TensorInfo::StorageOrder::W), std::runtime_error);

    REQUIRE(nndata.getTensor<float>("c1", dai::TensorInfo::StorageOrder::NC) == tensorC1);
}

TEST_CASE("addTensor overloads") {
    dai::NNData nndata;
    xt::xarray<int> tensorINT = {{1, 2, 1}, {4, -5, 9}};
    xt::xarray<uint16_t> tensorUINT16 = {{1, 2, 1}, {4, 5, 9}};
    xt::xarray<float> tensorFLOAT = {{1.1, 2, 1.3}, {4.4, 5.5, 9.6}};
    xt::xarray<double> tensorDOUBLE = {{1, 2, 1.1}, {4, 5.2, 9.3}};
    xt::xarray<std::int8_t> tensorINT8 = {{1, 2, -1}, {4, 5, 9}};
    xt::xarray<std::uint8_t> tensorUINT8 = {{1, 2, 1}, {4, 5, 9}};

    std::vector<int> vectorINT = {1, 2, 1, 4, -5, 9};
    std::vector<uint16_t> vectorUINT16 = {1, 2, 1, 4, 5, 9};
    std::vector<float> vectorFLOAT = {1.1, 2, 1.3, 4.4, 5.5, 9.6};
    std::vector<double> vectorDOUBLE = {1, 2, 1.1, 4, 5.2, 9.3};
    std::vector<std::int8_t> vectorINT8 = {1, 2, -1, 4, 5, 9};
    std::vector<std::uint8_t> vectorUINT8 = {1, 2, 1, 4, 5, 9};

    REQUIRE_NOTHROW(nndata.addTensor("INT", tensorINT));
    REQUIRE_NOTHROW(nndata.addTensor("UINT16", tensorUINT16));
    REQUIRE_NOTHROW(nndata.addTensor("FLOAT", tensorFLOAT));
    REQUIRE_NOTHROW(nndata.addTensor("DOUBLE", tensorDOUBLE));
    REQUIRE_NOTHROW(nndata.addTensor("INT8", tensorINT8));
    REQUIRE_NOTHROW(nndata.addTensor("UINT8", tensorUINT8));

    REQUIRE_NOTHROW(nndata.addTensor("VINT", vectorINT));
    REQUIRE_NOTHROW(nndata.addTensor("VUINT16", vectorUINT16));
    REQUIRE_NOTHROW(nndata.addTensor("VFLOAT", vectorFLOAT));
    REQUIRE_NOTHROW(nndata.addTensor("VDOUBLE", vectorDOUBLE));
    REQUIRE_NOTHROW(nndata.addTensor("VINT8", vectorINT8));
    REQUIRE_NOTHROW(nndata.addTensor("VUINT8", vectorUINT8));

    std::vector<std::string> thr = {"abc", "asdf", "jkl;"};
    REQUIRE_THROWS(nndata.addTensor("STR", thr));
}
