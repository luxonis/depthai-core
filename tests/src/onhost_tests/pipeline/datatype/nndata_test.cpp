#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>

#include "depthai/common/TensorInfo.hpp"
#include "xtensor/misc/xmanipulation.hpp"

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