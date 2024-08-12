#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/pipeline/datatype/NNData.hpp>

#include "depthai/common/TensorInfo.hpp"
#include "xtensor/xmanipulation.hpp"

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