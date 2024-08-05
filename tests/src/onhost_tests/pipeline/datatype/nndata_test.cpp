#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>

#include <depthai/pipeline/datatype/NNData.hpp>

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

TEST_CASE("NNData can perform storage order conversions") {
    // Add  test here...
}