#include <catch2/catch_all.hpp>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/pipeline/node/FocusedDepth.hpp"

TEST_CASE("FocusedDepth::pickNeuralModelForRoi smallest at-least model", "[FocusedDepth][host]") {
    using dai::DeviceModelZoo;
    using dai::node::FocusedDepth;
    REQUIRE(FocusedDepth::pickNeuralModelForRoi(200, 100) == DeviceModelZoo::NEURAL_DEPTH_288X180);
    REQUIRE(FocusedDepth::pickNeuralModelForRoi(480, 300) == DeviceModelZoo::NEURAL_DEPTH_480X300);
    REQUIRE(FocusedDepth::pickNeuralModelForRoi(2000, 2000) == DeviceModelZoo::NEURAL_DEPTH_1248X780);
}
