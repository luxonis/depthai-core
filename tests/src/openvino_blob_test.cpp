#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

// std
#include <atomic>
#include <iostream>

// Include depthai library
#include <depthai/depthai.hpp>

TEST_CASE("OpenVINO 2020.3 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2020_3_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2020_3);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2020.4 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2020_4_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2020_4);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.1 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_1_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_1);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.2 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_2_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_2);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.3 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_3_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_3);
    dai::Device d(p);
}

TEST_CASE("OpenVINO 2021.4 blob") {
    dai::Pipeline p;
    auto nn = p.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(OPENVINO_2021_4_BLOB_PATH);
    auto networkOpenvinoVersion = p.getOpenVINOVersion();
    REQUIRE(networkOpenvinoVersion == dai::OpenVINO::VERSION_2021_4);
    dai::Device d(p);
}