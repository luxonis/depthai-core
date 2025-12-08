#include <catch2/catch_all.hpp>
#include <depthai/device/CalibrationHandler.hpp>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <vector>

using namespace dai;

// Helper to create calibration data directly in code
static CalibrationHandler loadHandler() {
    nlohmann::json calibJson = {
        {"cameraData",
         {{0,
           {{"cameraType", 0},
            {"distortionCoeff", nlohmann::json::array()},
            {"extrinsics",
             {{"rotationMatrix", {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
              {"specTranslation", {{"x", 0.0}, {"y", -4.9000301361084}, {"z", 0.0}}},
              {"toCameraSocket", -1},
              {"translation", {{"x", 0.0}, {"y", -4.9000301361084}, {"z", 0.0}}}}},
            {"height", 1080},
            {"intrinsicMatrix", {{613.353271484375, 0.0, 863.0769653320312}, {0.0, 612.0604248046875, 535.8486328125}, {0.0, 0.0, 1.0}}},
            {"lensPosition", 0},
            {"specHfovDeg", 0.0},
            {"width", 1920}}},
          {3,
           {{"cameraType", 0},
            {"distortionCoeff", nlohmann::json::array()},
            {"extrinsics",
             {{"rotationMatrix", {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
              {"specTranslation", {{"x", 0.0}, {"y", -3.2509000301361084}, {"z", 0.0}}},
              {"toCameraSocket", 1},
              {"translation", {{"x", 0.0}, {"y", -3.2509000301361084}, {"z", 0.0}}}}},
            {"height", 1080},
            {"intrinsicMatrix", {{613.353271484375, 0.0, 863.0769653320312}, {0.0, 612.0604248046875, 535.8486328125}, {0.0, 0.0, 1.0}}},
            {"lensPosition", 0},
            {"specHfovDeg", 0.0},
            {"width", 1920}}},
          {2,
           {{"cameraType", 0},
            {"distortionCoeff", {0.0015864927554503083, -0.006391937844455242, 0.0014326359378173947, -0.0006138950702734292}},
            {"extrinsics",
             {{"rotationMatrix",
               {{0.9999265074729919, 0.006867990363389254, -0.009992731735110283},
                {-0.006895299535244703, 0.9999725818634033, -0.00270105991512537},
                {0.00997390691190958, 0.002769764279946685, 0.9999464154243469}}},
              {"specTranslation", {{"x", 6.548969268798828}, {"y", 0.01233222708106041}, {"z", 0.2119242250919342}}},
              {"toCameraSocket", -1},
              {"translation", {{"x", 6.548969268798828}, {"y", 0.01233222708106041}, {"z", 0.2119242250919342}}}}},
            {"height", 1200},
            {"intrinsicMatrix", {{613.353271484375, 0.0, 863.0769653320312}, {0.0, 612.0604248046875, 535.8486328125}, {0.0, 0.0, 1.0}}},
            {"lensPosition", 0},
            {"specHfovDeg", 0.0},
            {"width", 1920}}},
          {1,
           {{"cameraType", 0},
            {"distortionCoeff", {0.005035185255110264, -0.006586757488548756, -0.011081146076321602, 0.007332072593271732}},
            {"extrinsics",
             {{"rotationMatrix", {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}},
              {"specTranslation", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}},
              {"toCameraSocket", 0},
              {"translation", {{"x", 0.0}, {"y", 0.0}, {"z", 0.0}}}}},
            {"height", 1200},
            {"intrinsicMatrix", {{618.7698974609375, 0.0, 927.1336669921875}, {0.0, 617.28955078125, 548.0191040039062}, {0.0, 0.0, 1.0}}},
            {"lensPosition", 0},
            {"specHfovDeg", 0.0},
            {"width", 1920}}}}}};

    return CalibrationHandler::fromJson(calibJson);
}

TEST_CASE("Invalid camera ID throws", "[getCameraExtrinsics]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraExtrinsics(CameraBoardSocket::CAM_E, CameraBoardSocket::CAM_B, false), std::runtime_error);
}

TEST_CASE("No connection between two origins throws", "[getCameraExtrinsics]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_C, false), std::runtime_error);
}

TEST_CASE("Valid extrinsics for directly linked cameras", "[getCameraExtrinsics]") {
    auto handler = loadHandler();
    auto M = handler.getCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false);
    // Expect identity 4×4
    std::vector<std::vector<float>> I = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    REQUIRE(M == I);
}

TEST_CASE("Same-origin cameras return identity", "[getCameraExtrinsics]") {
    auto handler = loadHandler();
    // A→A should also be identity
    auto M = handler.getCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_A, false);
    std::vector<std::vector<float>> I = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    REQUIRE(M == I);
}

TEST_CASE("Multiple independent origins detected", "[getCameraExtrinsics]") {
    auto handler = loadHandler();
    // Create two separate trees: A→(root), B→A and D→C (root)
    auto R3 = std::vector<std::vector<float>>{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    auto zeros3 = std::vector<float>{0, 0, 0};
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, R3, zeros3, zeros3);
    REQUIRE_THROWS_AS(handler.getCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_D, false), std::runtime_error);
}

TEST_CASE("Cyclic connection throws", "[getCameraExtrinsics]") {
    auto handler = loadHandler();
    auto R3 = std::vector<std::vector<float>>{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    auto zeros3 = std::vector<float>{0, 0, 0};
    // A→B, B→C, C→A forms a cycle
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, R3, zeros3, zeros3);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C, R3, zeros3, zeros3);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_A, R3, zeros3, zeros3);
    REQUIRE_THROWS_AS(handler.getCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false), std::runtime_error);
}

TEST_CASE("Long chain extrinsics composition", "[getCameraExtrinsics]") {
    auto handler = loadHandler();
    auto R3 = std::vector<std::vector<float>>{{0.4, 0, 0}, {0, 0.3, 0}, {0, 0, 1.1}};
    auto zeros3 = std::vector<float>{0, 0, 0};

    // A→D, D→C, C→B, B→AUTO (origin)
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_D, R3, {0, 2, 0}, zeros3);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, R3, {1, -1, 0}, zeros3);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_B, R3, {0, 5, 3}, zeros3);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::AUTO, R3, zeros3, zeros3);

    auto M = handler.getCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_A, false);
    std::vector<std::vector<float>> expected = {
        {0.025600001f, 0.0f, 0.0f, -0.025600001f}, {0.0f, 0.008100001f, 0.0f, 0.003240019f}, {0.0f, 0.0f, 1.464100122f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}};
    REQUIRE(M == expected);
}
