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

TEST_CASE("Invalid camera ID throws", "[getCameraIntrinsics]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraIntrinsics(CameraBoardSocket::CAM_E, 1280, 800), std::out_of_range);
}

TEST_CASE("Invalid camera resolution", "[getCameraIntrinsics]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraIntrinsics(CameraBoardSocket::CAM_A, -1280, -800), std::runtime_error);
}

TEST_CASE("Valid intrinsics to camera", "[getCameraIntrinsics]") {
    auto handler = loadHandler();
    auto intrinsics = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, 1280, 800);
    handler.setCameraIntrinsics(CameraBoardSocket::CAM_B, intrinsics, 1280, 800);
    REQUIRE(handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, 1280, 800) == intrinsics);
}

TEST_CASE("Valid downscale; same aspect ratio", "[getCameraIntrinsics]") {
    // JSON shows socket 1 has width=1920, height=1200 (16:10) -> usually CAM_B
    const int nativeW = 1920;
    const int nativeH = 1200;

    const int downW = 640;
    const int downH = 400;

    auto handler = loadHandler();

    // Get native intrinsics (should exist in JSON)
    auto K_native = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, nativeW, nativeH);

    // Expected scaled intrinsics (float division, correct denominators)
    const float sx = static_cast<float>(downW) / static_cast<float>(nativeW);
    const float sy = static_cast<float>(downH) / static_cast<float>(nativeH);

    auto K_expected = K_native;
    K_expected[0][0] *= sx;  // fx
    K_expected[1][1] *= sy;  // fy
    K_expected[0][2] *= sx;  // cx
    K_expected[1][2] *= sy;  // cy

    // Ask handler directly for the downscaled size (should auto-scale or have a fallback)
    auto K_down = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, downW, downH);

    REQUIRE(K_down[0][0] == Catch::Approx(K_expected[0][0]).margin(1e-6));  // fx
    REQUIRE(K_down[1][1] == Catch::Approx(K_expected[1][1]).margin(1e-6));  // fy
    REQUIRE(K_down[0][2] == Catch::Approx(K_expected[0][2]).margin(1e-6));  // cx
    REQUIRE(K_down[1][2] == Catch::Approx(K_expected[1][2]).margin(1e-6));  // cy
}

TEST_CASE("Cropping updates principal point correctly", "[getCameraIntrinsics]") {
    auto handler = loadHandler();

    int width = 1280;
    int height = 800;

    auto intrinsicsFull = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, width, height);

    // Crop 100 px from left/top and 100 px from right/bottom
    Point2f topLeft(100, 100);
    Point2f bottomRight(width - 100, height - 100);

    auto intrinsicsCropped = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, width, height, topLeft, bottomRight);

    // cx and cy should be shifted left/up by the crop offset
    REQUIRE(intrinsicsCropped[0][2] == Catch::Approx(intrinsicsFull[0][2] - 100).margin(1e-6));  // cx
    REQUIRE(intrinsicsCropped[1][2] == Catch::Approx(intrinsicsFull[1][2] - 100).margin(1e-6));  // cy
}

TEST_CASE("Keep aspect ratio scaling", "[getCameraIntrinsics]") {
    auto handler = loadHandler();

    int srcW = 1280, srcH = 800;
    int dstW = 640, dstH = 480;  // different aspect ratio

    auto intrinsicsFull = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, srcW, srcH);

    auto intrinsicsAspectKept = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, dstW, dstH, {}, {}, true);
    auto intrinsicsAspectFree = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, dstW, dstH, {}, {}, false);

    // When keeping aspect ratio, fx/fy should scale uniformly
    float uniformScale = std::min(static_cast<float>(dstW) / srcW, static_cast<float>(dstH) / srcH);

    REQUIRE(intrinsicsAspectKept[0][0] == Catch::Approx(intrinsicsFull[0][0] * uniformScale).margin(1e-5));
    REQUIRE(intrinsicsAspectKept[1][1] == Catch::Approx(intrinsicsFull[1][1] * uniformScale).margin(1e-5));

    // When not keeping aspect ratio, scaling should differ in x/y
    float scaleX = static_cast<float>(dstW) / srcW;
    float scaleY = static_cast<float>(dstH) / srcH;
    REQUIRE(intrinsicsAspectFree[0][0] == Catch::Approx(intrinsicsFull[0][0] * scaleX).margin(1e-5));
    REQUIRE(intrinsicsAspectFree[1][1] == Catch::Approx(intrinsicsFull[1][1] * scaleY).margin(1e-5));
}

/*
 This test need to have proper redefinition of the Calibration handler for it to be viable,
 since there is a mistake with bottomRIght crop not even being initialized and as well the topLeft is appled
 in newly defined system instead of being defined in the origin system

TEST_CASE("Crop + keep aspect ratio scales and shifts correctly", "[getCameraIntrinsics]") {
    auto handler = loadHandler();

    int srcW = 1280;
    int srcH = 800;
    int dstW = 640;
    int dstH = 480;

    // Original intrinsics
    auto intrinsicsFull = handler.getCameraIntrinsics(CameraBoardSocket::CAM_B, srcW, srcH);

    // Define a crop: remove 100px from left/top and 50px from right/bottom
    Point2f topLeft(150, 150);
    Point2f bottomRight(dstW - 150, dstH - 150);

    // Get intrinsics with crop + keepAspectRatio enabled
    auto intrinsicsCropped = handler.getCameraIntrinsics(
        CameraBoardSocket::CAM_B,
        dstW,
        dstH,
        topLeft,
        bottomRight,
        false // keepAspectRatio
    );

    // Expected uniform scaling (smallest scale factor to keep aspect ratio)
    float scale = std::min(static_cast<float>(dstW) / (bottomRight.x - topLeft.x),
                           static_cast<float>(dstH) / (bottomRight.y - topLeft.y));

    // Expected principal point shift (account for crop, then scale)
    float expectedCx = (intrinsicsFull[0][2] - topLeft.x) * scale;
    float expectedCy = (intrinsicsFull[1][2] - topLeft.y) * scale;

    // Check focal lengths (uniform scaling)
    REQUIRE(intrinsicsCropped[0][0] == Catch::Approx(intrinsicsFull[0][0] * scale).margin(1e-5)); // fx
    REQUIRE(intrinsicsCropped[1][1] == Catch::Approx(intrinsicsFull[1][1] * scale).margin(1e-5)); // fy

    // Check principal points
    REQUIRE(intrinsicsCropped[0][2] == Catch::Approx(expectedCx).margin(1e-5)); // cx
    REQUIRE(intrinsicsCropped[1][2] == Catch::Approx(expectedCy).margin(1e-5)); // cy
}
*/

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

TEST_CASE("Invalid camera ID throws", "[getCameraTranslationVector]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraTranslationVector(CameraBoardSocket::CAM_E, CameraBoardSocket::CAM_B, false), std::runtime_error);
}

TEST_CASE("No connection between two origins throws", "[getCameraTranslationVector]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_C, false), std::runtime_error);
}

TEST_CASE("Same camera translation is zero vector", "[getCameraTranslationVector]") {
    auto handler = loadHandler();
    auto t = handler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_A, false);
    std::vector<float> zero = {0.0f, 0.0f, 0.0f};
    REQUIRE(t.size() == 3);
    REQUIRE(t[0] == Catch::Approx(zero[0]).margin(1e-6));
    REQUIRE(t[1] == Catch::Approx(zero[1]).margin(1e-6));
    REQUIRE(t[2] == Catch::Approx(zero[2]).margin(1e-6));
}

TEST_CASE("Directly linked cameras translation", "[getCameraTranslationVector]") {
    auto handler = loadHandler();

    std::vector<std::vector<float>> R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<float> tAB = {1.0f, 2.0f, 3.0f};
    std::vector<float> zeros = {0.0f, 0.0f, 0.0f};

    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, R, tAB, zeros);

    auto t = handler.getCameraTranslationVector(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, false);

    REQUIRE(t.size() == 3);
    REQUIRE(t[0] == Catch::Approx(tAB[0]).margin(1e-6));
    REQUIRE(t[1] == Catch::Approx(tAB[1]).margin(1e-6));
    REQUIRE(t[2] == Catch::Approx(tAB[2]).margin(1e-6));
}

TEST_CASE("Cyclic connection throws", "[getCameraTranslationVector]") {
    auto handler = loadHandler();
    auto R = std::vector<std::vector<float>>{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    auto zeros = std::vector<float>{0, 0, 0};

    handler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, R, zeros, zeros);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C, R, zeros, zeros);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_A, R, zeros, zeros);

    REQUIRE_THROWS_AS(handler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false), std::runtime_error);
}

TEST_CASE("Long chain translation composition", "[getCameraTranslationVector]") {
    auto handler = loadHandler();
    std::vector<std::vector<float>> R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    auto zeros3 = std::vector<float>{0, 0, 0};

    handler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_D, R, {1.0f, 0.0f, 0.0f}, {0, 0, 0});
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, R, {0.0f, 2.0f, 0.0f}, {0, 0, 0});
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_B, R, {0.0f, 0.0f, 3.0f}, {0, 0, 0});
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::AUTO, R, zeros3, zeros3);
    // Translation from A → D should be sum of all
    auto t = handler.getCameraTranslationVector(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, false);
    REQUIRE(t.size() == 3);
    REQUIRE(t[0] == Catch::Approx(1.0f).margin(1e-6));
    REQUIRE(t[1] == Catch::Approx(2.0f).margin(1e-6));
    REQUIRE(t[2] == Catch::Approx(3.0f).margin(1e-6));
}

TEST_CASE("Spec vs calibration translation distinction", "[getCameraTranslationVector]") {
    auto handler = loadHandler();

    // Suppose board spec gives 1 cm offset, calibration gives 1.1 cm offset
    std::vector<std::vector<float>> R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<float> specT = {1.0f, 0.0f, 0.0f};
    std::vector<float> calibT = {1.1f, 0.0f, 0.0f};

    // Fake "setCameraExtrinsics" stores both spec and calibration translations
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, R, calibT, specT);

    auto tSpec = handler.getCameraTranslationVector(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, true);
    auto tCalib = handler.getCameraTranslationVector(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, false);

    REQUIRE(tSpec.size() == 3);
    REQUIRE(tCalib.size() == 3);
    REQUIRE(tSpec[0] == Catch::Approx(1.0f).margin(1e-6));   // from board spec
    REQUIRE(tCalib[0] == Catch::Approx(1.1f).margin(1e-6));  // from calibration data
}

TEST_CASE("Extrinsics translation matches getCameraTranslationVector", "[getCameraTranslationVector][getCameraExtrinsics]") {
    auto handler = loadHandler();

    // Simple transform setup
    std::vector<std::vector<float>> R = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    std::vector<float> tAB = {3.2f, -1.4f, 5.0f};
    std::vector<float> specT = {3.0f, -1.0f, 5.0f};  // just to test useSpecTranslation

    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, R, tAB, specT);

    // Get 4x4 extrinsics matrix
    auto M = handler.getCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, false);
    REQUIRE(M.size() == 4);
    REQUIRE(M[0].size() == 4);

    // Extract translation vector from matrix (last column)
    std::vector<float> tFromExtrinsics = {M[0][3], M[1][3], M[2][3]};

    // Get translation directly
    auto tDirect = handler.getCameraTranslationVector(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, false);

    REQUIRE(tDirect.size() == 3);

    // Compare element-wise
    REQUIRE(tDirect[0] == Catch::Approx(tFromExtrinsics[0]).margin(1e-6));
    REQUIRE(tDirect[1] == Catch::Approx(tFromExtrinsics[1]).margin(1e-6));
    REQUIRE(tDirect[2] == Catch::Approx(tFromExtrinsics[2]).margin(1e-6));

    // Also check consistency for spec translation
    auto Mspec = handler.getCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, true);
    std::vector<float> tSpecFromMatrix = {Mspec[0][3], Mspec[1][3], Mspec[2][3]};
    auto tSpecDirect = handler.getCameraTranslationVector(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, true);

    REQUIRE(tSpecDirect[0] == Catch::Approx(tSpecFromMatrix[0]).margin(1e-6));
    REQUIRE(tSpecDirect[1] == Catch::Approx(tSpecFromMatrix[1]).margin(1e-6));
    REQUIRE(tSpecDirect[2] == Catch::Approx(tSpecFromMatrix[2]).margin(1e-6));
}

TEST_CASE("Invalid camera ID throws", "[getCameraRotationMatrix]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraRotationMatrix(CameraBoardSocket::CAM_E, CameraBoardSocket::CAM_B), std::runtime_error);
}

TEST_CASE("No connection between two origins throws", "[getCameraRotationMatrix]") {
    auto handler = loadHandler();
    REQUIRE_THROWS_AS(handler.getCameraRotationMatrix(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_C), std::runtime_error);
}

TEST_CASE("Same camera rotation is identity", "[getCameraRotationMatrix]") {
    auto handler = loadHandler();
    auto R = handler.getCameraRotationMatrix(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_A);
    std::vector<std::vector<float>> I = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    REQUIRE(R == I);
}

TEST_CASE("Directly linked cameras rotation", "[getCameraRotationMatrix]") {
    auto handler = loadHandler();

    std::vector<std::vector<float>> Rset = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    std::vector<float> t = {0.0f, 0.0f, 0.0f};
    std::vector<float> specT = {0.0f, 0.0f, 0.0f};

    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, Rset, t, specT);

    auto R = handler.getCameraRotationMatrix(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C);

    REQUIRE(R.size() == 3);
    REQUIRE(R[0].size() == 3);

    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) REQUIRE(R[i][j] == Catch::Approx(Rset[i][j]).margin(1e-6));
}

TEST_CASE("Cyclic connection throws", "[getCameraRotationMatrix]") {
    auto handler = loadHandler();
    std::vector<std::vector<float>> R = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    std::vector<float> zeros = {0, 0, 0};

    handler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B, R, zeros, zeros);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::CAM_C, R, zeros, zeros);
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_A, R, zeros, zeros);

    REQUIRE_THROWS_AS(handler.getCameraRotationMatrix(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B), std::runtime_error);
}

TEST_CASE("Long chain rotation composition", "[getCameraRotationMatrix]") {
    auto handler = loadHandler();

    // Rotate 90° around Z at each step
    std::vector<std::vector<float>> Rz90 = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
    auto zeros3 = std::vector<float>{0, 0, 0};
    std::vector<float> t = {0, 0, 0};
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_D, Rz90, {1.0f, 0.0f, 0.0f}, {0, 0, 0});
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, Rz90, {0.0f, 2.0f, 0.0f}, {0, 0, 0});
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_C, CameraBoardSocket::CAM_B, Rz90, {0.0f, 0.0f, 3.0f}, {0, 0, 0});
    handler.setCameraExtrinsics(CameraBoardSocket::CAM_B, CameraBoardSocket::AUTO, Rz90, zeros3, zeros3);

    auto R = handler.getCameraRotationMatrix(CameraBoardSocket::CAM_A, CameraBoardSocket::CAM_B);

    // 3 × 90° rotations around Z should yield 270° (i.e. -90°)
    std::vector<std::vector<float>> expected = {{0, 1, 0}, {-1, 0, 0}, {0, 0, 1}};

    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) REQUIRE(R[i][j] == Catch::Approx(expected[i][j]).margin(1e-6));
}

TEST_CASE("Rotation matrix matches getCameraExtrinsics", "[getCameraRotationMatrix][getCameraExtrinsics]") {
    auto handler = loadHandler();

    std::vector<std::vector<float>> Rset = {{0.866f, -0.5f, 0.0f}, {0.5f, 0.866f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    std::vector<float> t = {0.0f, 0.0f, 0.0f};
    std::vector<float> specT = {0.0f, 0.0f, 0.0f};

    handler.setCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, Rset, t, specT);

    auto M = handler.getCameraExtrinsics(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C, false);
    auto R = handler.getCameraRotationMatrix(CameraBoardSocket::CAM_D, CameraBoardSocket::CAM_C);

    REQUIRE(R.size() == 3);
    REQUIRE(R[0].size() == 3);

    // Top-left 3x3 of extrinsics must match rotation matrix
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) REQUIRE(R[i][j] == Catch::Approx(M[i][j]).margin(1e-6));
}

TEST_CASE("EEPROM data default set fields are present", "[getEepromData]") {
    auto handler = loadHandler();
    auto eeprom = handler.getEepromData();

    // Version should be nonzero and reasonable
    REQUIRE(eeprom.version == 7);

    REQUIRE(eeprom.batchTime == 0);
    REQUIRE(eeprom.boardOptions == 0);
    REQUIRE(eeprom.stereoEnableDistortionCorrection == false);
    REQUIRE(eeprom.stereoUseSpecTranslation == true);
}

TEST_CASE("EEPROM data round-trip preserves basic fields", "[getEepromData]") {
    auto handler = loadHandler();

    // --- Create dummy EEPROM data ---
    dai::EepromData data;
    data.version = 42;
    data.productName = "SuperCam";
    data.boardName = "MainBoard";
    data.boardRev = "R2.1";
    data.deviceName = "Unit001";
    data.batchTime = 1717171717;
    data.boardOptions = 0xCAFEBABE;
    data.stereoUseSpecTranslation = false;
    data.stereoEnableDistortionCorrection = true;
    data.verticalCameraSocket = dai::CameraBoardSocket::CAM_B;

    // --- Fill camera info for CAM_A and CAM_B ---
    dai::CameraInfo camA;
    camA.width = 1280;
    camA.height = 800;
    camA.lensPosition = 150;
    camA.specHfovDeg = 70.5f;
    camA.cameraType = dai::CameraModel::Perspective;
    camA.intrinsicMatrix = {{1000.0f, 0.0f, 640.0f}, {0.0f, 1000.0f, 400.0f}, {0.0f, 0.0f, 1.0f}};
    camA.distortionCoeff = {0.1f, -0.01f, 0.0f, 0.0f, 0.001f};
    camA.extrinsics.rotationMatrix = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    camA.extrinsics.translation = {0.0f, 0.0f, 0.0f};

    dai::CameraInfo camB = camA;
    camB.width = 1920;
    camB.height = 1080;
    camB.lensPosition = 200;
    camB.specHfovDeg = 60.0f;
    camB.extrinsics.translation = {2.5f, 0.0f, 0.0f};

    data.cameraData[dai::CameraBoardSocket::CAM_A] = camA;
    data.cameraData[dai::CameraBoardSocket::CAM_B] = camB;

    // --- Stereo rectification ---
    data.stereoRectificationData.leftCameraSocket = dai::CameraBoardSocket::CAM_A;
    data.stereoRectificationData.rightCameraSocket = dai::CameraBoardSocket::CAM_B;
    data.stereoRectificationData.rectifiedRotationLeft = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    data.stereoRectificationData.rectifiedRotationRight = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

    // --- Store to handler and retrieve ---
    auto newHandler = dai::CalibrationHandler(data);
    auto loaded = newHandler.getEepromData();

    // --- Verify simple fields ---
    REQUIRE(loaded.version == data.version);
    REQUIRE(loaded.productName == data.productName);
    REQUIRE(loaded.boardName == data.boardName);
    REQUIRE(loaded.deviceName == data.deviceName);
    REQUIRE(loaded.boardOptions == data.boardOptions);
    REQUIRE(loaded.stereoUseSpecTranslation == data.stereoUseSpecTranslation);
    REQUIRE(loaded.stereoEnableDistortionCorrection == data.stereoEnableDistortionCorrection);
    REQUIRE(loaded.verticalCameraSocket == data.verticalCameraSocket);

    // --- Verify camera data ---
    REQUIRE(loaded.cameraData.size() == 2);
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_A).width == camA.width);
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_A).height == camA.height);
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_A).lensPosition == camA.lensPosition);
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_B).lensPosition == camB.lensPosition);
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_A).specHfovDeg == camA.specHfovDeg);
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_B).specHfovDeg == camB.specHfovDeg);

    // --- Verify stereo rectification ---
    for(int i = 0; i < 3; ++i)
        for(int j = 0; j < 3; ++j) {
            REQUIRE(loaded.stereoRectificationData.rectifiedRotationLeft[i][j]
                    == Catch::Approx(data.stereoRectificationData.rectifiedRotationLeft[i][j]).margin(1e-6));
            REQUIRE(loaded.stereoRectificationData.rectifiedRotationRight[i][j]
                    == Catch::Approx(data.stereoRectificationData.rectifiedRotationRight[i][j]).margin(1e-6));
        }
}

TEST_CASE("EEPROM data stereo flags consistency", "[getEepromData]") {
    auto handler = loadHandler();

    dai::EepromData data;
    data.stereoUseSpecTranslation = true;
    data.stereoEnableDistortionCorrection = false;

    auto newHandler1 = dai::CalibrationHandler(data);
    auto loaded = newHandler1.getEepromData();

    REQUIRE(loaded.stereoUseSpecTranslation == true);
    REQUIRE(loaded.stereoEnableDistortionCorrection == false);

    // Flip flags and verify again
    data.stereoUseSpecTranslation = false;
    data.stereoEnableDistortionCorrection = true;

    auto newHandler2 = dai::CalibrationHandler(data);
    loaded = newHandler2.getEepromData();

    REQUIRE(loaded.stereoUseSpecTranslation == false);
    REQUIRE(loaded.stereoEnableDistortionCorrection == true);
}

TEST_CASE("EEPROM cameraData is replaced correctly when constructing CalibrationHandler", "[getEepromData]") {
    // Load some real eeprom data to get correct board config
    auto baseHandler = loadHandler();
    auto baseData = baseHandler.getEepromData();

    // Create minimal custom cameraData (one entry)
    dai::EepromData modified = baseData;
    modified.cameraData.clear();

    dai::CameraInfo customCam;
    customCam.width = 640;
    customCam.height = 480;

    modified.cameraData[dai::CameraBoardSocket::CAM_C] = customCam;

    // Create new handler based on modified EEPROM
    dai::CalibrationHandler newHandler(modified);
    auto loaded = newHandler.getEepromData();

    // Check that our custom entry was preserved
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_C).width == 640);
    REQUIRE(loaded.cameraData.at(dai::CameraBoardSocket::CAM_C).height == 480);

    // Check that all camera sockets present in the board config still exist
    // (i.e., CalibrationHandler has filled missing ones)
    REQUIRE(loaded.cameraData.size() >= 1);
}

TEST_CASE("Stereo rectification matrices are preserved when passed to CalibrationHandler", "[rectification]") {
    dai::EepromData data;

    // Construct custom left and right rectification matrices
    std::vector<std::vector<float>> R_left = {{1.0f, 0.1f, 0.2f}, {0.0f, 1.0f, 0.3f}, {0.0f, 0.0f, 1.0f}};

    std::vector<std::vector<float>> R_right = {{0.99f, 0.05f, 0.0f}, {0.01f, 0.98f, 0.02f}, {0.0f, 0.0f, 1.0f}};

    data.stereoRectificationData.rectifiedRotationLeft = R_left;
    data.stereoRectificationData.rectifiedRotationRight = R_right;

    dai::CalibrationHandler handler(data);
    auto loaded = handler.getEepromData();

    // Check size
    REQUIRE(loaded.stereoRectificationData.rectifiedRotationLeft.size() == 3);
    REQUIRE(loaded.stereoRectificationData.rectifiedRotationLeft[0].size() == 3);
    REQUIRE(loaded.stereoRectificationData.rectifiedRotationRight.size() == 3);
    REQUIRE(loaded.stereoRectificationData.rectifiedRotationRight[0].size() == 3);

    // Check values match original
    REQUIRE(loaded.stereoRectificationData.rectifiedRotationLeft == R_left);
    REQUIRE(loaded.stereoRectificationData.rectifiedRotationRight == R_right);
}

TEST_CASE("CalibrationHandler throws when rectification rotation is missing", "[rectification]") {
    dai::EepromData data;  // No rectification values

    dai::CalibrationHandler handler(data);

    REQUIRE_THROWS_WITH(handler.getStereoLeftRectificationRotation(), Catch::Matchers::ContainsSubstring("Rectified Rotation Matrix Doesn't exist"));

    REQUIRE_THROWS_WITH(handler.getStereoRightRectificationRotation(), Catch::Matchers::ContainsSubstring("Rectified Rotation Matrix Doesn't exist"));
}

TEST_CASE("Stereo left and right rectification matrices are different when provided differently", "[rectification]") {
    dai::EepromData data;

    std::vector<std::vector<float>> R_left = {{1, 0.1f, 0}, {0, 1, 0.2f}, {0, 0, 1}};
    std::vector<std::vector<float>> R_right = {{0.99f, 0, 0}, {0.0f, 1, 0.1f}, {0, 0, 1}};

    data.stereoRectificationData.rectifiedRotationLeft = R_left;
    data.stereoRectificationData.rectifiedRotationRight = R_right;

    dai::CalibrationHandler handler(data);

    REQUIRE(handler.getStereoLeftRectificationRotation() != handler.getStereoRightRectificationRotation());
}
