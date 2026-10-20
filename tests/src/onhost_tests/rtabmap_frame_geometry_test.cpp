#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <filesystem>
#include <limits>
#include <thread>

#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/rtabmap/RTABMapSLAM.hpp"
#include "rtabmap/RTABMapConversions.hpp"
#include "rtabmap/core/CameraModel.h"
#include "rtabmap/core/DBDriver.h"

namespace {
dai::ImgFrame frame(float cx = 683.89563f, int width = 640, int height = 400) {
    dai::ImgFrame image;
    image.setWidth(width).setHeight(height);
    image.transformation = dai::ImgTransformation(width, height, {{{283.2423f, 0, cx}, {0, 283.2423f, 172.2848f}, {0, 0, 1}}});
    dai::Extrinsics extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {0, 0, 0}, dai::CameraBoardSocket::CAM_A);
    image.transformation.setExtrinsics(extrinsics);
    return image;
}

rtabmap::CameraModel model(const dai::ImgFrame& image,
                           const dai::ImgFrame& depth,
                           const rtabmap::Transform& referenceToBase = rtabmap::CameraModel::opticalRotation()) {
    return dai::getRTABMapFrameCameraModel(image, depth, referenceToBase);
}
}  // namespace

TEST_CASE("RTABMap consumes the current projection, including an off-image principal point", "[rtabmap][geometry]") {
    auto image = frame();
    const auto camera = model(image, image);
    REQUIRE(camera.fx() == Catch::Approx(283.2423));
    REQUIRE(camera.cx() == Catch::Approx(683.89563));
    REQUIRE(camera.cy() == Catch::Approx(172.2848));
    REQUIRE(camera.imageWidth() == 640);
    // Check a reconstructed ray, not just copied matrix elements.
    REQUIRE((320 - camera.cx()) * 10 / camera.fx() == Catch::Approx(-12.84750).margin(0.001));

    image.transformation.addCrop(40, 20, 560, 360);
    image.setWidth(560).setHeight(360);
    const auto cropped = model(image, image);
    REQUIRE(cropped.cx() == Catch::Approx(camera.cx() - 40));
    REQUIRE(cropped.cy() == Catch::Approx(camera.cy() - 20));
    REQUIRE(cropped.imageWidth() == 560);
}

TEST_CASE("RTABMap composes rectification relative to the odometry camera in meters", "[rtabmap][geometry]") {
    auto image = frame();
    // Explicitly configure common-reference -> tracked camera -> FLU.
    // The common reference is rotated 90 degrees and translated (1, 2, 3) meters.
    const rtabmap::Transform commonToCamera(0, 1, 0, -2, -1, 0, 0, 1, 0, 0, 1, -3);
    const auto referenceToBody = rtabmap::CameraModel::opticalRotation() * commonToCamera;
    const float c = std::cos(0.2f), s = std::sin(0.2f);
    // T_image_to_common = T_reference_to_common * T_image_to_reference.
    dai::Extrinsics imageExtrinsics({{0, -1, 0}, {c, 0, s}, {-s, 0, c}}, {1, 2.1f, 3}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::METER);
    image.transformation.setExtrinsics(imageExtrinsics);
    auto depth = image;
    auto depthExtrinsics = imageExtrinsics;
    depthExtrinsics.lengthUnit = dai::LengthUnit::CENTIMETER;
    depthExtrinsics.translation = {100, 210, 300};
    depth.transformation.setExtrinsics(depthExtrinsics);
    const auto camera = model(image, depth, referenceToBody);
    const rtabmap::Transform imageToReference(c, 0, s, 0.1f, 0, 1, 0, 0, -s, 0, c, 0);
    const auto expected = rtabmap::CameraModel::opticalRotation() * imageToReference;
    for(int i = 0; i < 12; ++i) {
        REQUIRE(camera.localTransform().data()[i] == Catch::Approx(expected.data()[i]).margin(1e-6));
    }
}

TEST_CASE("RTABMap accepts consistently decimated depth", "[rtabmap][geometry]") {
    auto image = frame();
    auto depth = frame();
    depth.transformation.addScale(0.5f, 0.5f);
    depth.setWidth(320).setHeight(200);
    REQUIRE(model(image, depth).imageWidth() == 640);
}

TEST_CASE("RTABMap rejects mismatched image and depth geometry", "[rtabmap][geometry]") {
    auto image = frame();
    auto depth = frame(320.4844f);
    REQUIRE_THROWS(model(image, depth));
    depth = frame();
    auto extrinsics = depth.transformation.getExtrinsics();
    extrinsics.translation.x = 1;
    depth.transformation.setExtrinsics(extrinsics);
    REQUIRE_THROWS(model(image, depth));
    depth = frame();
    depth.setWidth(320);
    REQUIRE_THROWS(model(image, depth));
}

TEST_CASE("RTABMap rejects unusable intrinsics and unrectified input", "[rtabmap][geometry]") {
    auto image = frame();
    auto k = image.transformation.getIntrinsicMatrix();
    SECTION("negative focal length from the alpha=1 regression") {
        k[0][0] = k[1][1] = -25.39046f;
    }
    SECTION("nonfinite focal") {
        k[0][0] = std::numeric_limits<float>::quiet_NaN();
    }
    SECTION("nonzero skew") {
        k[0][1] = 2;
    }
    SECTION("placeholder intrinsics") {
        k = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    }
    SECTION("lens distortion") {
        image.transformation.setDistortionCoefficients({0.1f, 0, 0, 0});
    }
    image.transformation.setIntrinsicMatrix(k);
    REQUIRE_THROWS(model(image, image));
}

TEST_CASE("RTABMap requires known, compatible extrinsic reference frames", "[rtabmap][geometry]") {
    auto image = frame();
    auto depth = frame();
    auto extrinsics = depth.transformation.getExtrinsics();
    SECTION("different target camera") {
        extrinsics.toCameraSocket = dai::CameraBoardSocket::CAM_C;
    }
    SECTION("different device") {
        extrinsics.toDeviceId = "other-device";
    }
    SECTION("unknown camera") {
        extrinsics.toCameraSocket = dai::CameraBoardSocket::AUTO;
    }
    SECTION("missing rotation") {
        extrinsics.rotationMatrix.clear();
    }
    SECTION("custom length unit") {
        extrinsics.lengthUnit = dai::LengthUnit::CUSTOM;
    }
    SECTION("unknown length unit") {
        extrinsics.lengthUnit = static_cast<dai::LengthUnit>(99);
    }
    SECTION("nonfinite translation") {
        extrinsics.translation.x = std::numeric_limits<float>::infinity();
    }
    depth.transformation.setExtrinsics(extrinsics);
    REQUIRE_THROWS(model(image, depth));
}

TEST_CASE("RTABMap host pipeline saves per-frame geometry without a calibration device", "[rtabmap][geometry]") {
    const auto path = std::filesystem::temp_directory_path()
                      / ("depthai-rtabmap-geometry-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".db");
    struct RemoveDatabase {
        std::filesystem::path path;
        ~RemoveDatabase() {
            std::error_code error;
            std::filesystem::remove(path, error);
        }
    } cleanup{path};
    const rtabmap::Transform referenceToBody(0, 0, 1, 0.4f, -1, 0, 0, -0.2f, 0, -1, 0, 0.1f);
    {
        dai::Pipeline pipeline(false);
        auto slam = pipeline.create<dai::node::RTABMapSLAM>();
        slam->setDatabasePath(path.string());
        slam->setSaveDatabaseOnClose(true);
        slam->setFreq(5);
        slam->setPublishGrid(false);
        slam->setPublishObstacleCloud(false);
        slam->setPublishGroundCloud(false);
        slam->setParams({{"RGBD/LinearUpdate", "0"},
                         {"RGBD/AngularUpdate", "0"},
                         {"RGBD/CreateOccupancyGrid", "true"},
                         {"Grid/3D", "true"},
                         {"Mem/STMSize", "100"},
                         {"Kp/DetectorStrategy", "6"}});
        auto rectQueue = slam->rect.createInputQueue();
        auto depthQueue = slam->depth.createInputQueue();
        dai::Transform configured{};
        configured.matrix[3][3] = 1;
        for(int row = 0; row < 3; ++row)
            for(int col = 0; col < 4; ++col) configured.matrix[row][col] = referenceToBody.data()[row * 4 + col];
        slam->setLocalTransform(std::make_shared<dai::TransformData>(configured));
        slam->setUseFrameMetadata(true);
        REQUIRE_FALSE(slam->inputs.has("reference"));
        auto odomQueue = slam->odom.createInputQueue();
        auto passthrough = slam->passthroughRect.createOutputQueue();
        cv::Mat texture(400, 640, CV_8UC1);
        cv::RNG rng(123);
        rng.fill(texture, cv::RNG::UNIFORM, 0, 255);
        const cv::Mat depthPixels(400, 640, CV_16UC1, cv::Scalar(2000));
        pipeline.start();
        for(int i = 0; i < 16; ++i) {
            const auto stamp = std::chrono::steady_clock::now();
            auto image = std::make_shared<dai::ImgFrame>(frame(i < 8 ? 683.89563f : 402.26504f));
            if(i >= 8) {
                const float c = std::cos(0.2f), s = std::sin(0.2f);
                image->transformation.setExtrinsics(
                    dai::Extrinsics({{c, 0, s}, {0, 1, 0}, {-s, 0, c}}, {0.1f, 0, 0}, dai::CameraBoardSocket::CAM_A, dai::LengthUnit::METER));
            }
            auto depth = std::make_shared<dai::ImgFrame>(*image);
            image->setCvFrame(texture, dai::ImgFrame::Type::GRAY8);
            depth->setCvFrame(depthPixels, dai::ImgFrame::Type::RAW16);
            for(const auto& input : {image, depth}) {
                input->setSequenceNum(i + 1);
                input->setTimestamp(stamp);
                input->setTimestampDevice(stamp);
            }
            odomQueue->send(std::make_shared<dai::TransformData>(0.01 * i, 0, 0, 0, 0, 0, 1));
            rectQueue->send(image);
            depthQueue->send(depth);
            bool timedOut = false;
            REQUIRE(passthrough->get<dai::ImgFrame>(std::chrono::seconds(3), timedOut) != nullptr);
            REQUIRE_FALSE(timedOut);
            // The existing node uses its first frame only to initialize. Supply
            // the second immediately, before the first periodic map update.
            if(i > 0) std::this_thread::sleep_for(std::chrono::milliseconds(150));
        }
        pipeline.stop();
    }
    std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
    REQUIRE(db->openConnection(path.string()));
    std::set<int> ids;
    db->getAllNodeIds(ids);
    REQUIRE(ids.size() >= 2);
    bool firstProjection = false, secondProjection = false;
    for(const auto id : ids) {
        rtabmap::SensorData data;
        db->getNodeData(id, data);
        REQUIRE(data.cameraModels().size() == 1);
        const auto& camera = data.cameraModels().front();
        firstProjection |= std::abs(camera.cx() - 683.89563) < 0.001;
        secondProjection |= std::abs(camera.cx() - 402.26504) < 0.001;
        REQUIRE(camera.fx() == Catch::Approx(283.2423));
        const float angle = std::abs(camera.cx() - 402.26504) < 0.001 ? 0.2f : 0.0f;
        const float tx = angle == 0 ? 0 : 0.1f;
        const rtabmap::Transform relative(std::cos(angle), 0, std::sin(angle), tx, 0, 1, 0, 0, -std::sin(angle), 0, std::cos(angle), 0);
        const auto expected = referenceToBody * relative;
        for(int i = 0; i < 12; ++i) {
            REQUIRE(camera.localTransform().data()[i] == Catch::Approx(expected.data()[i]).margin(1e-6));
        }
    }
    REQUIRE(firstProjection);
    REQUIRE(secondProjection);
    db->closeConnection(false);
}
