#include <DynamicCalibration.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

#include "depthai/pipeline/node/DynamicCalibrationNode.hpp"

using Catch::Approx;

// -----------------------------------------------------------------------------
// Simple test doubles (no external mocking frameworks) + call counters
// -----------------------------------------------------------------------------

// Expose protected getProperties() for assertions
class TestDynamicCalibrationNode : public dai::node::DynamicCalibration {
   public:
    using dai::node::DynamicCalibration::DynamicCalibration;  // inherit ctors
    ~TestDynamicCalibrationNode() override = default;

    Properties& props() {
        return getProperties();
    }
};

// A configurable stub that returns pre-set results, captures args, and counts calls.
class StubDclDynamicCalibration : public dcl::DynamicCalibration {
   public:
    // --- knobs you can set from tests ---
    dcl::Result<dcl::CalibrationDifference> nextCheckCalibrationResult = dcl::Result<dcl::CalibrationDifference>::error(-999);
    dcl::Result<dcl::StereoCalibrationResult> nextFindNewCalibrationResult = dcl::Result<dcl::StereoCalibrationResult>::error(-999);
    dcl::Result<void> nextLoadStereoImagePairResult = dcl::Result<void>::ok();
    dcl::Result<dcl::CoverageData> nextComputeCoverageResult = dcl::Result<dcl::CoverageData>::error(-999);

    // --- captured args for assertions ---
    dcl::PerformanceMode lastPerfModeCheckCalibration = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dcl::PerformanceMode lastPerfModeFindNewCalibration = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dcl::PerformanceMode lastPerfModeComputeCoverage = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;

    dcl::ImageData lastImageA;
    dcl::ImageData lastImageB;

    // --- call counters (mutable so we can increment in const methods) ---
    mutable int checkCalibrationCalls = 0;
    mutable int findNewCalibrationCalls = 0;
    mutable int loadStereoImagePairCalls = 0;
    mutable int computeCoverageCalls = 0;

    // --- dcl::DynamicCalibration API ---
    dcl::Result<dcl::CalibrationDifference> checkCalibration(const std::shared_ptr<const dcl::Device>,
                                                             const dcl::socket_t,
                                                             const dcl::socket_t,
                                                             const dcl::PerformanceMode mode) const override {
        ++checkCalibrationCalls;
        const_cast<StubDclDynamicCalibration*>(this)->lastPerfModeCheckCalibration = mode;
        return nextCheckCalibrationResult;
    }

    dcl::Result<dcl::StereoCalibrationResult> findNewCalibration(const std::shared_ptr<const dcl::Device>,
                                                                 const dcl::socket_t,
                                                                 const dcl::socket_t,
                                                                 const dcl::PerformanceMode mode) const override {
        ++findNewCalibrationCalls;
        const_cast<StubDclDynamicCalibration*>(this)->lastPerfModeFindNewCalibration = mode;
        return nextFindNewCalibrationResult;
    }

    dcl::Result<void> loadStereoImagePair(const dcl::ImageData& imageA,
                                          const dcl::ImageData& imageB,
                                          const dcl::mxid_t&,
                                          const dcl::socket_t,
                                          const dcl::socket_t,
                                          const dcl::timestamp_t) override {
        ++loadStereoImagePairCalls;
        lastImageA = imageA;
        lastImageB = imageB;
        return nextLoadStereoImagePairResult;
    }

    dcl::Result<dcl::CoverageData> computeCoverage(const std::shared_ptr<const dcl::CameraSensorHandle>,
                                                   const std::shared_ptr<const dcl::CameraSensorHandle>,
                                                   const dcl::PerformanceMode mode) const override {
        ++computeCoverageCalls;
        const_cast<StubDclDynamicCalibration*>(this)->lastPerfModeComputeCoverage = mode;
        return nextComputeCoverageResult;
    }
};

// -----------------------------------------------------------------------------
// Shared data
// -----------------------------------------------------------------------------
namespace {
constexpr int WIDTH = 640;
constexpr int HEIGHT = 480;

std::vector<std::vector<float>> testIntrinsics{{600.0f, 0.0f, 320.0f}, {0.0f, 600.0f, 240.0f}, {0.0f, 0.0f, 1.0f}};
std::vector<float> testDistortion{0.1f, -0.05f, 0.001f, 0.0005f};
std::vector<std::vector<float>> testRotation{{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
std::vector<float> testTranslation{10.0f, 5.0f, 2.5f};
}  // namespace

// -----------------------------------------------------------------------------
// Tests that don't need DCL
// -----------------------------------------------------------------------------
TEST_CASE("Set & get runOnHost flag", "[DynamicCalibration]") {
    TestDynamicCalibrationNode dynCalib;  // stack ok; no logger use
    dynCalib.setRunOnHost(true);
    REQUIRE(dynCalib.runOnHost());

    dynCalib.setRunOnHost(false);
    REQUIRE_FALSE(dynCalib.runOnHost());
}

TEST_CASE("Set performance mode in properties", "[DynamicCalibration]") {
    TestDynamicCalibrationNode dynCalib;  // stack ok; no logger use
    dai::DynamicCalibrationConfig cfg;
    cfg.performanceMode = dcl::PerformanceMode::SKIP_CHECKS;
    dynCalib.setInitialConfig(cfg);

    REQUIRE(dynCalib.props().initialConfig.performanceMode == dcl::PerformanceMode::SKIP_CHECKS);
}

// -----------------------------------------------------------------------------
// Tests that exercise paths via a simple stub implementation
// -----------------------------------------------------------------------------
TEST_CASE("runQualityCheck() uses forced SKIP_CHECKS and returns failure", "[DynamicCalibration]") {
    dai::Pipeline pipeline(false);

    auto stub = std::make_unique<StubDclDynamicCalibration>();
    stub->nextCheckCalibrationResult = dcl::Result<dcl::CalibrationDifference>::error(1);
    auto* stubRaw = stub.get();

    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>(std::move(stub));

    const bool force = true;
    auto err = dynCalib->runQualityCheck(force);

    REQUIRE(err == dai::node::DynamicCalibration::ErrorCode::QUALITY_CHECK_FAILED);
    REQUIRE(stubRaw->lastPerfModeCheckCalibration == dcl::PerformanceMode::SKIP_CHECKS);
    REQUIRE(stubRaw->checkCalibrationCalls == 1);
}

TEST_CASE("runQualityCheck() uses config OPTIMIZE_PERFORMANCE and returns OK", "[DynamicCalibration]") {
    dai::Pipeline pipeline(false);

    auto stub = std::make_unique<StubDclDynamicCalibration>();
    dcl::CalibrationDifference diff{};
    stub->nextCheckCalibrationResult = dcl::Result<dcl::CalibrationDifference>::ok(diff);
    auto* stubRaw = stub.get();

    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>(std::move(stub));

    dai::DynamicCalibrationConfig cfg;
    cfg.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib->setInitialConfig(cfg);

    const bool force = false;
    auto err = dynCalib->runQualityCheck(force);
    REQUIRE(err == dai::node::DynamicCalibration::ErrorCode::OK);
    REQUIRE(stubRaw->lastPerfModeCheckCalibration == dcl::PerformanceMode::OPTIMIZE_PERFORMANCE);
    REQUIRE(stubRaw->checkCalibrationCalls == 1);
}

TEST_CASE("runCalibration() failure path", "[DynamicCalibration]") {
    dai::Pipeline pipeline(false);
    auto stub = std::make_unique<StubDclDynamicCalibration>();
    stub->nextFindNewCalibrationResult = dcl::Result<dcl::StereoCalibrationResult>::error(-1);
    auto* stubRaw = stub.get();
    auto dynCalib = pipeline.create<dai::node::DynamicCalibration>(std::move(stub));

    dai::CalibrationHandler ch;
    auto err = dynCalib->runCalibration(ch, /*force*/ true);
    REQUIRE(err == dai::node::DynamicCalibration::ErrorCode::CALIBRATION_FAILED);
    REQUIRE(stubRaw->findNewCalibrationCalls == 1);
}

TEST_CASE("runCalibration() success publishes result", "[DynamicCalibration]") {
    dai::Pipeline pl(false);
    auto stub = std::make_unique<StubDclDynamicCalibration>();
    auto* stubRaw = stub.get();
    auto dynCalib = pl.create<dai::node::DynamicCalibration>(std::move(stub));

    auto outQ = dynCalib->calibrationOutput.createOutputQueue();

    dynCalib->setWidth(1280);
    dynCalib->setHeight(800);

    const dcl::scalar_t zeroR[3] = {0., 0., 0.};
    const dcl::scalar_t zeroT[3] = {0., 0., 0.};
    const dcl::scalar_t r[3] = {1., 2., 3.};
    const dcl::scalar_t t[3] = {4., 5., 6.};
    const dcl::scalar_t K[9] = {1., 0., 1., 0., 1., 1., 0., 0., 1.};
    const dcl::scalar_t D[14] = {4., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.};

    auto hA = std::make_shared<dcl::CameraCalibrationHandle>(zeroR, zeroT, K, D);
    auto hB = std::make_shared<dcl::CameraCalibrationHandle>(r, t, K, D);
    std::pair<std::shared_ptr<dcl::CameraCalibrationHandle>, std::shared_ptr<dcl::CameraCalibrationHandle>> pairHB{hA, hB};

    dcl::StereoCalibrationResult cr{
        .newCalibration = pairHB, .currentCalibration = pairHB, .calibrationDifference = std::make_shared<dcl::CalibrationDifference>()};
    stubRaw->nextFindNewCalibrationResult = dcl::Result<dcl::StereoCalibrationResult>::ok(cr);

    dai::DynamicCalibrationConfig cfg;
    cfg.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib->setInitialConfig(cfg);

    dai::CalibrationHandler ch;
    std::vector<std::vector<float>> zeroRotationMat{{0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}, {0.f, 0.f, 0.f}};
    ch.setCameraExtrinsics(dai::CameraBoardSocket::CAM_B, dai::CameraBoardSocket::CAM_C, zeroRotationMat, {0.f, 0.f, 0.f}, {1.f, 1.f, 1.f});

    auto err = dynCalib->runCalibration(ch, /*force*/ false);
    REQUIRE(err == dai::node::DynamicCalibration::ErrorCode::OK);
    REQUIRE(stubRaw->findNewCalibrationCalls == 1);

    REQUIRE(outQ->getSize() == 1);
    auto returned = outQ->tryGet<dai::DynamicCalibrationResult>();
    REQUIRE(outQ->getSize() == 0);
    REQUIRE(returned);

    auto opt = returned->calibrationData;
    REQUIRE(opt.has_value());

    auto& newCal = opt->newCalibration;
    auto KiA = newCal.getCameraIntrinsics(dynCalib->getBorderSockerA(), dynCalib->getWidth(), dynCalib->getHeight());
    auto KiB = newCal.getCameraIntrinsics(dynCalib->getBorderSockerB(), dynCalib->getWidth(), dynCalib->getHeight());
    auto distA = newCal.getDistortionCoefficients(dynCalib->getBorderSockerA());
    auto distB = newCal.getDistortionCoefficients(dynCalib->getBorderSockerB());
    auto R = newCal.getCameraRotationMatrix(dynCalib->getBorderSockerA(), dynCalib->getBorderSockerB());
    auto T = newCal.getCameraTranslationVector(dynCalib->getBorderSockerA(), dynCalib->getBorderSockerB(), false);

    REQUIRE(KiA[0][0] == Approx(1.0f));
    REQUIRE(KiB[0][0] == Approx(1.0f));
    REQUIRE(KiA[1][0] == Approx(0.0f));
    REQUIRE(KiB[1][0] == Approx(0.0f));
    REQUIRE(distA[0] == Approx(4.0f));
    REQUIRE(distB[0] == Approx(4.0f));
    REQUIRE(distA[1] == Approx(2.0f));
    REQUIRE(distB[1] == Approx(2.0f));
    REQUIRE(R[0][0] != Approx(0.0f));
    REQUIRE(R[0][1] != Approx(0.0f));
    REQUIRE(T[1] == Approx(500.0f));
}

TEST_CASE("runLoadImage(): empty queue and missing images", "[DynamicCalibration]") {
    dai::Pipeline pipeline(false);
    SECTION("Empty queue") {
        auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
        auto err = dynCalib->runLoadImage();
        REQUIRE(err == dai::node::DynamicCalibration::ErrorCode::EMPTY_IMAGE_QUEUE);
    }
    SECTION("Missing left or right") {
        auto left = std::make_shared<dai::ImgFrame>();
        auto right = std::make_shared<dai::ImgFrame>();

        {
            auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
            auto group = std::make_shared<dai::MessageGroup>();
            dynCalib->syncInput.send(group);
            REQUIRE(dynCalib->runLoadImage() == dai::node::DynamicCalibration::ErrorCode::MISSING_IMAGE);
        }
        {
            auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
            auto group = std::make_shared<dai::MessageGroup>();
            group->add(dynCalib->rightInputName, right);
            dynCalib->syncInput.send(group);
            REQUIRE(dynCalib->runLoadImage() == dai::node::DynamicCalibration::ErrorCode::MISSING_IMAGE);
        }
        {
            auto dynCalib = pipeline.create<dai::node::DynamicCalibration>();
            auto group = std::make_shared<dai::MessageGroup>();
            group->add(dynCalib->leftInputName, left);
            dynCalib->syncInput.send(group);
            REQUIRE(dynCalib->runLoadImage() == dai::node::DynamicCalibration::ErrorCode::MISSING_IMAGE);
        }
    }
}

TEST_CASE("runLoadImage(): OK and forwards frames to DCL", "[DynamicCalibration]") {
    // Prepare frames
    uint8_t leftData[] = {1, 2, 3, 4};
    cv::Mat leftMat(2, 2, CV_8UC1, leftData);
    auto left = std::make_shared<dai::ImgFrame>();
    left->setWidth(2);
    left->setHeight(2);
    left->setCvFrame(leftMat, dai::ImgFrame::Type::GRAY8);

    uint8_t rightData[] = {5, 6, 7, 8};
    cv::Mat rightMat(2, 2, CV_8UC1, rightData);
    auto right = std::make_shared<dai::ImgFrame>();
    right->setWidth(2);
    right->setHeight(2);
    right->setSourceSize(2, 2);
    right->setCvFrame(rightMat, dai::ImgFrame::Type::GRAY8);

    dai::Pipeline pl(false);
    auto stub = std::make_unique<StubDclDynamicCalibration>();
    auto* stubRaw = stub.get();
    auto dynCalib = pl.create<dai::node::DynamicCalibration>(std::move(stub));

    auto group = std::make_shared<dai::MessageGroup>();
    group->add(dynCalib->rightInputName, right);
    group->add(dynCalib->leftInputName, left);
    dynCalib->syncInput.send(group);

    auto err = dynCalib->runLoadImage();
    REQUIRE(err == dai::node::DynamicCalibration::ErrorCode::OK);

    // Verify what the stub received and call count
    REQUIRE(stubRaw->loadStereoImagePairCalls == 1);

    REQUIRE(stubRaw->lastImageA.width == 2u);
    REQUIRE(stubRaw->lastImageA.height == 2u);
    REQUIRE(stubRaw->lastImageA.size() == 4u);
    REQUIRE(stubRaw->lastImageA.data[0] == 1);
    REQUIRE(stubRaw->lastImageA.data[1] == 2);
    REQUIRE(stubRaw->lastImageA.data[2] == 3);
    REQUIRE(stubRaw->lastImageA.data[3] == 4);

    REQUIRE(stubRaw->lastImageB.width == 2u);
    REQUIRE(stubRaw->lastImageB.height == 2u);
    REQUIRE(stubRaw->lastImageB.size() == 4u);
    REQUIRE(stubRaw->lastImageB.data[0] == 5);
    REQUIRE(stubRaw->lastImageB.data[1] == 6);
    REQUIRE(stubRaw->lastImageB.data[2] == 7);
    REQUIRE(stubRaw->lastImageB.data[3] == 8);
}

TEST_CASE("computeCoverage(): error throws and counts", "[DynamicCalibration]") {
    dai::Pipeline pl(false);
    auto stub = std::make_unique<StubDclDynamicCalibration>();
    stub->nextComputeCoverageResult = dcl::Result<dcl::CoverageData>::error(1);
    auto* stubRaw = stub.get();
    auto dynCalib = pl.create<dai::node::DynamicCalibration>(std::move(stub));

    dai::DynamicCalibrationConfig cfg;
    cfg.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dynCalib->setInitialConfig(cfg);

    REQUIRE_THROWS_AS(dynCalib->computeCoverage(), std::runtime_error);
    REQUIRE(stubRaw->lastPerfModeComputeCoverage == dcl::PerformanceMode::OPTIMIZE_PERFORMANCE);
    REQUIRE(stubRaw->computeCoverageCalls == 1);
}

TEST_CASE("computeCoverage(): success publishes coverage and counts", "[DynamicCalibration]") {
    dai::Pipeline pl(false);
    auto stub = std::make_unique<StubDclDynamicCalibration>();
    auto* stubRaw = stub.get();
    auto dyn = pl.create<dai::node::DynamicCalibration>(std::move(stub));

    dcl::CoverageData cov{};
    cov.coveragePerCellA = {{1.f, 2.f}, {3.f, 4.f}};
    cov.coveragePerCellB = {{5.f, 6.f}, {7.f, 8.f}};
    cov.meanCoverage = 4.2f;
    stubRaw->nextComputeCoverageResult = dcl::Result<dcl::CoverageData>::ok(cov);

    dai::DynamicCalibrationConfig cfg;
    cfg.performanceMode = dcl::PerformanceMode::OPTIMIZE_PERFORMANCE;
    dyn->setInitialConfig(cfg);

    auto outQ = dyn->coverageOutput.createOutputQueue();

    REQUIRE_NOTHROW(dyn->computeCoverage());
    REQUIRE(stubRaw->computeCoverageCalls == 1);

    REQUIRE(outQ->getSize() == 1);
    auto returned = outQ->tryGet<dai::CoverageData>();
    REQUIRE(outQ->getSize() == 0);
    REQUIRE(returned);

    REQUIRE(returned->coveragePerCellA[0][0] == Approx(1.0f));
    REQUIRE(returned->coveragePerCellA[0][1] == Approx(2.0f));
    REQUIRE(returned->coveragePerCellA[1][0] == Approx(3.0f));
    REQUIRE(returned->coveragePerCellA[1][1] == Approx(4.0f));
    REQUIRE(returned->coveragePerCellB[0][0] == Approx(5.0f));
    REQUIRE(returned->coveragePerCellB[0][1] == Approx(6.0f));
    REQUIRE(returned->coveragePerCellB[1][0] == Approx(7.0f));
    REQUIRE(returned->coveragePerCellB[1][1] == Approx(8.0f));
}

TEST_CASE("DoWork: enqueue command (smoke)", "[DynamicCalibration]") {
    dai::Pipeline pl(false);
    auto stub = std::make_unique<StubDclDynamicCalibration>();
    auto dyn = pl.create<dai::node::DynamicCalibration>(std::move(stub));

    auto cmd = std::make_shared<dai::StartCalibrationCommand>();
    auto inQ = dyn->inputControl.createInputQueue();
    inQ->send(cmd);

    SUCCEED();
}

// A lighter-weight integration that exercises a few codepaths in doWork
TEST_CASE("DoWork: processes load + coverage + quality-check paths and counts", "[DynamicCalibration]") {
    using namespace std::chrono;

    dai::Pipeline pl(false);
    auto stub = std::make_unique<StubDclDynamicCalibration>();
    auto* stubRaw = stub.get();
    auto dyn = pl.create<dai::node::DynamicCalibration>(std::move(stub));

    dyn->setWidth(1280);
    dyn->setHeight(800);

    // force action by making last time old
    auto previousLoadingTime = steady_clock::now() - seconds(10);
    dyn->properties.initialConfig.calibrationPeriod = 1;

    // frames
    uint8_t leftData[] = {1, 2, 3, 4};
    cv::Mat leftMat(2, 2, CV_8UC1, leftData);
    auto left = std::make_shared<dai::ImgFrame>();
    left->setWidth(2);
    left->setHeight(2);
    left->setCvFrame(leftMat, dai::ImgFrame::Type::GRAY8);

    uint8_t rightData[] = {5, 6, 7, 8};
    cv::Mat rightMat(2, 2, CV_8UC1, rightData);
    auto right = std::make_shared<dai::ImgFrame>();
    right->setWidth(2);
    right->setHeight(2);
    right->setSourceSize(2, 2);
    right->setCvFrame(rightMat, dai::ImgFrame::Type::GRAY8);

    // make coverage OK
    dcl::CoverageData cov{};
    stubRaw->nextComputeCoverageResult = dcl::Result<dcl::CoverageData>::ok(cov);

    // Load image path
    auto group = std::make_shared<dai::MessageGroup>();
    group->add(dyn->rightInputName, right);
    group->add(dyn->leftInputName, left);
    dyn->syncInput.send(group);

    auto loadCmd = std::make_shared<dai::LoadImageCommand>();
    dyn->inputControl.send(loadCmd);
    auto errLoad = dyn->doWork(previousLoadingTime);
    REQUIRE(errLoad == dai::node::DynamicCalibration::ErrorCode::OK);
    REQUIRE(stubRaw->loadStereoImagePairCalls == 1);
    REQUIRE(stubRaw->computeCoverageCalls == 1);  // during load or recalc path

    // force quality check with SKIP_CHECKS and success
    stubRaw->nextCheckCalibrationResult = dcl::Result<dcl::CalibrationDifference>::ok({});
    auto qcForce = std::make_shared<dai::CalibrationQualityCommand>(true);
    qcForce->force = true;
    dyn->inputControl.send(qcForce);
    auto errQc1 = dyn->doWork(previousLoadingTime);
    REQUIRE(errQc1 == dai::node::DynamicCalibration::ErrorCode::OK);
    REQUIRE(stubRaw->loadStereoImagePairCalls == 1);
    REQUIRE(stubRaw->checkCalibrationCalls == 1);  // forced + non-forced QC above

    // non-forced QC with OPTIMIZE_PERFORMANCE
    stubRaw->nextCheckCalibrationResult = dcl::Result<dcl::CalibrationDifference>::ok({});
    auto qc = std::make_shared<dai::CalibrationQualityCommand>(false, dcl::PerformanceMode::OPTIMIZE_PERFORMANCE);
    qc->force = false;
    dyn->inputControl.send(qc);
    auto errQc2 = dyn->doWork(previousLoadingTime);
    REQUIRE(errQc2 == dai::node::DynamicCalibration::ErrorCode::OK);
    REQUIRE(stubRaw->loadStereoImagePairCalls == 1);
    REQUIRE(stubRaw->checkCalibrationCalls == 2);  // forced + non-forced QC above

    // StartCalibration path where findNewCalibration fails
    stubRaw->nextFindNewCalibrationResult = dcl::Result<dcl::StereoCalibrationResult>::error(-1);

    // queue another image group so calibration path has data
    auto group2 = std::make_shared<dai::MessageGroup>();
    group2->add(dyn->rightInputName, right);
    group2->add(dyn->leftInputName, left);
    dyn->syncInput.send(group2);

    auto recalc = std::make_shared<dai::StartCalibrationCommand>(dcl::PerformanceMode::OPTIMIZE_PERFORMANCE);
    dyn->inputControl.send(recalc);

    // ensure coverage still returns something during the path
    stubRaw->nextComputeCoverageResult = dcl::Result<dcl::CoverageData>::ok({});
    stubRaw->nextFindNewCalibrationResult = dcl::Result<dcl::StereoCalibrationResult>::error(1);
    REQUIRE(stubRaw->findNewCalibrationCalls == 0);  // failed calibration path
    auto errCal = dyn->doWork(previousLoadingTime);
    REQUIRE(errCal == dai::node::DynamicCalibration::ErrorCode::CALIBRATION_FAILED);
    REQUIRE(stubRaw->computeCoverageCalls == 2);     // during load or recalc path
    REQUIRE(stubRaw->findNewCalibrationCalls == 1);  // failed calibration path
}
