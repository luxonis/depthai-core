#include "depthai/pipeline/node/Deblur.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <limits>

#include "../../../device/rvc4/src/utilities/DeblurSync.hpp"
#include "../../../src/utility/DeblurValidation.hpp"
#include "../utility/DeblurFixture.hpp"
using namespace dai::gate::deblur;

TEST_CASE("Deblur improves known local blur and preserves static support", "[Deblur]") {
    const auto fixture = deblur_test::make();
    const auto config = deblur_test::config();
    const auto original = fixture.target->getData();
    const std::vector<std::uint8_t> snapshot(original.begin(), original.end());
    const auto field = prepare(*fixture.reference, *fixture.target, *fixture.packet, config);
    REQUIRE(field.kernels.size() == 2);
    const auto output = finish(field, filterCpu(field), config);
    double beforeError = 0, afterError = 0;
    int changed = 0, accepted = 0;
    for(int y = 0; y < output.rows; ++y)
        for(int x = 0; x < output.cols; ++x) {
            const int value = output.at<std::uint8_t>(y, x), source = fixture.blurred.at<std::uint8_t>(y, x);
            if(field.blend.at<float>(y, x) == 0) REQUIRE(value == source);
            if(field.blend.at<float>(y, x) > 0.6f) {
                const double truth = fixture.sharp.at<std::uint8_t>(y, x);
                beforeError += (source - truth) * (source - truth);
                afterError += (value - truth) * (value - truth);
                ++accepted;
            }
            REQUIRE(std::abs(value - source) <= config.maxCorrection + 1);
            changed += value != source;
        }
    INFO("before=" << beforeError << " after=" << afterError << " pixels=" << accepted);
    REQUIRE(accepted > 1000);
    REQUIRE(changed > 1000);
    REQUIRE(afterError < beforeError * 0.8);
    REQUIRE(std::equal(snapshot.begin(), snapshot.end(), fixture.target->getData().begin()));
}
TEST_CASE("Deblur kernels preserve DC and respect constant exposure scaling", "[Deblur]") {
    const auto config = deblur_test::config();
    for(const auto vector : std::vector<cv::Point2f>{{0, 0}, {4, 0}, {0, -3}, {2, 3}}) {
        REQUIRE(cv::sum(linePsf(vector))[0] == Catch::Approx(1).margin(1e-5));
        REQUIRE(cv::sum(inverseKernel(vector, config))[0] == Catch::Approx(1).margin(1e-5));
    }
    const auto fixture = deblur_test::make();
    auto shortExposure = config;
    shortExposure.exposureTimeUs = 1000;
    REQUIRE(prepare(*fixture.reference, *fixture.target, *fixture.packet, shortExposure).kernels.empty());
    shortExposure.exposureTimeUs = 0;
    REQUIRE(prepare(*fixture.reference, *fixture.target, *fixture.packet, shortExposure).kernels.empty());
    fixture.target->cam.exposureTimeUs = 20000;
    REQUIRE(prepare(*fixture.reference, *fixture.target, *fixture.packet, shortExposure).kernels.size() == 2);
    shortExposure.exposureTimeUs = 50000;
    REQUIRE(prepare(*fixture.reference, *fixture.target, *fixture.packet, shortExposure).kernels.empty());
}
TEST_CASE("Deblur rejects malformed or mismatched flow without touching images", "[Deblur]") {
    const auto config = deblur_test::config();
    for(int mode = 0; mode < 7; ++mode) {
        auto fixture = deblur_test::make();
        auto flow = fixture.packet->get<dai::NNData>("flow");
        if(mode == 0) fixture.packet->get<dai::Buffer>("target")->setSequenceNum(15);
        if(mode == 1) fixture.packet->get<dai::Buffer>("target")->setTimestampDevice(std::chrono::steady_clock::time_point{});
        if(mode == 2) flow->tensors[0].strides = {1, 1, 1};
        if(mode == 3) flow->setData(std::vector<std::uint8_t>(1));
        if(mode == 4) fixture.packet->group.erase("reference");
        if(mode == 5) {
            auto data = flow->getData();
            const float nan = std::numeric_limits<float>::quiet_NaN();
            const auto offset = flow->getTensorInfo("backward")->offset;
            for(std::size_t i = 0; i < fixture.blurred.total() * 2; ++i) std::memcpy(data.data() + offset + i * 4, &nan, 4);
        }
        if(mode == 6) {
            auto data = flow->getData();
            const auto offset = flow->getTensorInfo("confidence")->offset;
            std::fill(data.begin() + offset, data.begin() + offset + fixture.blurred.total(), 0);
        }
        INFO("mode=" << mode);
        const auto field = prepare(*fixture.reference, *fixture.target, *fixture.packet, config);
        REQUIRE(field.kernels.empty());
        REQUIRE(cv::norm(finish(field, filterCpu(field), config), fixture.blurred, cv::NORM_INF) == 0);
    }
}
TEST_CASE("Deblur synchronization is bounded and matches exact pairs", "[Deblur]") {
    const auto fixture = deblur_test::make();
    const auto now = Synchronizer::Clock::now();
    Synchronizer sync(4, std::chrono::milliseconds(10));
    sync.addImage(fixture.reference, now);
    REQUIRE(!sync.take(now).target);
    auto first = sync.take(now + std::chrono::milliseconds(10));
    REQUIRE(first.target == fixture.reference);
    REQUIRE(!first.flow);
    sync.addFlow(fixture.packet);
    sync.addImage(fixture.target, now);
    auto second = sync.take(now);
    REQUIRE(second.target == fixture.target);
    REQUIRE(second.reference == fixture.reference);
    REQUIRE(second.flow == fixture.packet);
    REQUIRE(!sync.take(now).target);
    Synchronizer mismatch(2, std::chrono::milliseconds(10));
    auto wrong = deblur_test::image(fixture.blurred, 14, 1041);
    mismatch.addImage(fixture.reference, now);
    mismatch.addImage(wrong, now);
    mismatch.addFlow(fixture.packet);
    REQUIRE(!mismatch.take(now).flow);
    REQUIRE(!mismatch.take(now).target);
    REQUIRE(!mismatch.take(now + std::chrono::milliseconds(10)).flow);
    Synchronizer pressure(2, std::chrono::milliseconds(1000));
    for(int i = 0; i < 20; ++i) {
        pressure.addImage(deblur_test::image(fixture.blurred, i, 1000 + i), now);
        auto ready = pressure.take(now);
        if(i) {
            REQUIRE(ready.target != nullptr);
            REQUIRE(ready.target->getSequenceNum() == i - 1);
            REQUIRE(!ready.flow);
        }
    }
}
TEST_CASE("Deblur validates configuration and packed image layout", "[Deblur]") {
    auto config = deblur_test::config();
    REQUIRE_NOTHROW(dai::utility::validateDeblurConfig(config));
    config.strength = std::numeric_limits<float>::quiet_NaN();
    REQUIRE_THROWS_AS(dai::utility::validateDeblurConfig(config), std::invalid_argument);
    config = deblur_test::config();
    config.kernelRadius = 100;
    REQUIRE_THROWS_AS(dai::utility::validateDeblurConfig(config), std::invalid_argument);
    const auto fixture = deblur_test::make();
    fixture.target->setType(dai::ImgFrame::Type::RAW8);
    REQUIRE_THROWS_AS(validateImage(*fixture.target), std::invalid_argument);
}

TEST_CASE("Deblur handles early, reordered, missing-reference and late packets", "[Deblur]") {
    const auto a = deblur_test::make();
    const auto b = deblur_test::make(256, 128, 20, 1100);
    const auto now = Synchronizer::Clock::now(), expired = now + std::chrono::milliseconds(10);
    Synchronizer sync(8, std::chrono::milliseconds(10));
    sync.addFlow(b.packet);
    sync.addFlow(a.packet);  // out-of-order packets before images
    sync.addImage(a.reference, now);
    sync.addImage(a.target, now);
    REQUIRE(sync.take(expired).target == a.reference);
    const auto ready = sync.take(expired);
    REQUIRE(ready.flow == a.packet);
    REQUIRE(ready.reference == a.reference);
    sync.addFlow(a.packet);  // late packet cannot produce another output
    REQUIRE(!sync.take(expired).target);
    sync.addImage(b.reference, now);
    sync.addImage(b.target, now);
    REQUIRE(sync.take(expired).target == b.reference);
    REQUIRE(sync.take(expired).flow == b.packet);
    Synchronizer missing(2, std::chrono::milliseconds(10));
    missing.addFlow(a.packet);
    missing.addImage(a.target, now);
    REQUIRE(!missing.take(now).target);
    const auto bypass = missing.take(expired);
    REQUIRE(bypass.target == a.target);
    REQUIRE(!bypass.flow);
    // Bound the packet queue: oldest packet is evicted, never reused later.
    Synchronizer eviction(2, std::chrono::milliseconds(10));
    eviction.addFlow(a.packet);
    eviction.addFlow(b.packet);
    eviction.addFlow(b.packet);
    eviction.addImage(a.reference, now);
    REQUIRE(eviction.take(expired).target == a.reference);
    eviction.addImage(a.target, now);
    REQUIRE(!eviction.take(now).target);
    REQUIRE(!eviction.take(expired).flow);
}

TEST_CASE("Deblur configuration survives wire serialization and cloning", "[Deblur]") {
    auto config = deblur_test::config();
    config.strength = 0.4f;
    config.regularization = 0.1f;
    config.maxGain = 2;
    config.minBlur = 1;
    config.maxBlur = 8;
    config.motionQuantum = 1;
    config.forwardBackwardThreshold = 2;
    config.photometricThreshold = 30;
    config.minTextureVariance = 4;
    config.minConfidence = 180;
    config.maxCorrection = 16;
    config.kernelRadius = 4;
    config.maxClusters = 16;
    config.maxWaitMs = 200;
    config.cacheSize = 6;
    std::vector<std::uint8_t> bytes;
    config.serialize(bytes, dai::SerializationType::LIBNOP);
    dai::DeblurProperties restored;
    dai::utility::deserialize(bytes, restored);
    REQUIRE(restored.strength == config.strength);
    REQUIRE(restored.regularization == config.regularization);
    REQUIRE(restored.maxGain == config.maxGain);
    REQUIRE(restored.minBlur == config.minBlur);
    REQUIRE(restored.maxBlur == config.maxBlur);
    REQUIRE(restored.motionQuantum == config.motionQuantum);
    REQUIRE(restored.forwardBackwardThreshold == config.forwardBackwardThreshold);
    REQUIRE(restored.photometricThreshold == config.photometricThreshold);
    REQUIRE(restored.minTextureVariance == config.minTextureVariance);
    REQUIRE(restored.minConfidence == config.minConfidence);
    REQUIRE(restored.maxCorrection == config.maxCorrection);
    REQUIRE(restored.kernelRadius == config.kernelRadius);
    REQUIRE(restored.maxClusters == config.maxClusters);
    REQUIRE(restored.exposureTimeUs == config.exposureTimeUs);
    REQUIRE(restored.maxWaitMs == config.maxWaitMs);
    REQUIRE(restored.cacheSize == config.cacheSize);
    REQUIRE(restored.useOpenCL == config.useOpenCL);
    const auto cloned = config.clone();
    REQUIRE(dynamic_cast<dai::DeblurProperties*>(cloned.get()) != nullptr);
    std::vector<std::uint8_t> clonedBytes;
    cloned->serialize(clonedBytes, dai::SerializationType::LIBNOP);
    REQUIRE(clonedBytes == bytes);
}
