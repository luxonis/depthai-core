#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/InputQueue.hpp"
namespace {

const std::string kImgDetectionsRoundtripScript = R"(
while True:
    src = node.inputs["in"].get()
    if src is None:
        break

    # Reuse source message to preserve existing payload allocation for segmask data.
    dst = src

    copiedDetections = []
    for det in src.detections:
        copied = ImgDetection()
        copied.label = det.label
        copied.labelName = det.labelName
        copied.confidence = det.confidence

        try:
            copied.setBoundingBox(det.getBoundingBox())
        except Exception:
            pass

        keypoints = det.getKeypoints()
        if len(keypoints) > 0:
            copied.setKeypoints(keypoints)
            edges = det.getEdges()
            if len(edges) > 0:
                copied.setEdges(edges)

        copiedDetections.append(copied)

    dst.detections = copiedDetections

    node.outputs["out"].send(dst)
    break
)";

const std::string kSpatialImgDetectionsRoundtripScript = R"(
while True:
    src = node.inputs["in"].get()
    if src is None:
        break

    # Reuse source message to preserve existing payload allocation for segmask data.
    dst = src

    copiedDetections = []
    for det in src.detections:
        copied = SpatialImgDetection()
        copied.label = det.label
        copied.labelName = det.labelName
        copied.confidence = det.confidence
        copied.spatialCoordinates = det.spatialCoordinates
        copied.boundingBoxMapping = det.boundingBoxMapping

        try:
            copied.setBoundingBox(det.getBoundingBox())
        except Exception:
            pass

        keypoints = det.getKeypoints()
        if len(keypoints) > 0:
            copied.setKeypoints(keypoints)
            edges = det.getEdges()
            if len(edges) > 0:
                copied.setEdges(edges)

        copiedDetections.append(copied)

    dst.detections = copiedDetections

    node.outputs["out"].send(dst)
    break
)";

void runScriptBindingsSmoke(const std::string& scriptText) {
    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(scriptText);

    auto in = script->inputs["log"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.start();
    in->send(std::make_shared<dai::Buffer>());

    bool hasTimedOut = false;
    auto output = out->get<dai::Buffer>(std::chrono::seconds(1), hasTimedOut);
    REQUIRE_FALSE(hasTimedOut);
    REQUIRE(output != nullptr);
}

template <typename T>
std::shared_ptr<T> runTypedScriptRoundtrip(const std::shared_ptr<T>& input, const std::string& scriptText) {
    dai::Pipeline pipeline;
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(scriptText);

    auto in = script->inputs["in"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.start();
    in->send(input);

    bool hasTimedOut = false;
    auto output = out->get<T>(std::chrono::seconds(1), hasTimedOut);
    REQUIRE_FALSE(hasTimedOut);
    REQUIRE(output != nullptr);
    return output;
}

void requirePoint2fEqual(const dai::Point2f& expected, const dai::Point2f& actual) {
    REQUIRE(actual.x == Catch::Approx(expected.x));
    REQUIRE(actual.y == Catch::Approx(expected.y));
    REQUIRE(actual.normalized == expected.normalized);
    REQUIRE(actual.hasNormalized == expected.hasNormalized);
}

void requirePoint3fEqual(const dai::Point3f& expected, const dai::Point3f& actual) {
    REQUIRE(actual.x == Catch::Approx(expected.x));
    REQUIRE(actual.y == Catch::Approx(expected.y));
    REQUIRE(actual.z == Catch::Approx(expected.z));
}

void requireSize2fEqual(const dai::Size2f& expected, const dai::Size2f& actual) {
    REQUIRE(actual.width == Catch::Approx(expected.width));
    REQUIRE(actual.height == Catch::Approx(expected.height));
    REQUIRE(actual.normalized == expected.normalized);
    REQUIRE(actual.hasNormalized == expected.hasNormalized);
}

void requireRectEqual(const dai::Rect& expected, const dai::Rect& actual) {
    REQUIRE(actual.x == Catch::Approx(expected.x));
    REQUIRE(actual.y == Catch::Approx(expected.y));
    REQUIRE(actual.width == Catch::Approx(expected.width));
    REQUIRE(actual.height == Catch::Approx(expected.height));
    REQUIRE(actual.normalized == expected.normalized);
    REQUIRE(actual.hasNormalized == expected.hasNormalized);
}

void requireRotatedRectEqual(const dai::RotatedRect& expected, const dai::RotatedRect& actual) {
    requirePoint2fEqual(expected.center, actual.center);
    requireSize2fEqual(expected.size, actual.size);
    REQUIRE(actual.angle == Catch::Approx(expected.angle));
}

std::optional<dai::RotatedRect> maybeBoundingBox(const dai::ImgDetection& detection) {
    try {
        return detection.getBoundingBox();
    } catch(...) {
        return std::nullopt;
    }
}

std::optional<dai::RotatedRect> maybeBoundingBox(const dai::SpatialImgDetection& detection) {
    try {
        return detection.getBoundingBox();
    } catch(...) {
        return std::nullopt;
    }
}

void requireTransformEqual(const std::optional<dai::ImgTransformation>& expected, const std::optional<dai::ImgTransformation>& actual) {
    REQUIRE(actual.has_value() == expected.has_value());
    if(!expected.has_value()) return;

    REQUIRE(actual->getSize() == expected->getSize());
    REQUIRE(actual->getSourceSize() == expected->getSourceSize());
    REQUIRE(actual->getMatrix() == expected->getMatrix());
    REQUIRE(actual->getMatrixInv() == expected->getMatrixInv());
    REQUIRE(actual->getSourceIntrinsicMatrix() == expected->getSourceIntrinsicMatrix());
    REQUIRE(actual->getSourceIntrinsicMatrixInv() == expected->getSourceIntrinsicMatrixInv());
    REQUIRE(actual->getDistortionModel() == expected->getDistortionModel());
}

void requireKeypointEqual(const dai::Keypoint& expected, const dai::Keypoint& actual) {
    requirePoint3fEqual(expected.imageCoordinates, actual.imageCoordinates);
    REQUIRE(actual.confidence == Catch::Approx(expected.confidence));
    REQUIRE(actual.label == expected.label);
    REQUIRE(actual.labelName == expected.labelName);
}

void requireSpatialKeypointEqual(const dai::SpatialKeypoint& expected, const dai::SpatialKeypoint& actual) {
    requirePoint3fEqual(expected.imageCoordinates, actual.imageCoordinates);
    requirePoint3fEqual(expected.spatialCoordinates, actual.spatialCoordinates);
    REQUIRE(actual.confidence == Catch::Approx(expected.confidence));
    REQUIRE(actual.label == expected.label);
    REQUIRE(actual.labelName == expected.labelName);
}

void requireImgDetectionEqual(const dai::ImgDetection& expected, const dai::ImgDetection& actual) {
    REQUIRE(actual.label == expected.label);
    REQUIRE(actual.labelName == expected.labelName);
    REQUIRE(actual.confidence == Catch::Approx(expected.confidence));
    REQUIRE(actual.xmin == Catch::Approx(expected.xmin));
    REQUIRE(actual.ymin == Catch::Approx(expected.ymin));
    REQUIRE(actual.xmax == Catch::Approx(expected.xmax));
    REQUIRE(actual.ymax == Catch::Approx(expected.ymax));

    auto expectedBBox = maybeBoundingBox(expected);
    auto actualBBox = maybeBoundingBox(actual);
    REQUIRE(actualBBox.has_value() == expectedBBox.has_value());
    if(expectedBBox && actualBBox) {
        requireRotatedRectEqual(*expectedBBox, *actualBBox);
    }

    const auto expectedKeypoints = expected.getKeypoints();
    const auto actualKeypoints = actual.getKeypoints();
    REQUIRE(actualKeypoints.size() == expectedKeypoints.size());
    for(std::size_t i = 0; i < expectedKeypoints.size(); i++) {
        requireKeypointEqual(expectedKeypoints.at(i), actualKeypoints.at(i));
    }

    REQUIRE(actual.getEdges() == expected.getEdges());
}

void requireSpatialMappingEqual(const dai::SpatialLocationCalculatorConfigData& expected, const dai::SpatialLocationCalculatorConfigData& actual) {
    requireRectEqual(expected.roi, actual.roi);
    REQUIRE(actual.depthThresholds.lowerThreshold == expected.depthThresholds.lowerThreshold);
    REQUIRE(actual.depthThresholds.upperThreshold == expected.depthThresholds.upperThreshold);
    REQUIRE(actual.calculationAlgorithm == expected.calculationAlgorithm);
    REQUIRE(actual.stepSize == expected.stepSize);
}

void requireSpatialImgDetectionEqual(const dai::SpatialImgDetection& expected, const dai::SpatialImgDetection& actual) {
    REQUIRE(actual.label == expected.label);
    REQUIRE(actual.labelName == expected.labelName);
    REQUIRE(actual.confidence == Catch::Approx(expected.confidence));
    REQUIRE(actual.xmin == Catch::Approx(expected.xmin));
    REQUIRE(actual.ymin == Catch::Approx(expected.ymin));
    REQUIRE(actual.xmax == Catch::Approx(expected.xmax));
    REQUIRE(actual.ymax == Catch::Approx(expected.ymax));

    auto expectedBBox = maybeBoundingBox(expected);
    auto actualBBox = maybeBoundingBox(actual);
    REQUIRE(actualBBox.has_value() == expectedBBox.has_value());
    if(expectedBBox && actualBBox) {
        requireRotatedRectEqual(*expectedBBox, *actualBBox);
    }

    requirePoint3fEqual(expected.spatialCoordinates, actual.spatialCoordinates);
    requireSpatialMappingEqual(expected.boundingBoxMapping, actual.boundingBoxMapping);

    const auto expectedKeypoints = expected.getKeypoints();
    const auto actualKeypoints = actual.getKeypoints();
    REQUIRE(actualKeypoints.size() == expectedKeypoints.size());
    for(std::size_t i = 0; i < expectedKeypoints.size(); i++) {
        requireSpatialKeypointEqual(expectedKeypoints.at(i), actualKeypoints.at(i));
    }

    REQUIRE(actual.getEdges() == expected.getEdges());
}

void requireImgDetectionsEqual(const dai::ImgDetections& expected, const dai::ImgDetections& actual) {
    REQUIRE(actual.getSequenceNum() == expected.getSequenceNum());
    REQUIRE(actual.getSegmentationMaskWidth() == expected.getSegmentationMaskWidth());
    REQUIRE(actual.getSegmentationMaskHeight() == expected.getSegmentationMaskHeight());
    REQUIRE(actual.getMaskData() == expected.getMaskData());
    requireTransformEqual(expected.transformation, actual.transformation);

    REQUIRE(actual.detections.size() == expected.detections.size());
    for(std::size_t i = 0; i < expected.detections.size(); i++) {
        requireImgDetectionEqual(expected.detections.at(i), actual.detections.at(i));
    }
}

void requireSpatialImgDetectionsEqual(const dai::SpatialImgDetections& expected, const dai::SpatialImgDetections& actual) {
    REQUIRE(actual.getSequenceNum() == expected.getSequenceNum());
    REQUIRE(actual.getSegmentationMaskWidth() == expected.getSegmentationMaskWidth());
    REQUIRE(actual.getSegmentationMaskHeight() == expected.getSegmentationMaskHeight());
    REQUIRE(actual.getMaskData() == expected.getMaskData());
    requireTransformEqual(expected.transformation, actual.transformation);

    REQUIRE(actual.detections.size() == expected.detections.size());
    for(std::size_t i = 0; i < expected.detections.size(); i++) {
        requireSpatialImgDetectionEqual(expected.detections.at(i), actual.detections.at(i));
    }
}

void setCommonMessageFields(dai::ImgDetections& msg, int seqNo, int timestampMs) {
    msg.setSequenceNum(seqNo);
    msg.setTimestamp(std::chrono::steady_clock::now() + std::chrono::milliseconds{timestampMs});
    dai::ImgTransformation transform(640, 400);
    transform.addCrop(16, 12, 320, 200).addScale(0.7F, 0.9F);
    msg.transformation = transform;
}

void setCommonMessageFields(dai::SpatialImgDetections& msg, int seqNo, int timestampMs) {
    msg.setSequenceNum(seqNo);
    msg.setTimestamp(std::chrono::steady_clock::now() + std::chrono::milliseconds{timestampMs});
    dai::ImgTransformation transform(640, 400);
    transform.addCrop(10, 8, 300, 180).addScale(0.8F, 0.75F);
    msg.transformation = transform;
}

}  // namespace

TEST_CASE("Old API Test") {
    // Create pipeline
    dai::Pipeline pipeline;

    // Setup script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    while True:
        inMessage = node.io["log"].get()
        if inMessage is None:
            break
        message = Buffer(10)
        node.io["out"].send(message)
    )");

    auto in = script->inputs["log"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.start();
    // Send message and wait for response
    in->send(std::make_shared<dai::Buffer>());
    auto output = out->get<dai::Buffer>();

    REQUIRE(output != nullptr);
}

TEST_CASE("New API Test") {
    // Create pipeline
    dai::Pipeline pipeline;

    // Setup script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    while True:
        inMessage = node.inputs["log"].get()
        if inMessage is None:
            break
        message = Buffer(10)
        node.outputs["out"].send(message)
    )");

    auto in = script->inputs["log"].createInputQueue();
    auto out = script->outputs["out"].createOutputQueue();

    pipeline.start();
    // Send message and wait for response
    in->send(std::make_shared<dai::Buffer>());
    auto output = out->get<dai::Buffer>();

    REQUIRE(output != nullptr);
}

TEST_CASE("Missing script exception test") {
    // Create pipeline
    dai::Pipeline pipeline;

    // Setup script node
    auto script = pipeline.create<dai::node::Script>();

    REQUIRE_THROWS_AS(pipeline.build(), std::runtime_error);
}

TEST_CASE("Script bindings smoke: keypoint and spatial detection types") {
    runScriptBindingsSmoke(R"(
while True:
    inMessage = node.inputs["log"].get()
    if inMessage is None:
        break

    keypoint = Keypoint(Point2f(1.0, 2.0), 0.9, 1, "kp")
    keypoints = KeypointsList()
    keypoints.setKeypoints([keypoint])
    if len(keypoints.getKeypoints()) != 1:
        raise RuntimeError("KeypointsList keypoints count mismatch")

    spatialKeypoint = SpatialKeypoint(Point2f(3.0, 4.0), Point3f(10.0, 20.0, 30.0), 0.8, 2, "skp")
    spatialKeypoints = SpatialKeypointsList([spatialKeypoint])
    if len(spatialKeypoints.getKeypoints()) != 1:
        raise RuntimeError("SpatialKeypointsList keypoints count mismatch")

    spatialKeypoints.setSpatialCoordinates([Point3f(100.0, 200.0, 300.0)])
    if len(spatialKeypoints.getSpatialCoordinates()) != 1:
        raise RuntimeError("SpatialKeypointsList spatial coordinates count mismatch")

    det = SpatialImgDetection()
    det.setOuterBoundingBox(0.1, 0.2, 0.4, 0.5)
    det.setKeypoints(spatialKeypoints)
    det.setSpatialCoordinate(Point3f(100.0, 200.0, 300.0))
    if len(det.getKeypoints()) != 1:
        raise RuntimeError("SpatialImgDetection keypoints count mismatch")

    dets = SpatialImgDetections()
    dets.detections = [det]
    if len(dets.detections) != 1:
        raise RuntimeError("SpatialImgDetections detections count mismatch")

    node.outputs["out"].send(Buffer(1))
    break
)");
}

TEST_CASE("ImgDetections roundtrip in Script node", "[script][imgdetections]") {
    SECTION("basic") {
        auto input = std::make_shared<dai::ImgDetections>();
        setCommonMessageFields(*input, 101, 1234);

        dai::ImgDetection a;
        a.label = 1;
        a.labelName = "person";
        a.confidence = 0.91F;
        a.xmin = 0.12F;
        a.ymin = 0.15F;
        a.xmax = 0.46F;
        a.ymax = 0.67F;

        dai::ImgDetection b;
        b.label = 2;
        b.labelName = "car";
        b.confidence = 0.73F;
        b.xmin = 0.25F;
        b.ymin = 0.20F;
        b.xmax = 0.58F;
        b.ymax = 0.70F;

        input->detections = {a, b};

        auto output = runTypedScriptRoundtrip(input, kImgDetectionsRoundtripScript);
        requireImgDetectionsEqual(*input, *output);
    }

    SECTION("With keypoints") {
        auto input = std::make_shared<dai::ImgDetections>();
        setCommonMessageFields(*input, 102, 2234);

        dai::ImgDetection det;
        det.label = 3;
        det.labelName = "pose";
        det.confidence = 0.88F;
        det.setOuterBoundingBox(0.10F, 0.15F, 0.55F, 0.85F);

        std::vector<dai::Keypoint> keypoints{dai::Keypoint(dai::Point3f{0.2F, 0.3F, 0.0F}, 0.95F, 0, "nose"),
                                             dai::Keypoint(dai::Point3f{0.3F, 0.4F, 0.0F}, 0.90F, 1, "left_eye"),
                                             dai::Keypoint(dai::Point3f{0.4F, 0.4F, 0.0F}, 0.89F, 2, "right_eye")};
        std::vector<dai::Edge> edges{{0, 1}, {0, 2}};
        det.setKeypoints(keypoints, edges);

        input->detections = {det};

        auto output = runTypedScriptRoundtrip(input, kImgDetectionsRoundtripScript);
        requireImgDetectionsEqual(*input, *output);
    }

    SECTION("With bboxes") {
        auto input = std::make_shared<dai::ImgDetections>();
        setCommonMessageFields(*input, 103, 3234);

        dai::ImgDetection rotated(dai::RotatedRect{dai::Point2f{0.45F, 0.5F, true}, dai::Size2f{0.3F, 0.2F, true}, 23.0F}, 0.86F, 4);
        rotated.labelName = "rotated_box";

        dai::ImgDetection outer;
        outer.label = 5;
        outer.labelName = "outer_box";
        outer.confidence = 0.67F;
        outer.setOuterBoundingBox(0.05F, 0.07F, 0.35F, 0.40F);

        input->detections = {rotated, outer};

        auto output = runTypedScriptRoundtrip(input, kImgDetectionsRoundtripScript);
        requireImgDetectionsEqual(*input, *output);
    }

    SECTION("With segmask") {
        auto input = std::make_shared<dai::ImgDetections>();
        setCommonMessageFields(*input, 104, 4234);

        dai::ImgDetection det;
        det.label = 6;
        det.labelName = "instance";
        det.confidence = 0.99F;
        det.setOuterBoundingBox(0.20F, 0.20F, 0.80F, 0.80F);
        input->detections = {det};

        std::vector<uint8_t> mask{0, 0, 1, 1, 0, 2, 2, 1, 255, 2, 2, 255};
        input->setSegmentationMask(mask, 4, 3);

        auto output = runTypedScriptRoundtrip(input, kImgDetectionsRoundtripScript);
        requireImgDetectionsEqual(*input, *output);
    }
}

TEST_CASE("SpatialImgDetections roundtrip in Script node", "[script][spatial_imgdetections]") {
    auto makeMapping = []() {
        dai::SpatialLocationCalculatorConfigData cfg;
        cfg.roi = dai::Rect{0.2F, 0.25F, 0.3F, 0.35F, true};
        cfg.depthThresholds.lowerThreshold = 200;
        cfg.depthThresholds.upperThreshold = 3000;
        cfg.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MIN;
        cfg.stepSize = 2;
        return cfg;
    };

    SECTION("basic") {
        auto input = std::make_shared<dai::SpatialImgDetections>();
        setCommonMessageFields(*input, 201, 1334);

        dai::SpatialImgDetection det;
        det.label = 11;
        det.labelName = "basic_spatial";
        det.confidence = 0.93F;
        det.setOuterBoundingBox(0.11F, 0.18F, 0.50F, 0.72F);
        det.spatialCoordinates = dai::Point3f{120.0F, 250.0F, 1450.0F};
        det.boundingBoxMapping = makeMapping();
        input->detections = {det};

        auto output = runTypedScriptRoundtrip(input, kSpatialImgDetectionsRoundtripScript);
        requireSpatialImgDetectionsEqual(*input, *output);
    }

    SECTION("With keypoints") {
        auto input = std::make_shared<dai::SpatialImgDetections>();
        setCommonMessageFields(*input, 202, 2334);

        dai::SpatialImgDetection det;
        det.label = 12;
        det.labelName = "spatial_pose";
        det.confidence = 0.84F;
        det.setOuterBoundingBox(0.10F, 0.12F, 0.52F, 0.90F);
        det.spatialCoordinates = dai::Point3f{80.0F, 140.0F, 1200.0F};
        det.boundingBoxMapping = makeMapping();

        std::vector<dai::SpatialKeypoint> keypoints{
            dai::SpatialKeypoint(dai::Point3f{0.20F, 0.25F, 0.0F}, dai::Point3f{80.0F, 140.0F, 1200.0F}, 0.9F, 0, "kp0"),
            dai::SpatialKeypoint(dai::Point3f{0.26F, 0.30F, 0.0F}, dai::Point3f{82.0F, 138.0F, 1190.0F}, 0.88F, 1, "kp1"),
            dai::SpatialKeypoint(dai::Point3f{0.33F, 0.34F, 0.0F}, dai::Point3f{85.0F, 135.0F, 1180.0F}, 0.87F, 2, "kp2")};
        std::vector<dai::Edge> edges{{0, 1}, {1, 2}};
        det.setKeypoints(keypoints, edges);

        input->detections = {det};

        auto output = runTypedScriptRoundtrip(input, kSpatialImgDetectionsRoundtripScript);
        requireSpatialImgDetectionsEqual(*input, *output);
    }

    SECTION("With bboxes") {
        auto input = std::make_shared<dai::SpatialImgDetections>();
        setCommonMessageFields(*input, 203, 3334);

        dai::SpatialImgDetection rotated(
            dai::RotatedRect{dai::Point2f{0.5F, 0.5F, true}, dai::Size2f{0.35F, 0.2F, true}, 15.0F}, dai::Point3f{300.0F, 20.0F, 1300.0F}, 0.82F, 13);
        rotated.labelName = "rot_spatial";
        rotated.boundingBoxMapping = makeMapping();

        dai::SpatialImgDetection outer;
        outer.label = 14;
        outer.labelName = "outer_spatial";
        outer.confidence = 0.71F;
        outer.setOuterBoundingBox(0.15F, 0.20F, 0.45F, 0.55F);
        outer.setSpatialCoordinate(dai::Point3f{330.0F, -40.0F, 1500.0F});
        outer.boundingBoxMapping = makeMapping();

        input->detections = {rotated, outer};

        auto output = runTypedScriptRoundtrip(input, kSpatialImgDetectionsRoundtripScript);
        requireSpatialImgDetectionsEqual(*input, *output);
    }

    SECTION("With segmask") {
        auto input = std::make_shared<dai::SpatialImgDetections>();
        setCommonMessageFields(*input, 204, 4334);

        dai::SpatialImgDetection det;
        det.label = 15;
        det.labelName = "seg_spatial";
        det.confidence = 0.95F;
        det.setOuterBoundingBox(0.22F, 0.16F, 0.78F, 0.87F);
        det.setSpatialCoordinate(dai::Point3f{25.0F, 60.0F, 900.0F});
        det.boundingBoxMapping = makeMapping();
        input->detections = {det};

        std::vector<uint8_t> mask{0, 0, 0, 1, 0, 1, 1, 1, 255, 1, 1, 255};
        input->setSegmentationMask(mask, 4, 3);

        auto output = runTypedScriptRoundtrip(input, kSpatialImgDetectionsRoundtripScript);
        requireSpatialImgDetectionsEqual(*input, *output);
    }
}
