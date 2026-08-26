#include <depthai/schemas/common.pb.h>

#include <catch2/catch_all.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/ImgAnnotations.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <variant>

#ifdef DEPTHAI_ENABLE_PROTOBUF
namespace {

namespace fs = std::filesystem;

class ScopedTempDir {
   public:
    explicit ScopedTempDir(const std::string& prefix) {
        std::ostringstream name;
        name << prefix << "_" << std::chrono::steady_clock::now().time_since_epoch().count();
        path = fs::temp_directory_path() / name.str();
        fs::create_directories(path);
    }

    ~ScopedTempDir() {
        std::error_code ec;
        fs::remove_all(path, ec);
    }

    const fs::path& get() const {
        return path;
    }

   private:
    fs::path path;
};

fs::path resolveSavedPath(const fs::path& path) {
    if(path.has_extension()) return path;
    auto resolved = path;
    resolved += ".dai";
    return resolved;
}

template <typename T>
T saveLoad(const T& source, const fs::path& path, bool metadataOnly = false) {
    source.save(path, metadataOnly);
    REQUIRE(fs::exists(resolveSavedPath(path)));
    T restored;
    restored.load(path);
    return restored;
}

std::vector<std::uint8_t> toVector(dai::span<const std::uint8_t> data) {
    return {data.begin(), data.end()};
}

std::chrono::steady_clock::time_point makeSteadyTime(int64_t seed) {
    return std::chrono::steady_clock::time_point(std::chrono::seconds(100 + seed) + std::chrono::microseconds(seed));
}

std::chrono::system_clock::time_point makeSystemTime(int64_t seed) {
    return std::chrono::system_clock::time_point(std::chrono::seconds(1000 + seed) + std::chrono::microseconds(seed * 10));
}

template <typename T>
void applyBufferMetadata(T& message, int64_t seed) {
    message.setSequenceNum(seed);
    message.setTimestamp(makeSteadyTime(seed));
    message.setTimestampDevice(makeSteadyTime(seed + 100));
    message.setTimestampSystem(makeSystemTime(seed));
}

template <typename T>
void requireBufferMetadata(const T& actual, const T& expected) {
    REQUIRE(actual.getSequenceNum() == expected.getSequenceNum());
    REQUIRE(actual.getTimestamp() == expected.getTimestamp());
    REQUIRE(actual.getTimestampDevice() == expected.getTimestampDevice());
    REQUIRE(actual.getTimestampSystem() == expected.getTimestampSystem());
}

void requireTransformation(const std::optional<dai::ImgTransformation>& actual, const std::optional<dai::ImgTransformation>& expected) {
    REQUIRE(actual.has_value() == expected.has_value());
    if(actual.has_value()) {
        REQUIRE(actual->isEqualTransformation(*expected));
    }
}

void requirePoint2f(const dai::Point2f& actual, const dai::Point2f& expected) {
    REQUIRE(actual.x == Catch::Approx(expected.x));
    REQUIRE(actual.y == Catch::Approx(expected.y));
    REQUIRE(actual.hasNormalized == expected.hasNormalized);
    REQUIRE(actual.normalized == expected.normalized);
    REQUIRE(actual.isNormalized() == expected.isNormalized());
}

void requireSize2f(const dai::Size2f& actual, const dai::Size2f& expected) {
    REQUIRE(actual.width == Catch::Approx(expected.width));
    REQUIRE(actual.height == Catch::Approx(expected.height));
    REQUIRE(actual.hasNormalized == expected.hasNormalized);
    REQUIRE(actual.normalized == expected.normalized);
    REQUIRE(actual.isNormalized() == expected.isNormalized());
}

void requirePoint3f(const dai::Point3f& actual, const dai::Point3f& expected) {
    REQUIRE(actual.x == Catch::Approx(expected.x));
    REQUIRE(actual.y == Catch::Approx(expected.y));
    REQUIRE(actual.z == Catch::Approx(expected.z));
}

void requireRotatedRect(const dai::RotatedRect& actual, const dai::RotatedRect& expected) {
    requirePoint2f(actual.center, expected.center);
    requireSize2f(actual.size, expected.size);
    REQUIRE(actual.angle == Catch::Approx(expected.angle));
}

void requireRect(const dai::Rect& actual, const dai::Rect& expected) {
    REQUIRE(actual.x == Catch::Approx(expected.x));
    REQUIRE(actual.y == Catch::Approx(expected.y));
    REQUIRE(actual.width == Catch::Approx(expected.width));
    REQUIRE(actual.height == Catch::Approx(expected.height));
    REQUIRE(actual.hasNormalized == expected.hasNormalized);
    REQUIRE(actual.normalized == expected.normalized);
    REQUIRE(actual.isNormalized() == expected.isNormalized());
}

void requireImgFrame(const dai::ImgFrame& actual, const dai::ImgFrame& expected, bool expectData = true) {
    requireBufferMetadata(actual, expected);
    REQUIRE(actual.getInstanceNum() == expected.getInstanceNum());
    REQUIRE(actual.getCategory() == expected.getCategory());
    REQUIRE(actual.getWidth() == expected.getWidth());
    REQUIRE(actual.getHeight() == expected.getHeight());
    REQUIRE(actual.getSourceWidth() == expected.getSourceWidth());
    REQUIRE(actual.getSourceHeight() == expected.getSourceHeight());
    REQUIRE(actual.getType() == expected.getType());
    REQUIRE(actual.getExposureTime() == expected.getExposureTime());
    REQUIRE(actual.getSensitivity() == expected.getSensitivity());
    REQUIRE(actual.getColorTemperature() == expected.getColorTemperature());
    REQUIRE(actual.getLensPosition() == expected.getLensPosition());
    REQUIRE(actual.getLensPositionRaw() == Catch::Approx(expected.getLensPositionRaw()));
    REQUIRE(actual.getSensorTemperature() == expected.getSensorTemperature());
    if(expectData) {
        REQUIRE(toVector(actual.getData()) == toVector(expected.getData()));
    } else {
        REQUIRE(actual.getData().empty());
    }
}

void requireEncodedFrame(const dai::EncodedFrame& actual, const dai::EncodedFrame& expected, bool expectData = true) {
    requireBufferMetadata(actual, expected);
    REQUIRE(actual.getInstanceNum() == expected.getInstanceNum());
    REQUIRE(actual.getWidth() == expected.getWidth());
    REQUIRE(actual.getHeight() == expected.getHeight());
    REQUIRE(actual.getExposureTime() == expected.getExposureTime());
    REQUIRE(actual.getSensitivity() == expected.getSensitivity());
    REQUIRE(actual.getColorTemperature() == expected.getColorTemperature());
    REQUIRE(actual.getLensPosition() == expected.getLensPosition());
    REQUIRE(actual.getLensPositionRaw() == Catch::Approx(expected.getLensPositionRaw()));
    REQUIRE(actual.getSensorTemperature() == expected.getSensorTemperature());
    REQUIRE(actual.getQuality() == expected.getQuality());
    REQUIRE(actual.getBitrate() == expected.getBitrate());
    REQUIRE(actual.getLossless() == expected.getLossless());
    REQUIRE(actual.getProfile() == expected.getProfile());
    REQUIRE(actual.type == expected.type);
    REQUIRE(actual.frameOffset == expected.frameOffset);
    REQUIRE(actual.frameSize == expected.frameSize);
    REQUIRE(actual.transformation.isEqualTransformation(expected.transformation));
    if(expectData) {
        REQUIRE(toVector(actual.getData()) == toVector(expected.getData()));
    } else {
        REQUIRE(actual.getData().empty());
    }
}

dai::ImgFrame makeImgFrame(int64_t seed) {
    dai::ImgFrame frame;
    applyBufferMetadata(frame, seed);
    frame.setInstanceNum(9).setCategory(4).setType(dai::ImgFrame::Type::BGR888i);
    frame.setSourceSize(3, 2);
    frame.setSize(3, 2);
    frame.cam.exposureTimeUs = 2000;
    frame.cam.sensitivityIso = 800;
    frame.cam.wbColorTemp = 5100;
    frame.cam.lensPosition = 77;
    frame.cam.lensPositionRaw = 0.75F;
    frame.cam.sensorTemperatureC = 33.5F;
    frame.setData(std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6});
    return frame;
}

dai::EncodedFrame makeEncodedFrame(int64_t seed) {
    dai::EncodedFrame frame;
    applyBufferMetadata(frame, seed);
    frame.setInstanceNum(3).setSize(8, 6).setQuality(88).setBitrate(123456).setLossless(false);
    frame.setProfile(dai::EncodedFrame::Profile::AVC).setFrameType(dai::EncodedFrame::FrameType::P);
    frame.cam.exposureTimeUs = 4000;
    frame.cam.sensitivityIso = 320;
    frame.cam.wbColorTemp = 4700;
    frame.cam.lensPosition = 22;
    frame.cam.lensPositionRaw = 0.3F;
    frame.cam.sensorTemperatureC = 41.0F;
    frame.frameOffset = 11;
    frame.frameSize = 4;
    frame.transformation = dai::ImgTransformation(8, 6);
    frame.setData(std::vector<std::uint8_t>{9, 8, 7, 6});
    return frame;
}

}  // namespace

TEST_CASE("ProtoSerializable save/load roundtrip for ImgFrame", "[ProtoSerializable][ImgFrame]") {
    ScopedTempDir tempDir("depthai_proto_imgframe");
    const auto path = tempDir.get() / "img_frame";

    const auto source = makeImgFrame(1);
    const auto restored = saveLoad(source, path);
    requireImgFrame(restored, source);

    source.save(path, true);
    dai::ImgFrame reused = makeImgFrame(101);
    reused.load(path);
    requireImgFrame(reused, source, false);
}

TEST_CASE("ProtoSerializable preserves explicit file extensions", "[ProtoSerializable][Paths]") {
    ScopedTempDir tempDir("depthai_proto_paths");
    const auto path = tempDir.get() / "img_frame.pb";

    const auto source = makeImgFrame(13);
    source.save(path, false);

    REQUIRE(fs::exists(path));
    REQUIRE_FALSE(fs::exists(path.string() + ".dai"));

    dai::ImgFrame restored;
    restored.load(path);
    requireImgFrame(restored, source);
}

TEST_CASE("ProtoSerializable persists a schema envelope", "[ProtoSerializable][Envelope]") {
    ScopedTempDir tempDir("depthai_proto_envelope");
    const auto path = tempDir.get() / "img_frame";
    const auto source = makeImgFrame(14);

    source.save(path, true);

    std::ifstream file(resolveSavedPath(path), std::ios::binary);
    dai::proto::common::ProtoSerializableMessage envelope;
    REQUIRE(envelope.ParseFromIstream(&file));
    REQUIRE(envelope.schema_name() == source.serializeSchema().schemaName);
    REQUIRE(envelope.metadata_only());
    const auto serializedMessage = source.serializeProto(true);
    REQUIRE(envelope.proto_message() == std::string(serializedMessage.begin(), serializedMessage.end()));
}

TEST_CASE("ProtoSerializable rejects a different schema", "[ProtoSerializable][Envelope]") {
    ScopedTempDir tempDir("depthai_proto_schema_mismatch");
    const auto path = tempDir.get() / "img_frame";

    makeImgFrame(15).save(path);
    dai::EncodedFrame message;
    REQUIRE_THROWS_WITH(message.load(path), Catch::Matchers::ContainsSubstring("Schema mismatch"));
}

TEST_CASE("ProtoSerializable save/load roundtrip for EncodedFrame", "[ProtoSerializable][EncodedFrame]") {
    ScopedTempDir tempDir("depthai_proto_encoded_frame");
    const auto path = tempDir.get() / "encoded_frame";
    const auto metadataPath = tempDir.get() / "encoded_frame_metadata";

    const auto source = makeEncodedFrame(2);
    const auto restored = saveLoad(source, path);
    requireEncodedFrame(restored, source);

    source.save(path, false);
    source.save(metadataPath, true);
    REQUIRE(fs::file_size(resolveSavedPath(metadataPath)) < fs::file_size(resolveSavedPath(path)));

    const auto metadataOnly = saveLoad(source, metadataPath, true);
    requireEncodedFrame(metadataOnly, source, false);

    dai::EncodedFrame reused = makeEncodedFrame(102);
    reused.load(metadataPath);
    requireEncodedFrame(reused, source, false);
}

TEST_CASE("ProtoSerializable save/load roundtrip for PointCloudData", "[ProtoSerializable][PointCloudData]") {
    ScopedTempDir tempDir("depthai_proto_pointcloud");
    const auto path = tempDir.get() / "pointcloud";
    const auto metadataPath = tempDir.get() / "pointcloud_metadata";

    dai::PointCloudData source;
    applyBufferMetadata(source, 3);
    source.setPointsRGB({{1.f, 2.f, 3.f, 10, 20, 30, 255}, {4.f, 5.f, 6.f, 40, 50, 60, 255}});
    source.setSize(2, 1).setInstanceNum(5);
    source.updateBoundingBox();

    const auto restored = saveLoad(source, path);
    requireBufferMetadata(restored, source);
    REQUIRE(restored.isColor() == source.isColor());
    REQUIRE(restored.getWidth() == source.getWidth());
    REQUIRE(restored.getHeight() == source.getHeight());
    REQUIRE(restored.getInstanceNum() == source.getInstanceNum());
    REQUIRE(restored.getMinX() == Catch::Approx(source.getMinX()));
    REQUIRE(restored.getMinY() == Catch::Approx(source.getMinY()));
    REQUIRE(restored.getMinZ() == Catch::Approx(source.getMinZ()));
    REQUIRE(restored.getMaxX() == Catch::Approx(source.getMaxX()));
    REQUIRE(restored.getMaxY() == Catch::Approx(source.getMaxY()));
    REQUIRE(restored.getMaxZ() == Catch::Approx(source.getMaxZ()));
    REQUIRE(restored.getPointsRGB().size() == source.getPointsRGB().size());
    REQUIRE(restored.getPointsRGB()[1].g == source.getPointsRGB()[1].g);

    source.save(path, false);
    source.save(metadataPath, true);
    REQUIRE(fs::file_size(resolveSavedPath(metadataPath)) < fs::file_size(resolveSavedPath(path)));

    const auto metadataOnly = saveLoad(source, metadataPath, true);
    requireBufferMetadata(metadataOnly, source);
    REQUIRE(metadataOnly.getWidth() == source.getWidth());
    REQUIRE(metadataOnly.getHeight() == source.getHeight());
    REQUIRE(metadataOnly.getInstanceNum() == source.getInstanceNum());
    REQUIRE(metadataOnly.getData().empty());

    dai::PointCloudData reused;
    reused.setPointsRGB({{9.f, 8.f, 7.f, 1, 2, 3, 255}});
    reused.setSize(1, 1).setInstanceNum(99);
    reused.load(metadataPath);
    requireBufferMetadata(reused, source);
    REQUIRE(reused.getWidth() == source.getWidth());
    REQUIRE(reused.getHeight() == source.getHeight());
    REQUIRE(reused.getInstanceNum() == source.getInstanceNum());
    REQUIRE(reused.getData().empty());
}

TEST_CASE("ProtoSerializable save/load roundtrip for SegmentationMask", "[ProtoSerializable][SegmentationMask]") {
    ScopedTempDir tempDir("depthai_proto_segmentation_mask");
    const auto path = tempDir.get() / "segmentation_mask";
    const auto metadataPath = tempDir.get() / "segmentation_mask_metadata";

    dai::SegmentationMask source;
    applyBufferMetadata(source, 4);
    source.transformation = dai::ImgTransformation(2, 2);
    source.setLabels({"background", "person", "car"});
    source.setMask(std::vector<std::uint8_t>{0, 1, 2, 255}, 2, 2);

    const auto restored = saveLoad(source, path);
    requireBufferMetadata(restored, source);
    requireTransformation(restored.transformation, source.transformation);
    REQUIRE(restored.getWidth() == source.getWidth());
    REQUIRE(restored.getHeight() == source.getHeight());
    REQUIRE(restored.getLabels() == source.getLabels());
    REQUIRE(restored.getMaskData() == source.getMaskData());

    source.save(metadataPath, true);
    dai::SegmentationMask reused(std::vector<std::uint8_t>{7, 7, 7, 7}, 2, 2);
    reused.load(metadataPath);
    requireBufferMetadata(reused, source);
    requireTransformation(reused.transformation, source.transformation);
    REQUIRE(reused.getWidth() == source.getWidth());
    REQUIRE(reused.getHeight() == source.getHeight());
    REQUIRE(reused.getLabels() == source.getLabels());
    REQUIRE(reused.getMaskData().empty());
}

TEST_CASE("ProtoSerializable save/load roundtrip for ImgAnnotations", "[ProtoSerializable][ImgAnnotations]") {
    ScopedTempDir tempDir("depthai_proto_img_annotations");
    const auto path = tempDir.get() / "img_annotations";

    dai::ImgAnnotations source;
    applyBufferMetadata(source, 5);
    source.transformation = dai::ImgTransformation(640, 480);

    dai::ImgAnnotation annotation;
    annotation.circles.push_back({dai::Point2f{0.2F, 0.3F, true}, 0.1F, 2.0F, dai::Color{0.1F, 0.2F, 0.3F, 1.0F}, dai::Color{0.4F, 0.5F, 0.6F, 1.0F}});
    annotation.points.push_back({dai::PointsAnnotationType::LINE_STRIP,
                                 {{0.1F, 0.2F, true}, {0.3F, 0.4F, true}},
                                 dai::Color{0.7F, 0.2F, 0.1F, 1.0F},
                                 {dai::Color{0.2F, 0.8F, 0.1F, 1.0F}},
                                 dai::Color{0.1F, 0.1F, 0.9F, 0.5F},
                                 3.0F});
    annotation.texts.push_back({dai::Point2f{0.6F, 0.7F, true}, "label", 18.0F, dai::Color{1.0F, 1.0F, 1.0F, 1.0F}, dai::Color{0.0F, 0.0F, 0.0F, 1.0F}});
    source.annotations.push_back(annotation);

    const auto restored = saveLoad(source, path);
    requireBufferMetadata(restored, source);
    requireTransformation(restored.transformation, source.transformation);
    REQUIRE(restored.annotations.size() == 1);
    REQUIRE(restored.annotations[0].circles.size() == 1);
    REQUIRE(restored.annotations[0].points.size() == 1);
    REQUIRE(restored.annotations[0].texts.size() == 1);
    requirePoint2f(restored.annotations[0].circles[0].position, source.annotations[0].circles[0].position);
    REQUIRE(restored.annotations[0].points[0].type == source.annotations[0].points[0].type);
    REQUIRE(restored.annotations[0].points[0].outlineColors.size() == source.annotations[0].points[0].outlineColors.size());
    REQUIRE(restored.annotations[0].texts[0].text == source.annotations[0].texts[0].text);
}

TEST_CASE("ProtoSerializable preserves explicit normalized geometry flags", "[ProtoSerializable][Geometry]") {
    ScopedTempDir tempDir("depthai_proto_geometry");
    const auto annotationsPath = tempDir.get() / "img_annotations";
    const auto detectionsPath = tempDir.get() / "img_detections";

    dai::ImgAnnotations annotations;
    applyBufferMetadata(annotations, 50);
    annotations.transformation = dai::ImgTransformation(640, 480);

    dai::ImgAnnotation annotation;
    annotation.circles.push_back({dai::Point2f{0.0F, 1.0F, true}, 0.1F, 2.0F, dai::Color{0.1F, 0.2F, 0.3F, 1.0F}, dai::Color{0.4F, 0.5F, 0.6F, 1.0F}});
    annotation.points.push_back({dai::PointsAnnotationType::POINTS,
                                 {{0.0F, 0.0F, true}, {1.0F, 1.0F, true}},
                                 dai::Color{0.7F, 0.2F, 0.1F, 1.0F},
                                 {},
                                 dai::Color{0.1F, 0.1F, 0.9F, 0.5F},
                                 3.0F});
    annotation.texts.push_back({dai::Point2f{1.0F, 0.0F, true}, "edge", 18.0F, dai::Color{1.0F, 1.0F, 1.0F, 1.0F}, dai::Color{0.0F, 0.0F, 0.0F, 1.0F}});
    annotations.annotations.push_back(annotation);

    const auto restoredAnnotations = saveLoad(annotations, annotationsPath);
    requirePoint2f(restoredAnnotations.annotations[0].circles[0].position, annotations.annotations[0].circles[0].position);
    requirePoint2f(restoredAnnotations.annotations[0].points[0].points[0], annotations.annotations[0].points[0].points[0]);
    requirePoint2f(restoredAnnotations.annotations[0].points[0].points[1], annotations.annotations[0].points[0].points[1]);
    requirePoint2f(restoredAnnotations.annotations[0].texts[0].position, annotations.annotations[0].texts[0].position);

    dai::ImgDetections detections;
    applyBufferMetadata(detections, 51);
    detections.transformation = dai::ImgTransformation(320, 240);

    dai::ImgDetection detection;
    detection.label = 3;
    detection.labelName = "edge";
    detection.confidence = 0.75F;
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f{0.0F, 1.0F, true}, dai::Size2f{1.0F, 0.0F, true}, 15.0F));
    detections.detections.push_back(detection);

    const auto restoredDetections = saveLoad(detections, detectionsPath);
    requireRotatedRect(restoredDetections.detections[0].getBoundingBox(), detections.detections[0].getBoundingBox());
}

TEST_CASE("ProtoSerializable save/load roundtrip for ImgDetections", "[ProtoSerializable][ImgDetections]") {
    ScopedTempDir tempDir("depthai_proto_img_detections");
    const auto path = tempDir.get() / "img_detections";
    const auto metadataPath = tempDir.get() / "img_detections_metadata";
    const auto noTransformPath = tempDir.get() / "img_detections_no_transform";
    const auto noTransformMetadataPath = tempDir.get() / "img_detections_no_transform_metadata";

    dai::ImgDetections source;
    applyBufferMetadata(source, 6);
    source.transformation = dai::ImgTransformation(320, 240);

    dai::ImgDetection detection;
    detection.label = 3;
    detection.labelName = "person";
    detection.confidence = 0.75F;
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f{0.5F, 0.5F, true}, dai::Size2f{0.25F, 0.4F, true}, 15.0F));
    detection.setKeypoints({dai::Keypoint(dai::Point3f{0.1F, 0.2F, 0.0F}, 0.9F, 1), dai::Keypoint(dai::Point3f{0.3F, 0.4F, 0.0F}, 0.8F, 2)}, {dai::Edge{0, 1}});
    source.detections.push_back(detection);
    source.setSegmentationMask(std::vector<std::uint8_t>{0, 0, 1, 255}, 2, 2);

    const auto restored = saveLoad(source, path);
    requireBufferMetadata(restored, source);
    requireTransformation(restored.transformation, source.transformation);
    REQUIRE(restored.detections.size() == 1);
    REQUIRE(restored.getSegmentationMaskWidth() == source.getSegmentationMaskWidth());
    REQUIRE(restored.getSegmentationMaskHeight() == source.getSegmentationMaskHeight());
    REQUIRE(restored.getMaskData() == source.getMaskData());
    REQUIRE(restored.detections[0].label == source.detections[0].label);
    REQUIRE(restored.detections[0].labelName == source.detections[0].labelName);
    REQUIRE(restored.detections[0].confidence == Catch::Approx(source.detections[0].confidence));
    requireRotatedRect(restored.detections[0].getBoundingBox(), source.detections[0].getBoundingBox());
    REQUIRE(restored.detections[0].getKeypoints().size() == source.detections[0].getKeypoints().size());
    REQUIRE(restored.detections[0].getEdges() == source.detections[0].getEdges());

    source.save(metadataPath, true);
    dai::ImgDetections reused;
    reused.setSegmentationMask(std::vector<std::uint8_t>{1, 1, 1, 1}, 2, 2);
    reused.load(metadataPath);
    requireBufferMetadata(reused, source);
    requireTransformation(reused.transformation, source.transformation);
    REQUIRE(reused.getSegmentationMaskWidth() == source.getSegmentationMaskWidth());
    REQUIRE(reused.getSegmentationMaskHeight() == source.getSegmentationMaskHeight());
    REQUIRE_FALSE(reused.getMaskData().has_value());

    dai::ImgDetections noTransformSource;
    applyBufferMetadata(noTransformSource, 16);
    noTransformSource.detections.push_back(detection);
    noTransformSource.setSegmentationMask(std::vector<std::uint8_t>{1, 2, 3, 4}, 2, 2);

    const auto restoredWithoutTransform = saveLoad(noTransformSource, noTransformPath);
    requireBufferMetadata(restoredWithoutTransform, noTransformSource);
    REQUIRE_FALSE(restoredWithoutTransform.transformation.has_value());
    REQUIRE(restoredWithoutTransform.getSegmentationMaskWidth() == noTransformSource.getSegmentationMaskWidth());
    REQUIRE(restoredWithoutTransform.getSegmentationMaskHeight() == noTransformSource.getSegmentationMaskHeight());
    REQUIRE(restoredWithoutTransform.getMaskData() == noTransformSource.getMaskData());
    REQUIRE(restoredWithoutTransform.detections.size() == 1);

    noTransformSource.save(noTransformMetadataPath, true);
    dai::ImgDetections noTransformReused;
    noTransformReused.load(noTransformMetadataPath);
    requireBufferMetadata(noTransformReused, noTransformSource);
    REQUIRE_FALSE(noTransformReused.transformation.has_value());
    REQUIRE(noTransformReused.getSegmentationMaskWidth() == noTransformSource.getSegmentationMaskWidth());
    REQUIRE(noTransformReused.getSegmentationMaskHeight() == noTransformSource.getSegmentationMaskHeight());
    REQUIRE_FALSE(noTransformReused.getMaskData().has_value());
}

TEST_CASE("ProtoSerializable save/load roundtrip for SpatialImgDetections", "[ProtoSerializable][SpatialImgDetections]") {
    ScopedTempDir tempDir("depthai_proto_spatial_img_detections");
    const auto path = tempDir.get() / "spatial_img_detections";
    const auto metadataPath = tempDir.get() / "spatial_img_detections_metadata";
    const auto noTransformPath = tempDir.get() / "spatial_img_detections_no_transform";
    const auto noTransformMetadataPath = tempDir.get() / "spatial_img_detections_no_transform_metadata";

    dai::SpatialImgDetections source;
    applyBufferMetadata(source, 7);
    source.transformation = dai::ImgTransformation(300, 200);
    source.transformation->addSrcCrops({dai::RotatedRect{dai::Point2f{0.5F, 0.4F, true}, dai::Size2f{0.25F, 0.35F, true}, 12.0F}});
    source.unit = dai::LengthUnit::CENTIMETER;

    dai::SpatialImgDetection detection;
    detection.label = 1;
    detection.labelName = "target";
    detection.confidence = 0.6F;
    detection.setBoundingBox(dai::RotatedRect(dai::Point2f{0.4F, 0.5F, true}, dai::Size2f{0.2F, 0.3F, true}, 5.0F));
    detection.spatialCoordinates = {10.F, 20.F, 30.F};
    detection.boundingBoxMapping.roi = dai::Rect(0.1F, 0.2F, 0.3F, 0.4F, true);
    detection.boundingBoxMapping.depthThresholds.lowerThreshold = 111;
    detection.boundingBoxMapping.depthThresholds.upperThreshold = 999;
    detection.boundingBoxMapping.calculationAlgorithm = dai::SpatialLocationCalculatorAlgorithm::MAX;
    detection.boundingBoxMapping.stepSize = 3;
    detection.setKeypoints(
        {dai::SpatialKeypoint(0.1F, 0.2F, 0.0F, 1.0F, 2.0F, 3.0F, 0.9F, 7, "nose"), dai::SpatialKeypoint(0.3F, 0.4F, 0.0F, 4.0F, 5.0F, 6.0F, 0.8F, 8, "eye")},
        {dai::Edge{0, 1}});
    source.detections.push_back(detection);
    source.setSegmentationMask(std::vector<std::uint8_t>{0, 255, 1, 1}, 2, 2);

    const auto restored = saveLoad(source, path);
    requireBufferMetadata(restored, source);
    requireTransformation(restored.transformation, source.transformation);
    REQUIRE(restored.transformation.has_value());
    REQUIRE(source.transformation.has_value());
    REQUIRE(restored.transformation->getSrcCrops().size() == source.transformation->getSrcCrops().size());
    requireRotatedRect(restored.transformation->getSrcCrops().at(0), source.transformation->getSrcCrops().at(0));
    REQUIRE(restored.unit == source.unit);
    REQUIRE(restored.detections.size() == 1);
    REQUIRE(restored.getMaskData() == source.getMaskData());
    REQUIRE(restored.detections[0].labelName == source.detections[0].labelName);
    requireRotatedRect(restored.detections[0].getBoundingBox(), source.detections[0].getBoundingBox());
    requirePoint3f(restored.detections[0].spatialCoordinates, source.detections[0].spatialCoordinates);
    requireRect(restored.detections[0].boundingBoxMapping.roi, source.detections[0].boundingBoxMapping.roi);
    REQUIRE(restored.detections[0].boundingBoxMapping.depthThresholds.lowerThreshold == source.detections[0].boundingBoxMapping.depthThresholds.lowerThreshold);
    REQUIRE(restored.detections[0].boundingBoxMapping.depthThresholds.upperThreshold == source.detections[0].boundingBoxMapping.depthThresholds.upperThreshold);
    REQUIRE(restored.detections[0].boundingBoxMapping.calculationAlgorithm == source.detections[0].boundingBoxMapping.calculationAlgorithm);
    REQUIRE(restored.detections[0].boundingBoxMapping.stepSize == source.detections[0].boundingBoxMapping.stepSize);
    REQUIRE(restored.detections[0].getKeypoints().size() == source.detections[0].getKeypoints().size());
    REQUIRE(restored.detections[0].getEdges() == source.detections[0].getEdges());

    source.save(metadataPath, true);
    dai::SpatialImgDetections reused;
    reused.setSegmentationMask(std::vector<std::uint8_t>{1, 1, 1, 1}, 2, 2);
    reused.load(metadataPath);
    requireBufferMetadata(reused, source);
    requireTransformation(reused.transformation, source.transformation);
    REQUIRE(reused.transformation.has_value());
    REQUIRE(source.transformation.has_value());
    REQUIRE(reused.transformation->getSrcCrops().size() == source.transformation->getSrcCrops().size());
    requireRotatedRect(reused.transformation->getSrcCrops().at(0), source.transformation->getSrcCrops().at(0));
    REQUIRE(reused.getSegmentationMaskWidth() == source.getSegmentationMaskWidth());
    REQUIRE(reused.getSegmentationMaskHeight() == source.getSegmentationMaskHeight());
    REQUIRE_FALSE(reused.getMaskData().has_value());
    REQUIRE(reused.detections.size() == 1);
    requireRect(reused.detections[0].boundingBoxMapping.roi, source.detections[0].boundingBoxMapping.roi);

    dai::SpatialImgDetections noTransformSource;
    applyBufferMetadata(noTransformSource, 17);
    noTransformSource.unit = dai::LengthUnit::CENTIMETER;
    noTransformSource.detections.push_back(detection);
    noTransformSource.setSegmentationMask(std::vector<std::uint8_t>{4, 3, 2, 1}, 2, 2);

    const auto restoredWithoutTransform = saveLoad(noTransformSource, noTransformPath);
    requireBufferMetadata(restoredWithoutTransform, noTransformSource);
    REQUIRE_FALSE(restoredWithoutTransform.transformation.has_value());
    REQUIRE(restoredWithoutTransform.unit == noTransformSource.unit);
    REQUIRE(restoredWithoutTransform.getMaskData() == noTransformSource.getMaskData());
    REQUIRE(restoredWithoutTransform.detections.size() == 1);
    requireRect(restoredWithoutTransform.detections[0].boundingBoxMapping.roi, noTransformSource.detections[0].boundingBoxMapping.roi);

    noTransformSource.save(noTransformMetadataPath, true);
    dai::SpatialImgDetections noTransformReused;
    noTransformReused.load(noTransformMetadataPath);
    requireBufferMetadata(noTransformReused, noTransformSource);
    REQUIRE_FALSE(noTransformReused.transformation.has_value());
    REQUIRE(noTransformReused.unit == noTransformSource.unit);
    REQUIRE(noTransformReused.getSegmentationMaskWidth() == noTransformSource.getSegmentationMaskWidth());
    REQUIRE(noTransformReused.getSegmentationMaskHeight() == noTransformSource.getSegmentationMaskHeight());
    REQUIRE_FALSE(noTransformReused.getMaskData().has_value());
    REQUIRE(noTransformReused.detections.size() == 1);
}

TEST_CASE("ProtoSerializable save/load roundtrip for IMUData", "[ProtoSerializable][IMUData]") {
    ScopedTempDir tempDir("depthai_proto_imu_data");
    const auto path = tempDir.get() / "imu_data";

    dai::IMUData source;
    applyBufferMetadata(source, 8);

    dai::IMUPacket packet;
    packet.acceleroMeter.x = 1.0F;
    packet.acceleroMeter.y = 2.0F;
    packet.acceleroMeter.z = 3.0F;
    packet.acceleroMeter.sequence = 10;
    packet.acceleroMeter.accuracy = dai::IMUReport::Accuracy::HIGH;
    packet.acceleroMeter.timestamp.sec = 1;
    packet.acceleroMeter.timestamp.nsec = 2;
    packet.acceleroMeter.tsDevice.sec = 3;
    packet.acceleroMeter.tsDevice.nsec = 4;
    packet.acceleroMeter.tsSystem = dai::Timestamp{5, 6};

    packet.gyroscope.x = 4.0F;
    packet.gyroscope.y = 5.0F;
    packet.gyroscope.z = 6.0F;
    packet.gyroscope.sequence = 11;
    packet.gyroscope.accuracy = dai::IMUReport::Accuracy::MEDIUM;

    packet.magneticField.x = 7.0F;
    packet.magneticField.y = 8.0F;
    packet.magneticField.z = 9.0F;
    packet.magneticField.sequence = 12;
    packet.magneticField.accuracy = dai::IMUReport::Accuracy::LOW;

    packet.rotationVector.i = 0.1F;
    packet.rotationVector.j = 0.2F;
    packet.rotationVector.k = 0.3F;
    packet.rotationVector.real = 0.4F;
    packet.rotationVector.sequence = 13;
    packet.rotationVector.accuracy = dai::IMUReport::Accuracy::UNRELIABLE;
    packet.rotationVector.tsSystem = dai::Timestamp{7, 8};

    source.packets.push_back(packet);

    const auto restored = saveLoad(source, path);
    requireBufferMetadata(restored, source);
    REQUIRE(restored.packets.size() == 1);
    REQUIRE(restored.packets[0].acceleroMeter.x == Catch::Approx(source.packets[0].acceleroMeter.x));
    REQUIRE(restored.packets[0].acceleroMeter.sequence == source.packets[0].acceleroMeter.sequence);
    REQUIRE(restored.packets[0].acceleroMeter.tsSystem.has_value());
    REQUIRE(restored.packets[0].acceleroMeter.tsSystem->sec == source.packets[0].acceleroMeter.tsSystem->sec);
    REQUIRE(restored.packets[0].gyroscope.z == Catch::Approx(source.packets[0].gyroscope.z));
    REQUIRE(restored.packets[0].magneticField.accuracy == source.packets[0].magneticField.accuracy);
    REQUIRE(restored.packets[0].rotationVector.real == Catch::Approx(source.packets[0].rotationVector.real));
    REQUIRE(restored.packets[0].rotationVector.tsSystem.has_value());
    REQUIRE(restored.packets[0].rotationVector.tsSystem->nsec == source.packets[0].rotationVector.tsSystem->nsec);
}

TEST_CASE("ProtoSerializable save/load roundtrip for RGBDData", "[ProtoSerializable][RGBDData]") {
    ScopedTempDir tempDir("depthai_proto_rgbd_data");
    const auto path = tempDir.get() / "rgbd_data";
    const auto metadataPath = tempDir.get() / "rgbd_data_metadata";

    const auto colorFrame = std::make_shared<dai::ImgFrame>(makeImgFrame(9));
    const auto depthFrame = std::make_shared<dai::EncodedFrame>(makeEncodedFrame(10));

    dai::RGBDData source;
    applyBufferMetadata(source, 11);
    source.setRGBFrame(colorFrame);
    source.setDepthFrame(depthFrame);

    const auto restored = saveLoad(source, path);
    requireBufferMetadata(restored, source);

    REQUIRE(restored.getRGBFrame().has_value());
    REQUIRE(std::holds_alternative<std::shared_ptr<dai::ImgFrame>>(restored.getRGBFrame().value()));
    requireImgFrame(*std::get<std::shared_ptr<dai::ImgFrame>>(restored.getRGBFrame().value()), *colorFrame);

    REQUIRE(restored.getDepthFrame().has_value());
    REQUIRE(std::holds_alternative<std::shared_ptr<dai::EncodedFrame>>(restored.getDepthFrame().value()));
    requireEncodedFrame(*std::get<std::shared_ptr<dai::EncodedFrame>>(restored.getDepthFrame().value()), *depthFrame);

    source.save(path, false);
    source.save(metadataPath, true);
    REQUIRE(fs::file_size(resolveSavedPath(metadataPath)) < fs::file_size(resolveSavedPath(path)));

    const auto metadataOnly = saveLoad(source, metadataPath, true);
    requireBufferMetadata(metadataOnly, source);
    REQUIRE(metadataOnly.getRGBFrame().has_value());
    REQUIRE(metadataOnly.getDepthFrame().has_value());
    requireImgFrame(*std::get<std::shared_ptr<dai::ImgFrame>>(metadataOnly.getRGBFrame().value()), *colorFrame, false);
    requireEncodedFrame(*std::get<std::shared_ptr<dai::EncodedFrame>>(metadataOnly.getDepthFrame().value()), *depthFrame, false);

    dai::RGBDData reused;
    reused.setRGBFrame(std::make_shared<dai::ImgFrame>(makeImgFrame(201)));
    reused.setDepthFrame(std::make_shared<dai::EncodedFrame>(makeEncodedFrame(202)));
    reused.load(metadataPath);
    requireBufferMetadata(reused, source);
    REQUIRE(reused.getRGBFrame().has_value());
    REQUIRE(reused.getDepthFrame().has_value());
    requireImgFrame(*std::get<std::shared_ptr<dai::ImgFrame>>(reused.getRGBFrame().value()), *colorFrame, false);
    requireEncodedFrame(*std::get<std::shared_ptr<dai::EncodedFrame>>(reused.getDepthFrame().value()), *depthFrame, false);

    dai::RGBDData depthOnly;
    applyBufferMetadata(depthOnly, 12);
    depthOnly.setDepthFrame(depthFrame);
    const auto depthOnlyPath = tempDir.get() / "rgbd_data_depth_only";
    depthOnly.save(depthOnlyPath, false);

    reused.setRGBFrame(std::make_shared<dai::ImgFrame>(makeImgFrame(203)));
    reused.setDepthFrame(std::make_shared<dai::EncodedFrame>(makeEncodedFrame(204)));
    reused.load(depthOnlyPath);
    requireBufferMetadata(reused, depthOnly);
    REQUIRE_FALSE(reused.getRGBFrame().has_value());
    REQUIRE(reused.getDepthFrame().has_value());
    requireEncodedFrame(*std::get<std::shared_ptr<dai::EncodedFrame>>(reused.getDepthFrame().value()), *depthFrame);
}
#endif
