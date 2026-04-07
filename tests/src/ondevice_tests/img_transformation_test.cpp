#include <fmt/base.h>

#include <algorithm>
#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <depthai/depthai.hpp>
#include <depthai/utility/matrixOps.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <mcap/reader.hpp>
#include <memory>
#include <nlohmann/json.hpp>
#include <numeric>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/common/Point2f.hpp"
#include "depthai/depthai.hpp"
#include "depthai/utility/Compression.hpp"
#include "pipeline/utilities/Alignment/AlignmentUtilities.hpp"

bool isIdentity(const std::array<std::array<float, 3>, 3>& mat) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            if(i == j && mat[i][j] != 1) return false;
            if(i != j && mat[i][j] != 0) return false;
        }
    }
    return true;
}

bool approxIdentity(const std::array<std::array<float, 3>, 3>& mat, float eps = 3e-4f) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            float expected = (i == j) ? 1.0f : 0.0f;
            if(std::abs(mat[i][j] - expected) > eps) return false;
        }
    }
    return true;
}

inline std::array<std::array<float, 3>, 3> matmul(std::array<std::array<float, 3>, 3> A, std::array<std::array<float, 3>, 3> B) {
    return {{{A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
              A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
              A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2]},
             {A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
              A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
              A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2]},
             {A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
              A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
              A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2]}}};
}

nlohmann::json loadMetadataJsonByName(const std::filesystem::path& mcapPath, const std::string& metadataName) {
    mcap::McapReader reader;
    auto status = reader.open(mcapPath.string());
    if(!status.ok()) {
        throw std::runtime_error("Failed to open MCAP '" + mcapPath.string() + "': " + status.message);
    }
    status = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
    if(!status.ok()) {
        throw std::runtime_error("Failed to read MCAP summary '" + mcapPath.string() + "': " + status.message);
    }

    const auto [beginIt, endIt] = reader.metadataIndexes().equal_range(metadataName);
    for(auto it = beginIt; it != endIt; ++it) {
        mcap::Record record;
        auto* dataSource = reader.dataSource();
        if(dataSource == nullptr) {
            break;
        }
        status = mcap::McapReader::ReadRecord(*dataSource, it->second.offset, &record);
        if(!status.ok()) {
            continue;
        }
        mcap::Metadata metadata;
        status = mcap::McapReader::ParseMetadata(record, &metadata);
        if(!status.ok()) {
            continue;
        }
        const auto jsonIt = metadata.metadata.find("json");
        if(jsonIt == metadata.metadata.end()) {
            continue;
        }
        return nlohmann::json::parse(jsonIt->second);
    }

    throw std::runtime_error("Metadata record '" + metadataName + "' not found in MCAP: " + mcapPath.string());
}

std::filesystem::path findFile(const std::filesystem::path& folder, const std::string& filename) {
    std::vector<std::filesystem::path> matches;
    for(const auto& entry : std::filesystem::directory_iterator(folder)) {
        if(!entry.is_regular_file()) continue;
        const auto& path = entry.path();
        const auto name = path.filename().string();
        if(name.find(filename) == std::string::npos) continue;
        matches.push_back(path);

        if(entry.path().filename() == filename) {
            return entry.path();
        }
    }

    std::sort(matches.begin(), matches.end());
    if(matches.empty()) {
        throw std::runtime_error("No file matching '" + filename + "' in folder: " + folder.string());
    }
    if(matches.size() > 1) {
        throw std::runtime_error("Multiple files matching '" + filename + "' in folder: " + folder.string());
    }
    return matches.front();
}

std::unique_ptr<mcap::McapReader> openMcap(const std::filesystem::path& mcapPath) {
    auto reader = std::make_unique<mcap::McapReader>();
    auto status = reader->open(mcapPath.string());
    if(!status.ok()) {
        throw std::runtime_error("Failed to open MCAP '" + mcapPath.string() + "': " + status.message);
    }
    return reader;
}

struct CharucoPoint {
    dai::Point2f coordinates;
    std::optional<float> depthMm;
    bool valid;
    int charucoCornerId;
};

struct CharucoPointsMessage {
    uint64_t timestampUs;
    std::vector<CharucoPoint> points;
};

std::vector<CharucoPointsMessage> decodeMcapPoints(const std::filesystem::path& mcapPath, const std::string& expectedTopic) {
    auto reader = openMcap(mcapPath);
    std::vector<CharucoPointsMessage> decodedMessages;

    for(const auto& msg : reader->readMessages()) {
        if(msg.channel == nullptr || msg.channel->topic != expectedTopic) {
            throw std::runtime_error("Unexpected topic in MCAP: " + (msg.channel ? msg.channel->topic : "null"));
        }
        std::string strData(reinterpret_cast<const char*>(msg.message.data), msg.message.dataSize);
        auto jsonFrameMessage = nlohmann::json::parse(strData);

        std::vector<CharucoPoint> decodedPoints;
        for(auto p : jsonFrameMessage["points"]) {
            if(!p.is_object()) {
                std::cout << "Warning: Skipping point that is not an object in frame with timestamp " << jsonFrameMessage["timestamp_us"] << std::endl;
                continue;
            }
            if(!p.contains("charuco_corner_id")) {
                std::cout << "Warning: Skipping point without charuco_corner_id in frame with timestamp " << jsonFrameMessage["timestamp_us"] << std::endl;
                continue;
            }
            if(!p["charuco_corner_id"].is_number_integer()) {
                std::cout << "Warning: Skipping point with non-integer charuco_corner_id in frame with timestamp " << jsonFrameMessage["timestamp_us"]
                          << std::endl;
                continue;
            }
            decodedPoints.push_back({{p["x"], p["y"], false}, p["depth_mm"], p["valid"], p["charuco_corner_id"]});
        }

        decodedMessages.push_back({jsonFrameMessage["timestamp_us"], decodedPoints});
    }

    return decodedMessages;
}

std::vector<CharucoPoint> projectPoints(const std::vector<CharucoPoint>& referencePoints,
                                        const dai::ImgTransformation& refTransform,
                                        const dai::ImgTransformation& targetTransform) {
    std::vector<CharucoPoint> projectedPoints;
    for(auto p : referencePoints) {
        if(!p.valid || p.depthMm < 0.0f) {
            continue;
        }

        const auto projectedPoint = refTransform.projectPointTo(targetTransform, p.coordinates, p.depthMm.value());
        projectedPoints.push_back({projectedPoint, p.depthMm, p.valid, p.charucoCornerId});
    }

    return projectedPoints;
}

std::vector<float> calculateProjectionErrors(const std::vector<CharucoPoint>& projectedPoints, const std::vector<CharucoPoint>& targetPoints) {
    std::vector<float> errors;
    for(const auto& projected : projectedPoints) {
        auto it = std::find_if(targetPoints.begin(), targetPoints.end(), [&](const CharucoPoint& p) { return p.charucoCornerId == projected.charucoCornerId; });
        if(it != targetPoints.end()) {
            const float error = std::hypot(projected.coordinates.x - it->coordinates.x, projected.coordinates.y - it->coordinates.y);
            errors.push_back(error);
        }
    }
    return errors;
}

float calculateMeanError(const std::vector<float>& errors) {
    if(errors.empty()) return -1.0f;
    float sum = std::accumulate(errors.begin(), errors.end(), 0.0f);
    return sum / static_cast<float>(errors.size());
}

float calculateMedian(const std::vector<float>& errors) {
    if(errors.empty()) return -1.0f;
    std::vector<float> sorted = errors;
    std::sort(sorted.begin(), sorted.end());
    size_t mid = sorted.size() / 2;
    return (sorted.size() % 2 == 0) ? 0.5f * (sorted[mid - 1] + sorted[mid]) : sorted[mid];
}

float calculateStdDev(const std::vector<float>& errors, float mean) {
    if(errors.empty()) return -1.0f;
    float variance = 0.0f;
    for(float e : errors) {
        float delta = e - mean;
        variance += delta * delta;
    }
    variance /= static_cast<float>(errors.size());
    return std::sqrt(variance);
}

std::tuple<dai::ImgTransformation, dai::ImgTransformation> parseTransformations(const std::filesystem::path& refMetadataPath,
                                                                                const std::filesystem::path& targetMetadataPath) {
    dai::Pipeline pipeline{false};

    auto referenceMetadataReplayNode = pipeline.create<dai::node::ReplayMetadataOnly>();
    referenceMetadataReplayNode->setReplayFile(refMetadataPath);
    referenceMetadataReplayNode->setLoop(false);

    auto targetMetadataReplayNode = pipeline.create<dai::node::ReplayMetadataOnly>();
    targetMetadataReplayNode->setReplayFile(targetMetadataPath);
    targetMetadataReplayNode->setLoop(false);

    auto refQueue = referenceMetadataReplayNode->out.createOutputQueue(1, true);
    auto targetQueue = targetMetadataReplayNode->out.createOutputQueue(1, true);

    pipeline.start();

    auto refFrameMessage = refQueue->get<dai::ImgFrame>();
    auto targetFrameMessage = targetQueue->get<dai::ImgFrame>();

    auto refImgTransformation = refFrameMessage->transformation;
    auto targetImgTransformation = targetFrameMessage->transformation;

    return {refImgTransformation, targetImgTransformation};
}

nlohmann::json processVideo(const std::filesystem::path& videoPath) {
    const auto referencePointsPath = findFile(videoPath, "reference_points.mcap");
    const auto targetPointsPath = findFile(videoPath, "target_points.mcap");
    const auto calibrationPath = findFile(videoPath, "calibration.json");
    const auto referenceMetadataPath = findFile(videoPath, "reference_metadata.mcap");
    const auto targetMetadataPath = findFile(videoPath, "target_metadata.mcap");

    auto referencePoints = decodeMcapPoints(referencePointsPath, "reference_points");
    auto targetPoints = decodeMcapPoints(targetPointsPath, "target_points");

    if(referencePoints.size() != targetPoints.size()) {
        throw std::runtime_error("Reference and target point frame counts differ");
    }

    auto [refImgTransformation, targetImgTransformation] = parseTransformations(referenceMetadataPath, targetMetadataPath);

    std::vector<float> errors;
    for(int i = 0; i < referencePoints.size(); ++i) {
        const auto& referencePointsFrame = referencePoints[i];
        const auto& targetPointsFrame = targetPoints[i];

        auto projectedPoints = projectPoints(referencePointsFrame.points, refImgTransformation, targetImgTransformation);
        auto frameErrors = calculateProjectionErrors(projectedPoints, targetPointsFrame.points);
        errors.insert(errors.end(), frameErrors.begin(), frameErrors.end());
    }

    float meanError = calculateMeanError(errors);
    float medianError = calculateMedian(errors);
    float stdError = calculateStdDev(errors, meanError);

    auto calibrationFile = nlohmann::json::parse(std::ifstream(calibrationPath));
    const auto device = calibrationFile.value("productName", "unknown");

    const auto refConfig = loadMetadataJsonByName(referencePointsPath, "camera_config");
    const auto refSensor = refConfig.value("sensor", "unknown");
    const auto refSize = refConfig.value("size", nlohmann::json::object());
    const auto refIsRectified = refConfig.value("is_rectified", false);
    const auto refSocket = refConfig.value("socket", "unknown");
    const auto refUndistortion = refConfig.value("undistortion_enabled", false);

    const auto targetConfig = loadMetadataJsonByName(targetPointsPath, "camera_config");
    const auto targetSize = targetConfig.value("size", nlohmann::json::object());
    const auto targetIsRectified = targetConfig.value("is_rectified", false);
    const auto targetSensor = targetConfig.value("sensor", "unknown");
    const auto targetSocket = targetConfig.value("socket", "unknown");
    const auto targetUndistortion = targetConfig.value("undistortion_enabled", false);

    return nlohmann::json::object({
        {"device", device},
        {"ref_sensor", refSensor},
        {"target_sensor", targetSensor},
        {"ref_cam", refSocket},
        {"target_cam", targetSocket},
        {"ref_undistortion", refUndistortion},
        {"target_undistortion", targetUndistortion},
        {"ref_size", refSize},
        {"target_size", targetSize},
        {"ref_is_rectified", refIsRectified},
        {"target_is_rectified", targetIsRectified},
        {"mean_projection_error_px", meanError},
        {"median_projection_error_px", medianError},
        {"std_projection_error_px", stdError},
        {"projection_error_count", static_cast<int>(errors.size())},
        {"processed_frames", static_cast<int>(referencePoints.size())},
    });
}

std::filesystem::path extractTransformationTestDataFolder() {
    const std::filesystem::path archivePath{TRANSFORMATION_TEST_DATA};
    if(!std::filesystem::exists(archivePath)) {
        return {};
    }
    const std::filesystem::path tempFolder = std::filesystem::temp_directory_path() / "depthai_img_transformations_test_data";
    std::filesystem::remove_all(tempFolder);
    std::filesystem::create_directories(tempFolder);

    const auto archiveEntries = dai::utility::filenamesInArchive(archivePath);
    std::vector<std::string> filesInArchive;
    std::vector<std::filesystem::path> extractedFiles;
    filesInArchive.reserve(archiveEntries.size());
    extractedFiles.reserve(archiveEntries.size());

    for(const auto& entry : archiveEntries) {
        if(entry.empty()) continue;

        const std::filesystem::path outputPath = tempFolder / entry;
        if(entry.back() == '/' || outputPath.filename().string().empty()) {
            std::filesystem::create_directories(outputPath);
            continue;
        }

        std::filesystem::create_directories(outputPath.parent_path());
        filesInArchive.push_back(entry);
        extractedFiles.push_back(outputPath);
    }

    if(!filesInArchive.empty()) {
        dai::utility::extractFiles(archivePath, filesInArchive, extractedFiles);
    }

    const std::filesystem::path extractedRoot = tempFolder / archivePath.stem().stem();
    if(std::filesystem::exists(extractedRoot)) {
        return extractedRoot;
    }

    return tempFolder;
}

const std::filesystem::path& getTransformationTestDataFolder() {
    static const std::filesystem::path baseFolder = extractTransformationTestDataFolder();
    return baseFolder;
}

// -----------------------------------------------------------------------------
// ImgTransformation in ImgFrame
// Purpose:
//   Ensures that ISP output frames directly produced on the device contain a
//   valid and non-identity ImgTransformation. This confirms that device-side
//   generation of intrinsic matrices, scaling factors, and normalization is
//   functioning correctly.
//
//   Specifically verifies:
//     • validateTransformations() succeeds on device
//     • Forward matrix M is not identity
//     • Inverse matrix M⁻¹ is not identity
//     • Source intrinsic matrices K and K⁻¹ are populated correctly
//
//   Device-side correctness is crucial because these transforms are generated
//   by firmware and must be trusted downstream by NN nodes.
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation in ImgFrame") {
    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput({600, 400}, dai::ImgFrame::Type::NV12);
    auto q = camOut->createOutputQueue();
    pipeline.start();
    auto frame = q->get<dai::ImgFrame>();
    REQUIRE(frame != nullptr);
    pipeline.stop();
    REQUIRE(frame->validateTransformations());
    REQUIRE(!isIdentity(frame->transformation.getMatrix()));
    REQUIRE(!isIdentity(frame->transformation.getMatrixInv()));
    REQUIRE(!isIdentity(frame->transformation.getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(frame->transformation.getSourceIntrinsicMatrixInv()));
}

// -----------------------------------------------------------------------------
// ImgTransformation in SpatialDetectionNetwork
// Purpose:
//   Confirms that SpatialDetectionNetwork running *on the device* retains and
//   propagates ImgTransformation metadata from both the RGB camera and the
//   StereoDepth node.
//
//   This test validates that:
//     • SpatialImgDetections include a valid transformation
//     • Intrinsics/extrinsics are preserved correctly across nodes
//     • The output matrices are NOT identity matrices
//
//   This is essential because 3D bounding boxes rely on projecting 2D NN
//   detections back into disparate camera spaces.
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation in SpatialDetectionNetwork") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto left = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto right = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);
    auto stereo = pipeline.create<dai::node::StereoDepth>()->build(*left->requestFullResolutionOutput(), *right->requestFullResolutionOutput());
    stereo->setSubpixel(false);
    dai::NNModelDescription modelDesc{"yolov6-nano"};
    // Load NNArchive
    auto nn = pipeline.create<dai::node::SpatialDetectionNetwork>();
    REQUIRE_NOTHROW(nn->build(camera, stereo, modelDesc));
    auto detectionsQueue = nn->out.createOutputQueue();
    pipeline.start();
    auto tensor = detectionsQueue->get<dai::SpatialImgDetections>();
    REQUIRE(tensor != nullptr);
    pipeline.stop();
    REQUIRE(tensor->transformation.has_value());
    REQUIRE(tensor->transformation->isValid());
    REQUIRE(!isIdentity(tensor->transformation->getMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getMatrixInv()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrixInv()));
}

// -----------------------------------------------------------------------------
// ImgTransformation in DetectionNetwork
// Purpose:
//   Ensures that the standard DetectionNetwork (2D detection without depth)
//   also preserves ImgTransformation metadata end-to-end on device.
//
//   This guarantees that detections can be accurately mapped back to sensor
//   coordinates or shared across heterogeneous streams (e.g., RGB + IR).
//
//   Verifies:
//     • transformation.has_value()
//     • transformation.isValid()
//     • No matrix returned is an identity matrix
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation in DetectionNetwork") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    dai::NNModelDescription modelDesc{"yolov6-nano"};
    // Load NNArchive
    auto nn = pipeline.create<dai::node::DetectionNetwork>()->build(camera, modelDesc);
    auto detectionsQueue = nn->out.createOutputQueue();
    pipeline.start();
    auto tensor = detectionsQueue->get<dai::ImgDetections>();
    REQUIRE(tensor != nullptr);
    pipeline.stop();
    REQUIRE(tensor->transformation.has_value());
    REQUIRE(tensor->transformation->isValid());
    REQUIRE(!isIdentity(tensor->transformation->getMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getMatrixInv()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrixInv()));
}

// -----------------------------------------------------------------------------
// ImgTransformation in NeuralNetwork
// Purpose:
//   Validates the generic NeuralNetwork node's ability to propagate the camera
//   transformation metadata without modification.
//
//   Many custom models require correct mapping back to original image space.
//   This test ensures that the device firmware always attaches the correct
//   transformation metadata to NNData outputs.
//
//   As with the other tests:
//     • Matrices must be valid
//     • None may be identity
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation in NeuralNetwork") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    dai::NNModelDescription modelDesc{"yolov6-nano"};
    // Load NNArchive
    auto nn = pipeline.create<dai::node::NeuralNetwork>()->build(camera, modelDesc);
    auto detectionsQueue = nn->out.createOutputQueue();
    pipeline.start();
    auto tensor = detectionsQueue->get<dai::NNData>();
    REQUIRE(tensor != nullptr);
    pipeline.stop();
    REQUIRE(tensor->transformation.has_value());
    REQUIRE(tensor->transformation->isValid());
    REQUIRE(!isIdentity(tensor->transformation->getMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getMatrixInv()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrix()));
    REQUIRE(!isIdentity(tensor->transformation->getSourceIntrinsicMatrixInv()));
}

// -----------------------------------------------------------------------------
// ImgTransformation remap vertical
// Purpose:
//   Validates device-side geometric consistency when mapping ROIs between two
//   camera outputs of different orientations and aspect ratios:
//
//       Stream A: 600 × 400   (landscape)
//       Stream B: 400 × 600   (portrait)
//
//   The test checks that:
//     • remapRectTo() preserves aspect ratio
//     • Rect denormalization yields geometrically consistent output
//     • The transformation system handles 90-degree orientation changes
//       produced by ISP scaling paths.
//
//   Ensures robust cross-stream remapping for pipelines relying on:
//     • Multi-angle inference
//     • Simultaneous portrait/landscape streams
//     • Stream synchronization or fusion
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation remap vertical") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto camOut1 = camera->requestOutput({600, 400});
    auto camOut2 = camera->requestOutput({400, 600});
    auto q1 = camOut1->createOutputQueue();
    auto q2 = camOut2->createOutputQueue();
    pipeline.start();
    auto frame1 = q1->get<dai::ImgFrame>();
    REQUIRE(frame1 != nullptr);
    auto frame2 = q2->get<dai::ImgFrame>();
    REQUIRE(frame2 != nullptr);
    auto rect = dai::Rect(200, 100, 100, 200);
    auto rRect = dai::RotatedRect(rect.normalize(frame1->getWidth(), frame1->getHeight()), 0.f);
    auto remapped = frame1->transformation.remapRectTo(frame2->transformation, rRect);

    auto sourceDen = rRect.denormalize(frame1->getWidth(), frame1->getHeight());
    auto destDen = remapped.denormalize(frame2->getWidth(), frame2->getHeight());
    auto sourceAR = sourceDen.size.width / sourceDen.size.height;
    auto destAR = destDen.size.width / destDen.size.height;

    REQUIRE_THAT(sourceAR, Catch::Matchers::WithinAbs(destAR, 0.01));

    pipeline.stop();
}

// -----------------------------------------------------------------------------
// ImgTransformation isAlignedTo
// Purpose:
//   Validates alignment checks between transformations. Ensures that
//   transformations from the same stream are aligned, while transformations
//   from differently-sized outputs are not.
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation isAlignedTo") {
    dai::Pipeline pipeline;
    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto alignedOut = camera->requestOutput({640, 480});
    auto misalignedOut = camera->requestOutput({320, 240});
    auto alignedQueue = alignedOut->createOutputQueue();
    auto misalignedQueue = misalignedOut->createOutputQueue();
    pipeline.start();
    auto alignedFrame = alignedQueue->get<dai::ImgFrame>();
    auto misalignedFrame = misalignedQueue->get<dai::ImgFrame>();
    REQUIRE(alignedFrame != nullptr);
    REQUIRE(misalignedFrame != nullptr);
    pipeline.stop();

    REQUIRE(alignedFrame->transformation.isValid());
    REQUIRE(misalignedFrame->transformation.isValid());
    REQUIRE(alignedFrame->transformation.isAlignedTo(alignedFrame->transformation));
    REQUIRE_FALSE(alignedFrame->transformation.isAlignedTo(misalignedFrame->transformation));
    REQUIRE_FALSE(misalignedFrame->transformation.isAlignedTo(alignedFrame->transformation));
}

// -----------------------------------------------------------------------------
// ImgTransformation matrix inverse consistency (ImgFrame)
// Purpose:
//   Ensures that the forward matrix (M) and its stored inverse (Minv)
//   multiply to identity. Same for intrinsics (K * Kinv).
//   Validates correctness of ImgTransformation’s internal math.
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation matrix inverse consistency (ImgFrame)") {
    dai::Pipeline pipeline;
    auto cam = pipeline.create<dai::node::Camera>()->build();
    auto camOut = cam->requestOutput({1280, 800}, dai::ImgFrame::Type::NV12);
    auto q = camOut->createOutputQueue();
    pipeline.start();
    auto frame = q->get<dai::ImgFrame>();
    REQUIRE(frame != nullptr);
    pipeline.stop();

    REQUIRE(frame->transformation.isValid());

    auto M = frame->transformation.getMatrix();
    auto Minv = frame->transformation.getMatrixInv();
    auto K = frame->transformation.getSourceIntrinsicMatrix();
    auto Kinv = frame->transformation.getSourceIntrinsicMatrixInv();

    auto I1 = matmul(M, Minv);
    auto I2 = matmul(Minv, M);
    auto I3 = matmul(K, Kinv);
    auto I4 = matmul(Kinv, K);

    REQUIRE(approxIdentity(I1));
    REQUIRE(approxIdentity(I2));
    REQUIRE(approxIdentity(I3));
    REQUIRE(approxIdentity(I4));
}

// -----------------------------------------------------------------------------
// ImgTransformation isAlignedTo distortion coefficients handling
// Purpose:
//   Ensures isAlignedTo treats missing distortion coefficients as zeros while
//   still detecting real mismatches.
// -----------------------------------------------------------------------------
TEST_CASE("ImgTransformation isAlignedTo distortion coefficients handling") {
    dai::ImgTransformation base(640, 480);
    dai::ImgTransformation zeros(640, 480);
    dai::ImgTransformation nonZero(640, 480);

    base.setDistortionCoefficients({});
    zeros.setDistortionCoefficients({0.0f, 0.0f, 0.0f, 0.0f, 0.0f});
    nonZero.setDistortionCoefficients({0.0f, 0.0f, 0.0f, 0.0f, 0.01f});

    REQUIRE(base.isAlignedTo(zeros));
    REQUIRE_FALSE(base.isAlignedTo(nonZero));
}

TEST_CASE("AlignmentUtilities distort point") {
    dai::Point3f point3D{40, 75, 150.0f};
    cv::Point3f point3Dcv(point3D.x, point3D.y, point3D.z);
    std::vector<cv::Point3f> pointCloud{point3Dcv};

    std::array<std::array<float, 3>, 3> cameraMatrixValues{{
        {857.48296979, 0.0, 968.06224829},
        {0.0, 876.71824265, 556.37145899},
        {0.0, 0.0, 1.0},
    }};
    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 857.48296979, 0.0, 968.06224829, 0.0, 876.71824265, 556.37145899, 0.0, 0.0, 1.0);

    std::vector<float> distCoeffs{20.43730926513672,
                                  -17.942808151245117,
                                  -0.015188916586339474,
                                  0.0008560882997699082,
                                  2.0712976455688477,
                                  15.986123085021973,
                                  -16.679258346557617,
                                  1.121966004371643,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  -0.006426393520087004,
                                  0.006430431269109249};

    dai::ImgTransformation transformation{
        1920,
        1080,
        cameraMatrixValues,
        dai::CameraModel::Perspective,
        distCoeffs,
    };

    SECTION("Perspective distortion") {
        cv::Mat distortionCoeffsCv = cv::Mat(distCoeffs).reshape(1, 1);

        dai::Point2f projectedPoint = transformation.project3DPoint(point3D);
        std::vector<cv::Point2f> projected;
        cv::projectPoints(pointCloud, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0), cameraMatrix, distortionCoeffsCv, projected);

        INFO("Projected point with AlignmentUtilities: " << projectedPoint.x << ", " << projectedPoint.y);
        INFO("Projected point with OpenCV: " << projected[0].x << ", " << projected[0].y);
        REQUIRE(std::hypot(projectedPoint.x - projected[0].x, projectedPoint.y - projected[0].y) < 1e-3);
    }

    SECTION("Fisheye distortion") {
        std::vector<float> distCoeffs{0.5f, -0.11f, 0.331f, -0.0001f};
        cv::Mat distortionCoeffsCv;
        cv::Mat(distCoeffs).reshape(1, 1).convertTo(distortionCoeffsCv, CV_64F);

        transformation.setDistortionModel(dai::CameraModel::Fisheye);
        transformation.setDistortionCoefficients(distCoeffs);

        dai::Point2f projectedPoint = transformation.project3DPoint(point3D);

        std::vector<cv::Point2f> projected;
        cv::fisheye::projectPoints(pointCloud, projected, cv::Vec3d(0.0, 0.0, 0.0), cv::Vec3d(0.0, 0.0, 0.0), cameraMatrix, distortionCoeffsCv);

        INFO("Projected point with AlignmentUtilities: " << projectedPoint.x << ", " << projectedPoint.y);
        INFO("Projected point with OpenCV: " << projected[0].x << ", " << projected[0].y);
        REQUIRE(std::hypot(projectedPoint.x - projected[0].x, projectedPoint.y - projected[0].y) < 1e-3);
    }

    SECTION("Unsupported Camera models") {
        transformation.setDistortionModel(dai::CameraModel::RadialDivision);
        REQUIRE_THROWS_AS(transformation.project3DPoint(point3D), std::invalid_argument);
        transformation.setDistortionModel(dai::CameraModel::Equirectangular);
        REQUIRE_THROWS_AS(transformation.project3DPoint(point3D), std::invalid_argument);
    }
}

TEST_CASE("AlignmentUtilities undistort point") {
    dai::Point2f point{1594.8793471442408, 200.6646247524987};
    cv::Point2d distortedPoint(point.x, point.y);

    std::vector<float> distCoeffs{20.43730926513672,
                                  -17.942808151245117,
                                  -0.015188916586339474,
                                  0.0008560882997699082,
                                  2.0712976455688477,
                                  15.986123085021973,
                                  -16.679258346557617,
                                  1.121966004371643,
                                  0.0,
                                  0.0,
                                  0.0,
                                  0.0,
                                  -0.006426393520087004,
                                  0.006430431269109249};
    cv::Mat distortionCoeffsCv = cv::Mat(distCoeffs).reshape(1, 1);

    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 857.48296979, 0.0, 968.06224829, 0.0, 876.71824265, 556.37145899, 0.0, 0.0, 1.0);

    std::array<std::array<float, 3>, 3> cameraMatrixValues{{
        {857.48296979, 0.0, 968.06224829},
        {0.0, 876.71824265, 556.37145899},
        {0.0, 0.0, 1.0},
    }};

    dai::ImgTransformation transformation{
        1920,
        1080,
        cameraMatrixValues,
        dai::CameraModel::Perspective,
        distCoeffs,
    };

    SECTION("Perspective undistortion") {
        const auto undistortedRay = pixelToRay(point, transformation);
        std::vector<cv::Point2d> opencvUndistorted;
        cv::undistortPoints(std::vector<cv::Point2d>{distortedPoint},
                            opencvUndistorted,
                            cameraMatrix,
                            distortionCoeffsCv,
                            cv::noArray(),
                            cv::noArray(),
                            {cv::TermCriteria::MAX_ITER, 50, 0.0001});

        INFO("Undistorted point with AlignmentUtilities: " << undistortedRay[0] / undistortedRay[2] << ", " << undistortedRay[1] / undistortedRay[2]);
        INFO("Undistorted point with OpenCV: " << opencvUndistorted[0].x << ", " << opencvUndistorted[0].y);
        REQUIRE(std::hypot((undistortedRay[0] / undistortedRay[2]) - opencvUndistorted[0].x, (undistortedRay[1] / undistortedRay[2]) - opencvUndistorted[0].y)
                < 1e-3);
    }

    SECTION("Fisheye undistortion") {
        std::vector<float> distCoeffs{0.5f, -0.11f, 0.331f, -0.0001f};
        cv::Mat distortionCoeffsCv;
        cv::Mat(distCoeffs).reshape(1, 1).convertTo(distortionCoeffsCv, CV_64F);

        transformation.setDistortionModel(dai::CameraModel::Fisheye);
        transformation.setDistortionCoefficients(distCoeffs);

        const auto undistortedRay = pixelToRay(point, transformation);
        std::vector<cv::Point2d> opencvUndistorted;
        cv::fisheye::undistortPoints(std::vector<cv::Point2d>{distortedPoint}, opencvUndistorted, cameraMatrix, distortionCoeffsCv);

        INFO("Undistorted point with AlignmentUtilities: " << undistortedRay[0] / undistortedRay[2] << ", " << undistortedRay[1] / undistortedRay[2]);
        INFO("Undistorted point with OpenCV: " << opencvUndistorted[0].x << ", " << opencvUndistorted[0].y);
        REQUIRE(std::hypot((undistortedRay[0] / undistortedRay[2]) - opencvUndistorted[0].x, (undistortedRay[1] / undistortedRay[2]) - opencvUndistorted[0].y)
                < 1e-3);
    }

    SECTION("Unsupported Camera models") {
        transformation.setDistortionModel(dai::CameraModel::RadialDivision);
        REQUIRE_THROWS_AS(pixelToRay(point, transformation), std::invalid_argument);
        transformation.setDistortionModel(dai::CameraModel::Equirectangular);
        REQUIRE_THROWS_AS(pixelToRay(point, transformation), std::invalid_argument);
    }
}

TEST_CASE("projectPoints test") {
    const std::filesystem::path& baseFolder = getTransformationTestDataFolder();

    if(!std::filesystem::exists(baseFolder)) {
        WARN("Capture folder not found, skipping projectPoints test: " << baseFolder.string());
        return;
    }

    nlohmann::json currentResults;
    int testIterator = 0;
    for(const auto& directory : std::filesystem::directory_iterator(baseFolder)) {
        if(!directory.is_directory()) continue;
        const auto capturePath = directory.path();
        const std::filesystem::path folder = std::filesystem::path(capturePath);
        const auto result = processVideo(folder);
        INFO(result.dump(2));
        const int errorCount = result.value("projection_error_count", 0);
        const double meanProjectionErrorPx = result.value("mean_projection_error_px", std::numeric_limits<double>::quiet_NaN());
        const double medianProjectionErrorPx = result.value("median_projection_error_px", std::numeric_limits<double>::quiet_NaN());
        const double stdProjectionErrorPx = result.value("std_projection_error_px", std::numeric_limits<double>::quiet_NaN());

        REQUIRE(errorCount > 0);
        REQUIRE(std::isfinite(meanProjectionErrorPx));
        REQUIRE(std::isfinite(medianProjectionErrorPx));
        REQUIRE(std::isfinite(stdProjectionErrorPx));

        currentResults[folder.stem().string()] = result;
        testIterator++;
    }

    REQUIRE(testIterator > 0);

    const std::filesystem::path outputPath = baseFolder / "aggregated_results.json";
    std::ifstream inputFile(outputPath);
    REQUIRE(inputFile.is_open());

    const auto aggregatedResults = nlohmann::json::parse(inputFile);
    REQUIRE(aggregatedResults.is_object());

    for(const auto& [captureName, currentResult] : currentResults.items()) {
        INFO("Comparing results for capture: " << captureName);
        REQUIRE(aggregatedResults.contains(captureName));

        const auto& aggregatedResult = aggregatedResults.at(captureName);
        const double currentMeanProjectionErrorPx = currentResult.value("mean_projection_error_px", std::numeric_limits<double>::infinity());
        const double aggregatedMeanProjectionErrorPx = aggregatedResult.value("mean_projection_error_px", std::numeric_limits<double>::quiet_NaN());

        REQUIRE(std::isfinite(currentMeanProjectionErrorPx));
        REQUIRE(std::isfinite(aggregatedMeanProjectionErrorPx));
        REQUIRE(currentMeanProjectionErrorPx <= aggregatedMeanProjectionErrorPx);
    }
}
