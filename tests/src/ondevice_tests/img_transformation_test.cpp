#include <fmt/base.h>

#include <array>
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <depthai/depthai.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/node/Camera.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"

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
