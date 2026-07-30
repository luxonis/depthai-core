#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/ImgFrame.hpp>
#include <depthai/pipeline/node/host/Stitching.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <utility>
#include <vector>

namespace {

// A virtual rig looking at a textured ground plane: cameras mounted at the same height, pitched down and yawed apart,
// so that their footprints on the ground overlap only partially. The reference frame of the whole dataset is the frame
// of the first camera, the way CoordinateFrameTransform leaves the streams of a real multi-device setup.
constexpr int CAMERA_WIDTH = 640;
constexpr int CAMERA_HEIGHT = 400;
constexpr double CAMERA_FOCAL = 380.0;
constexpr double MOUNT_HEIGHT = 100.0;  // cm above the ground
constexpr double MOUNT_PITCH = 35.0;    // degrees below the horizon

constexpr int TEXTURE_SIZE = 1600;     // pixels
constexpr double TEXTURE_SCALE = 0.5;  // cm per texture pixel
constexpr double MARKER_SIZE = 24.0;   // cm
constexpr int MARKER_TOLERANCE = 8;    // pixels between the expected and the rendered marker center
constexpr int COLOR_TOLERANCE = 40;    // per channel

const dai::CoordinateFrame REFERENCE_FRAME{"virtualrig", dai::CameraBoardSocket::CAM_A};

/// A patch of a distinct color painted on the ground, used to check where the plane ends up in the rendered image.
struct Marker {
    cv::Vec3b color;
    cv::Point3d position;
};

const std::vector<Marker> MARKERS = {
    {{0, 0, 255}, {0.0, 150.0, 0.0}},
    {{0, 255, 0}, {-90.0, 110.0, 0.0}},
    {{255, 0, 0}, {90.0, 110.0, 0.0}},
    {{0, 255, 255}, {0.0, 240.0, 0.0}},
};

cv::Point2d worldToTexture(const cv::Point3d& point) {
    return {point.x / TEXTURE_SCALE + TEXTURE_SIZE / 2.0, point.y / TEXTURE_SCALE + TEXTURE_SIZE / 2.0};
}

/// Ground texture: blocks of random color, so that every part of the plane looks different, plus the markers.
cv::Mat renderGround() {
    cv::Mat texture(TEXTURE_SIZE, TEXTURE_SIZE, CV_8UC3);
    cv::RNG rng(20240517);
    constexpr int BLOCK = 20;
    for(int y = 0; y < TEXTURE_SIZE; y += BLOCK) {
        for(int x = 0; x < TEXTURE_SIZE; x += BLOCK) {
            const cv::Rect block(x, y, std::min(BLOCK, TEXTURE_SIZE - x), std::min(BLOCK, TEXTURE_SIZE - y));
            texture(block).setTo(cv::Scalar(rng.uniform(0, 200), rng.uniform(0, 200), rng.uniform(0, 200)));
        }
    }

    for(const auto& marker : MARKERS) {
        const auto center = worldToTexture(marker.position);
        const auto half = 0.5 * MARKER_SIZE / TEXTURE_SCALE;
        cv::rectangle(texture,
                      cv::Point2d(center.x - half, center.y - half),
                      cv::Point2d(center.x + half, center.y + half),
                      cv::Scalar(marker.color[0], marker.color[1], marker.color[2]),
                      cv::FILLED);
    }
    return texture;
}

struct SyntheticCamera {
    /// Camera to world rotation, in the depthai convention: x right, y down, z along the optical axis.
    cv::Matx33d rotation;
    /// Camera center in the world.
    cv::Vec3d center;
    cv::Matx33d intrinsics;
};

SyntheticCamera makeCamera(double x, double y, double yawDegrees) {
    const double yaw = yawDegrees * CV_PI / 180.0;
    const double pitch = MOUNT_PITCH * CV_PI / 180.0;

    const cv::Vec3d forward(std::sin(yaw) * std::cos(pitch), std::cos(yaw) * std::cos(pitch), -std::sin(pitch));
    const cv::Vec3d right(std::cos(yaw), -std::sin(yaw), 0.0);
    const cv::Vec3d down = forward.cross(right);

    SyntheticCamera camera;
    camera.rotation = cv::Matx33d(right[0], down[0], forward[0], right[1], down[1], forward[1], right[2], down[2], forward[2]);
    camera.center = cv::Vec3d(x, y, MOUNT_HEIGHT);
    camera.intrinsics = cv::Matx33d(CAMERA_FOCAL, 0.0, 0.5 * (CAMERA_WIDTH - 1), 0.0, CAMERA_FOCAL, 0.5 * (CAMERA_HEIGHT - 1), 0.0, 0.0, 1.0);
    return camera;
}

/// What the camera sees of the ground, the ground being the z == 0 plane the texture is painted on.
cv::Mat renderView(const cv::Mat& texture, const SyntheticCamera& camera) {
    const cv::Matx33d worldToCamera = camera.rotation.t();
    // Columns of the texture pixel to world point mapping, the third one being the origin of the texture
    const cv::Vec3d columnX = worldToCamera * cv::Vec3d(TEXTURE_SCALE, 0.0, 0.0);
    const cv::Vec3d columnY = worldToCamera * cv::Vec3d(0.0, TEXTURE_SCALE, 0.0);
    const cv::Vec3d origin = worldToCamera * (cv::Vec3d(-0.5 * TEXTURE_SIZE * TEXTURE_SCALE, -0.5 * TEXTURE_SIZE * TEXTURE_SCALE, 0.0) - camera.center);

    const cv::Matx33d homography =
        camera.intrinsics * cv::Matx33d(columnX[0], columnY[0], origin[0], columnX[1], columnY[1], origin[1], columnX[2], columnY[2], origin[2]);

    cv::Mat view;
    cv::warpPerspective(texture, view, cv::Mat(homography), cv::Size(CAMERA_WIDTH, CAMERA_HEIGHT), cv::INTER_LINEAR);
    return view;
}

std::array<std::array<float, 4>, 4> toTransformationMatrix(const cv::Matx33d& rotation, const cv::Vec3d& translation) {
    std::array<std::array<float, 4>, 4> matrix{};
    for(int row = 0; row < 3; ++row) {
        for(int column = 0; column < 3; ++column) {
            matrix[row][column] = static_cast<float>(rotation(row, column));
        }
        matrix[row][3] = static_cast<float>(translation[row]);
    }
    matrix[3] = {0.0f, 0.0f, 0.0f, 1.0f};
    return matrix;
}

std::array<std::array<float, 3>, 3> toIntrinsicMatrix(const cv::Matx33d& intrinsics) {
    std::array<std::array<float, 3>, 3> matrix{};
    for(int row = 0; row < 3; ++row) {
        for(int column = 0; column < 3; ++column) {
            matrix[row][column] = static_cast<float>(intrinsics(row, column));
        }
    }
    return matrix;
}

/// The synthetic dataset: images plus the calibration a real pipeline would carry in the messages.
struct Dataset {
    std::vector<cv::Mat> images;
    std::vector<dai::ImgTransformation> transformations;
    /// World to reference frame transform, the reference frame being the frame of the first camera.
    cv::Matx33d worldToReferenceRotation;
    cv::Vec3d worldToReferenceTranslation;

    cv::Vec3d toReference(const cv::Point3d& worldPoint) const {
        return worldToReferenceRotation * cv::Vec3d(worldPoint.x, worldPoint.y, worldPoint.z) + worldToReferenceTranslation;
    }

    dai::Point3f planePoint() const {
        const auto point = toReference({0.0, 0.0, 0.0});
        return {static_cast<float>(point[0]), static_cast<float>(point[1]), static_cast<float>(point[2])};
    }

    dai::Point3f planeNormal() const {
        const cv::Vec3d normal = worldToReferenceRotation * cv::Vec3d(0.0, 0.0, 1.0);
        return {static_cast<float>(normal[0]), static_cast<float>(normal[1]), static_cast<float>(normal[2])};
    }
};

Dataset renderDataset(const std::vector<SyntheticCamera>& cameras) {
    const cv::Mat texture = renderGround();

    Dataset dataset;
    dataset.worldToReferenceRotation = cameras.front().rotation.t();
    dataset.worldToReferenceTranslation = -(cameras.front().rotation.t() * cameras.front().center);

    for(const auto& camera : cameras) {
        dataset.images.push_back(renderView(texture, camera));

        // Pose of the camera in the reference frame, which is what the extrinsics of a message carry
        const cv::Matx33d rotation = dataset.worldToReferenceRotation * camera.rotation;
        const cv::Vec3d translation = dataset.worldToReferenceRotation * camera.center + dataset.worldToReferenceTranslation;

        dai::Extrinsics extrinsics;
        extrinsics.setTransformationMatrix(toTransformationMatrix(rotation, translation), dai::LengthUnit::CENTIMETER);
        extrinsics.setReferenceFrame(REFERENCE_FRAME);
        dataset.transformations.emplace_back(
            CAMERA_WIDTH, CAMERA_HEIGHT, toIntrinsicMatrix(camera.intrinsics), dai::CameraModel::Perspective, std::vector<float>{}, extrinsics);
    }
    return dataset;
}

std::shared_ptr<dai::ImgFrame> toFrame(const cv::Mat& image, const dai::ImgTransformation& transformation, int64_t sequenceNum) {
    auto frame = std::make_shared<dai::ImgFrame>();
    frame->setCvFrame(image, dai::ImgFrame::Type::BGR888i);
    frame->setSequenceNum(sequenceNum);
    frame->setTimestamp(std::chrono::steady_clock::now());
    frame->getTransformation() = transformation;
    return frame;
}

/// Where a point of the reference frame lands in an image described by `transformation`.
cv::Point2d project(const dai::ImgTransformation& transformation, const cv::Vec3d& referencePoint) {
    const auto pose = transformation.getExtrinsics().getTransformationMatrix(false, dai::LengthUnit::CENTIMETER);
    const cv::Matx33d rotation(pose[0][0], pose[0][1], pose[0][2], pose[1][0], pose[1][1], pose[1][2], pose[2][0], pose[2][1], pose[2][2]);
    const cv::Vec3d center(pose[0][3], pose[1][3], pose[2][3]);
    const cv::Vec3d inCamera = rotation.t() * (referencePoint - center);

    const auto projected = transformation.project3DPoint({static_cast<float>(inCamera[0]), static_cast<float>(inCamera[1]), static_cast<float>(inCamera[2])});
    return {projected.x, projected.y};
}

/// Center of the largest blob of `color` in the image, empty when the color is not present.
std::optional<cv::Point2d> findMarker(const cv::Mat& image, const cv::Vec3b& color) {
    const cv::Scalar lower(std::max(0, color[0] - COLOR_TOLERANCE), std::max(0, color[1] - COLOR_TOLERANCE), std::max(0, color[2] - COLOR_TOLERANCE));
    const cv::Scalar upper(std::min(255, color[0] + COLOR_TOLERANCE), std::min(255, color[1] + COLOR_TOLERANCE), std::min(255, color[2] + COLOR_TOLERANCE));

    cv::Mat mask;
    cv::inRange(image, lower, upper, mask);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(contours.empty()) return std::nullopt;

    const auto largest = std::max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
        return cv::contourArea(a) < cv::contourArea(b);
    });
    const auto moments = cv::moments(*largest);
    if(moments.m00 < 16.0) return std::nullopt;
    return cv::Point2d(moments.m10 / moments.m00, moments.m01 / moments.m00);
}

/// A node fed by the synthetic dataset, with the blending kept out of the way of the color checks.
std::shared_ptr<dai::node::Stitching> makePlanarNode(dai::Pipeline& pipeline, const Dataset& dataset) {
    auto stitching = pipeline.create<dai::node::Stitching>()->build(dataset.images.size());
    stitching->setMode(dai::node::Stitching::Mode::PLANAR_PROJECTION);
    stitching->setPlane(dataset.planePoint(), dataset.planeNormal());
    stitching->setMaxRange(4.0f, dai::LengthUnit::METER);
    stitching->setSyncThreshold(std::chrono::seconds(1));
    stitching->setExposureCompensator(dai::node::Stitching::ExposureCompensator::NONE);
    stitching->setSeamFinder(dai::node::Stitching::SeamFinder::VORONOI);
    stitching->setBlender(dai::node::Stitching::Blender::NONE);
    return stitching;
}

std::shared_ptr<dai::ImgFrame> runOnce(dai::Pipeline& pipeline, const std::shared_ptr<dai::node::Stitching>& stitching, const Dataset& dataset) {
    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    for(size_t i = 0; i < dataset.images.size(); ++i) {
        inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
    }
    auto output = stitching->out.createOutputQueue();

    pipeline.start();
    for(size_t i = 0; i < dataset.images.size(); ++i) {
        inputQueues[i]->send(toFrame(dataset.images[i], dataset.transformations[i], 3));
    }
    auto projected = output->get<dai::ImgFrame>();
    pipeline.stop();
    return projected;
}

const std::vector<SyntheticCamera> RIG = {makeCamera(0.0, -60.0, 0.0), makeCamera(-70.0, -40.0, -35.0), makeCamera(70.0, -40.0, 35.0)};

}  // namespace

TEST_CASE("Stitching validates the planar projection settings", "[Stitching]") {
    dai::Pipeline pipeline(false);
    auto stitching = pipeline.create<dai::node::Stitching>();

    REQUIRE_NOTHROW(stitching->setMode(dai::node::Stitching::Mode::PLANAR_PROJECTION));
    REQUIRE(stitching->getMode() == dai::node::Stitching::Mode::PLANAR_PROJECTION);
    REQUIRE_FALSE(stitching->getPlane().has_value());
    REQUIRE_FALSE(stitching->getView().has_value());

    REQUIRE_THROWS(stitching->setPlane({0.0f, 0.0f, 100.0f}, {0.0f, 0.0f, 0.0f}));
    REQUIRE_NOTHROW(stitching->setPlane({0.0f, 0.0f, 100.0f}, {0.0f, -1.0f, 0.0f}));
    REQUIRE(stitching->getPlane()->normal.y == -1.0f);

    REQUIRE_THROWS(stitching->setMaxRange(-1.0f));
    REQUIRE_THROWS(stitching->setMinIncidenceAngle(90.0f));
    REQUIRE_THROWS(stitching->setMaxViewSize(0, 100));

    stitching->setMaxRange(5.0f, dai::LengthUnit::METER);
    REQUIRE(stitching->getMaxRange(dai::LengthUnit::CENTIMETER) == 500.0f);

    auto view = dai::node::Stitching::VirtualCamera::lookAt({0, 0, 0}, {0, 0, 100}, {0, -1, 0}, 90.0f, 640, 480);
    REQUIRE(view.intrinsics[0][0] == 320.0f);
    REQUIRE_NOTHROW(stitching->setView(view));
    REQUIRE(stitching->getView()->width == 640);
    stitching->setViewAuto();
    REQUIRE_FALSE(stitching->getView().has_value());

    view.width = 0;
    REQUIRE_THROWS(stitching->setView(view));
}

TEST_CASE("Stitching projects a virtual rig onto the ground plane", "[Stitching]") {
    const auto dataset = renderDataset(RIG);

    dai::Pipeline pipeline(false);
    auto stitching = makePlanarNode(pipeline, dataset);
    auto projected = runOnce(pipeline, stitching, dataset);

    REQUIRE(projected != nullptr);
    REQUIRE(projected->getType() == dai::ImgFrame::Type::BGR888i);
    REQUIRE(projected->getSequenceNum() == 3);

    // The rendered image is described by the virtual camera it was rendered with, in the frame of the inputs
    const auto& transformation = projected->getTransformation();
    REQUIRE(transformation.getSize() == std::make_pair(static_cast<size_t>(projected->getWidth()), static_cast<size_t>(projected->getHeight())));
    REQUIRE(transformation.getReferenceFrame() == REFERENCE_FRAME);
    REQUIRE(transformation.getSourceIntrinsicMatrix()[0][0] > 0.0f);

    // The automatic view is a sensibly sized image bounded by the configured maximum
    REQUIRE(projected->getWidth() >= 128);
    REQUIRE(projected->getWidth() <= 1920);
    REQUIRE(projected->getHeight() >= 128);
    REQUIRE(projected->getHeight() <= 1920);

    // Every marker painted on the ground ends up where the geometry says it should
    const cv::Mat image = projected->getCvFrame();
    for(const auto& marker : MARKERS) {
        const auto expected = project(transformation, dataset.toReference(marker.position));
        const auto found = findMarker(image, marker.color);
        REQUIRE(found.has_value());
        REQUIRE(cv::norm(*found - expected) < MARKER_TOLERANCE);
    }
}

TEST_CASE("Stitching renders the plane from a given virtual camera", "[Stitching]") {
    const auto dataset = renderDataset(RIG);

    // A camera hanging three meters above the ground, looking straight down, with the world y axis pointing up in the
    // image - the classic bird's eye view of the rig
    const cv::Vec3d position = dataset.toReference({0.0, 100.0, 300.0});
    const cv::Vec3d target = dataset.toReference({0.0, 100.0, 0.0});
    const cv::Vec3d up = dataset.toReference({0.0, 1.0, 0.0}) - dataset.toReference({0.0, 0.0, 0.0});
    const auto view =
        dai::node::Stitching::VirtualCamera::lookAt({static_cast<float>(position[0]), static_cast<float>(position[1]), static_cast<float>(position[2])},
                                                    {static_cast<float>(target[0]), static_cast<float>(target[1]), static_cast<float>(target[2])},
                                                    {static_cast<float>(up[0]), static_cast<float>(up[1]), static_cast<float>(up[2])},
                                                    90.0f,
                                                    800,
                                                    600);

    dai::Pipeline pipeline(false);
    auto stitching = makePlanarNode(pipeline, dataset);
    stitching->setView(view);
    auto projected = runOnce(pipeline, stitching, dataset);

    REQUIRE(projected != nullptr);
    REQUIRE(projected->getWidth() == 800);
    REQUIRE(projected->getHeight() == 600);

    const cv::Mat image = projected->getCvFrame();
    const auto& transformation = projected->getTransformation();
    for(const auto& marker : MARKERS) {
        const auto expected = project(transformation, dataset.toReference(marker.position));
        const auto found = findMarker(image, marker.color);
        REQUIRE(found.has_value());
        REQUIRE(cv::norm(*found - expected) < MARKER_TOLERANCE);
    }
}

TEST_CASE("Stitching keeps the planar geometry across frames and rebuilds it on request", "[Stitching]") {
    const auto dataset = renderDataset(RIG);

    dai::Pipeline pipeline(false);
    auto stitching = makePlanarNode(pipeline, dataset);
    // The default compositing stack has to survive the projection as well
    stitching->setExposureCompensator(dai::node::Stitching::ExposureCompensator::GAIN_BLOCKS);
    stitching->setSeamFinder(dai::node::Stitching::SeamFinder::GRAPHCUT_COLOR);
    stitching->setBlender(dai::node::Stitching::Blender::MULTI_BAND);

    std::vector<std::shared_ptr<dai::InputQueue>> inputQueues;
    for(size_t i = 0; i < dataset.images.size(); ++i) {
        inputQueues.push_back(stitching->inputs["input" + std::to_string(i)].createInputQueue());
    }
    auto output = stitching->out.createOutputQueue();

    pipeline.start();

    unsigned int width = 0;
    unsigned int height = 0;
    for(int64_t group = 0; group < 3; ++group) {
        if(group == 2) {
            stitching->resetTransform();
        }
        for(size_t i = 0; i < dataset.images.size(); ++i) {
            inputQueues[i]->send(toFrame(dataset.images[i], dataset.transformations[i], group));
        }

        auto projected = output->get<dai::ImgFrame>();
        REQUIRE(projected != nullptr);
        REQUIRE(projected->getSequenceNum() == group);
        if(group == 0) {
            width = projected->getWidth();
            height = projected->getHeight();
        } else {
            // The geometry is fixed, so the view does not move from frame to frame, and rebuilding it from the same
            // calibration gives the same view again
            REQUIRE(projected->getWidth() == width);
            REQUIRE(projected->getHeight() == height);
        }
    }

    pipeline.stop();
}
