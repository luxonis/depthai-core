#include <archive.h>
#include <archive_entry.h>

#include <array>
#include <catch2/catch_all.hpp>
#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <utility>

#include "depthai/depthai.hpp"

using namespace Catch::Matchers;

// Covers what the NeuralNetwork, DetectionNetwork and SpatialDetectionNetwork camera builds write into the ImgFrameCapability
// that they request from the camera, for each combination of NNArchive preprocessing.resize_mode and build arguments.
// The builds only create the pipeline. The pipeline is never started, so nothing runs on the device and the quasi archives below hold an empty model.

namespace {

constexpr uint32_t INPUT_WIDTH = 512;
constexpr uint32_t INPUT_HEIGHT = 288;

enum class NodeType { NEURAL_NETWORK, DETECTION_NETWORK, SPATIAL_DETECTION_NETWORK };
constexpr std::array<NodeType, 3> ALL_NODES = {NodeType::NEURAL_NETWORK, NodeType::DETECTION_NETWORK, NodeType::SPATIAL_DETECTION_NETWORK};

const char* nodeName(NodeType node) {
    switch(node) {
        case NodeType::NEURAL_NETWORK:
            return "NeuralNetwork";
        case NodeType::DETECTION_NETWORK:
            return "DetectionNetwork";
        case NodeType::SPATIAL_DETECTION_NETWORK:
            return "SpatialDetectionNetwork";
    }
    return "unknown";
}

constexpr const char* QUASI_ARCHIVE_PATH = "neural_network_nnarchive_behavior_test.tar";

void addArchiveEntry(struct archive* archive, const std::string& name, const std::string& data) {
    std::unique_ptr<struct archive_entry, decltype(&archive_entry_free)> entry(archive_entry_new(), &archive_entry_free);
    archive_entry_set_pathname(entry.get(), name.c_str());
    archive_entry_set_size(entry.get(), static_cast<la_int64_t>(data.size()));
    archive_entry_set_filetype(entry.get(), AE_IFREG);
    archive_entry_set_perm(entry.get(), 0644);
    REQUIRE(archive_write_header(archive, entry.get()) == ARCHIVE_OK);
    if(!data.empty()) {
        REQUIRE(archive_write_data(archive, data.data(), data.size()) == static_cast<la_ssize_t>(data.size()));
    }
}

nlohmann::json quasiConfig(const std::string& modelFile, const std::optional<std::string>& resizeMode) {
    nlohmann::json preprocessing = {
        {"mean", {0.0, 0.0, 0.0}}, {"scale", {1.0, 1.0, 1.0}}, {"reverse_channels", false}, {"interleaved_to_planar", true}, {"dai_type", nullptr}};
    if(resizeMode.has_value()) preprocessing["resize_mode"] = *resizeMode;

    nlohmann::json input = {{"name", "images"},
                            {"dtype", "uint8"},
                            {"input_type", "image"},
                            {"shape", {1, INPUT_HEIGHT, INPUT_WIDTH, 3}},
                            {"layout", "NHWC"},
                            {"preprocessing", preprocessing}};
    nlohmann::json output = {{"name", "output"}, {"dtype", "uint8"}, {"shape", {1, 85, 3024}}};
    nlohmann::json headMetadata = {{"classes", nlohmann::json::array({"person"})},
                                   {"n_classes", 1},
                                   {"iou_threshold", 0.5},
                                   {"conf_threshold", 0.5},
                                   {"max_det", 300},
                                   {"yolo_outputs", nlohmann::json::array({"output"})},
                                   {"subtype", "yolov6r2"}};
    nlohmann::json head = {{"parser", "YOLO"}, {"metadata", headMetadata}, {"outputs", nlohmann::json::array({"output"})}};
    nlohmann::json model = {{"metadata", {{"name", "quasi"}, {"path", modelFile}, {"precision", "int8"}}},
                            {"inputs", nlohmann::json::array({input})},
                            {"outputs", nlohmann::json::array({output})},
                            {"heads", nlohmann::json::array({head})}};
    return {{"config_version", "1.0"}, {"model", model}};
}

void writeQuasiArchive(const std::filesystem::path& path, const std::string& modelFile, const std::optional<std::string>& resizeMode) {
    std::unique_ptr<struct archive, decltype(&archive_write_free)> archive(archive_write_new(), &archive_write_free);
    REQUIRE(archive_write_set_format_ustar(archive.get()) == ARCHIVE_OK);
    REQUIRE(archive_write_open_filename(archive.get(), path.string().c_str()) == ARCHIVE_OK);
    addArchiveEntry(archive.get(), "config.json", quasiConfig(modelFile, resizeMode).dump());
    addArchiveEntry(archive.get(), modelFile, "");
    REQUIRE(archive_write_close(archive.get()) == ARCHIVE_OK);
}

dai::NNArchive loadQuasiArchive(const std::string& modelFile, const std::optional<std::string>& resizeMode) {
    const std::filesystem::path path = QUASI_ARCHIVE_PATH;
    struct RemoveFile {
        const std::filesystem::path& path;
        ~RemoveFile() {
            std::error_code ec;
            std::filesystem::remove(path, ec);
        }
    } removeFile{path};
    writeQuasiArchive(path, modelFile, resizeMode);
    return dai::NNArchive(path);
}

dai::NNArchive rvc4Archive(const std::optional<std::string>& resizeMode) {
    return loadQuasiArchive("quasi.dlc", resizeMode);
}

dai::ImgFrameCapability buildWithFps(dai::Pipeline& pipeline,
                                     const std::shared_ptr<dai::node::Camera>& camera,
                                     NodeType node,
                                     const dai::NNArchive& archive,
                                     std::optional<float> fps,
                                     std::optional<dai::ImgResizeMode> resizeMode) {
    const auto requestCount = camera->properties.outputRequests.size();
    switch(node) {
        case NodeType::NEURAL_NETWORK:
            pipeline.create<dai::node::NeuralNetwork>()->build(camera, archive, fps, resizeMode);
            break;
        case NodeType::DETECTION_NETWORK:
            pipeline.create<dai::node::DetectionNetwork>()->build(camera, archive, fps, resizeMode);
            break;
        case NodeType::SPATIAL_DETECTION_NETWORK:
            pipeline.create<dai::node::SpatialDetectionNetwork>()->build(camera, pipeline.create<dai::node::StereoDepth>(), archive, fps, resizeMode);
            break;
    }
    REQUIRE(camera->properties.outputRequests.size() == requestCount + 1);
    return camera->properties.outputRequests.back();
}

dai::ImgFrameCapability buildWithCapability(dai::Pipeline& pipeline,
                                            const std::shared_ptr<dai::node::Camera>& camera,
                                            NodeType node,
                                            const dai::NNArchive& archive,
                                            const dai::ImgFrameCapability& capability) {
    const auto requestCount = camera->properties.outputRequests.size();
    switch(node) {
        case NodeType::NEURAL_NETWORK:
            pipeline.create<dai::node::NeuralNetwork>()->build(camera, archive, capability);
            break;
        case NodeType::DETECTION_NETWORK:
            pipeline.create<dai::node::DetectionNetwork>()->build(camera, archive, capability);
            break;
        case NodeType::SPATIAL_DETECTION_NETWORK:
            pipeline.create<dai::node::SpatialDetectionNetwork>()->build(camera, pipeline.create<dai::node::StereoDepth>(), archive, capability);
            break;
    }
    REQUIRE(camera->properties.outputRequests.size() == requestCount + 1);
    return camera->properties.outputRequests.back();
}

void skipIfNotRvc4(dai::Pipeline& pipeline) {
    auto device = pipeline.getDefaultDevice();
    REQUIRE(device != nullptr);
    if(device->getPlatform() != dai::Platform::RVC4) {
        SKIP("The quasi archives hold a DLC model, which only RVC4 supports");
    }
}

// Decision matrix: fields that every camera build sets in the same way (getFrameCapability)
// | Field | Value                                                                                  |
// |-------|----------------------------------------------------------------------------------------|
// | size  | The archive input width x height. If the archive does not give them, the build stops.  |
// | type  | The archive dai_type if it is set. Else BGR888p on RVC2/RVC3, BGR888i on RVC4.         |
void requireArchiveSizeAndType(const dai::ImgFrameCapability& request) {
    REQUIRE(request.size.value.has_value());
    REQUIRE(std::get<std::pair<uint32_t, uint32_t>>(*request.size.value) == std::make_pair(INPUT_WIDTH, INPUT_HEIGHT));
    REQUIRE(request.type == dai::ImgFrame::Type::BGR888i);
}

}  // namespace

// | Archive resize_mode | resizeMode argument | Result in ImgFrameCapability |
// |---------------------|---------------------|------------------------------|
// | not set             | not set             | CROP                         |
// | not set             | X                   | X                            |
// | A                   | not set             | A                            |
// | A                   | X                   | X                            |
TEST_CASE("Camera fps build: resize mode is the argument, else the archive resize_mode, else CROP") {
    const auto archiveUnset = rvc4Archive(std::nullopt);
    const auto archiveLetterbox = rvc4Archive("LETTERBOX");

    dai::Pipeline pipeline;
    skipIfNotRvc4(pipeline);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    struct Row {
        const char* label;
        const dai::NNArchive& archive;
        std::optional<dai::ImgResizeMode> argument;
        dai::ImgResizeMode expected;
    };
    const std::array<Row, 5> rows = {{
        {"archive not set, argument not set", archiveUnset, std::nullopt, dai::ImgResizeMode::CROP},
        {"archive not set, argument STRETCH", archiveUnset, dai::ImgResizeMode::STRETCH, dai::ImgResizeMode::STRETCH},
        {"archive LETTERBOX, argument not set", archiveLetterbox, std::nullopt, dai::ImgResizeMode::LETTERBOX},
        {"archive LETTERBOX, argument STRETCH", archiveLetterbox, dai::ImgResizeMode::STRETCH, dai::ImgResizeMode::STRETCH},
        {"archive LETTERBOX, argument CROP", archiveLetterbox, dai::ImgResizeMode::CROP, dai::ImgResizeMode::CROP},
    }};

    for(auto node : ALL_NODES) {
        for(const auto& row : rows) {
            INFO(nodeName(node) << ": " << row.label);
            const auto request = buildWithFps(pipeline, camera, node, row.archive, std::nullopt, row.argument);
            REQUIRE(request.resizeMode == row.expected);
            requireArchiveSizeAndType(request);
        }
    }
}

// | Archive resize_mode | capability.resizeMode        | Result in ImgFrameCapability |
// |---------------------|------------------------------|------------------------------|
// | any                 | not changed (default CROP)   | CROP                         |
// | any                 | X                            | X                            |
TEST_CASE("Camera capability build: resize mode is the capability value and the archive resize_mode is not used") {
    const auto archiveUnset = rvc4Archive(std::nullopt);
    const auto archiveLetterbox = rvc4Archive("LETTERBOX");

    dai::Pipeline pipeline;
    skipIfNotRvc4(pipeline);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    dai::ImgFrameCapability defaultCapability;
    dai::ImgFrameCapability stretchCapability;
    stretchCapability.resizeMode = dai::ImgResizeMode::STRETCH;

    struct Row {
        const char* label;
        const dai::NNArchive& archive;
        const dai::ImgFrameCapability& capability;
        dai::ImgResizeMode expected;
    };
    const std::array<Row, 4> rows = {{
        {"archive not set, capability not changed", archiveUnset, defaultCapability, dai::ImgResizeMode::CROP},
        {"archive not set, capability STRETCH", archiveUnset, stretchCapability, dai::ImgResizeMode::STRETCH},
        {"archive LETTERBOX, capability not changed", archiveLetterbox, defaultCapability, dai::ImgResizeMode::CROP},
        {"archive LETTERBOX, capability STRETCH", archiveLetterbox, stretchCapability, dai::ImgResizeMode::STRETCH},
    }};

    for(auto node : ALL_NODES) {
        for(const auto& row : rows) {
            INFO(nodeName(node) << ": " << row.label);
            const auto request = buildWithCapability(pipeline, camera, node, row.archive, row.capability);
            REQUIRE(request.resizeMode == row.expected);
            requireArchiveSizeAndType(request);
        }
    }
}

// | Build   | fps                        | enableUndistortion | alphaScaling, ispOutput    |
// |---------|----------------------------|--------------------|----------------------------|
// | NN fps  | fps argument, else not set | not set            | default (nullopt, false)   |
// | DN fps  | same as NN fps             | not set            | default                    |
// | SDN fps | fps argument, else not set | true               | default                    |
TEST_CASE("Camera fps build: fps comes from the argument, undistortion is set only by SpatialDetectionNetwork") {
    const auto archive = rvc4Archive(std::nullopt);

    dai::Pipeline pipeline;
    skipIfNotRvc4(pipeline);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    for(auto node : ALL_NODES) {
        INFO(nodeName(node));
        const std::optional<bool> expectedUndistortion = node == NodeType::SPATIAL_DETECTION_NETWORK ? std::optional<bool>(true) : std::nullopt;

        const auto withoutFps = buildWithFps(pipeline, camera, node, archive, std::nullopt, std::nullopt);
        REQUIRE_FALSE(withoutFps.fps.value.has_value());
        REQUIRE(withoutFps.enableUndistortion == expectedUndistortion);
        REQUIRE_FALSE(withoutFps.alphaScaling.has_value());
        REQUIRE_FALSE(withoutFps.ispOutput);

        const auto withFps = buildWithFps(pipeline, camera, node, archive, 15.0f, std::nullopt);
        REQUIRE(withFps.fps.value.has_value());
        REQUIRE(std::get<float>(*withFps.fps.value) == 15.0f);
        REQUIRE(withFps.enableUndistortion == expectedUndistortion);
        REQUIRE_FALSE(withFps.alphaScaling.has_value());
        REQUIRE_FALSE(withFps.ispOutput);
    }
}

// | Build          | fps                 | enableUndistortion                     | alphaScaling, ispOutput |
// |----------------|---------------------|----------------------------------------|-------------------------|
// | NN capability  | from the capability | from the capability                    | from the capability     |
// | DN capability  | from the capability | from the capability                    | from the capability     |
// | SDN capability | from the capability | the capability value; true if not set  | from the capability     |
TEST_CASE("Camera capability build: fields come from the capability, SpatialDetectionNetwork sets undistortion when it is not set") {
    const auto archive = rvc4Archive(std::nullopt);

    dai::Pipeline pipeline;
    skipIfNotRvc4(pipeline);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    for(auto node : ALL_NODES) {
        for(const std::optional<bool> undistortion : {std::optional<bool>(), std::optional<bool>(false), std::optional<bool>(true)}) {
            INFO(nodeName(node) << ": capability enableUndistortion " << (undistortion.has_value() ? (*undistortion ? "true" : "false") : "not set"));
            dai::ImgFrameCapability capability;
            capability.fps.fixed(10.0f);
            capability.alphaScaling = 0.5f;
            capability.ispOutput = true;
            capability.enableUndistortion = undistortion;
            // The build replaces size and type with the archive values
            capability.size.fixed(std::make_pair(640u, 480u));
            capability.type = dai::ImgFrame::Type::NV12;

            const auto request = buildWithCapability(pipeline, camera, node, archive, capability);
            REQUIRE(request.fps.value.has_value());
            REQUIRE(std::get<float>(*request.fps.value) == 10.0f);
            REQUIRE(request.alphaScaling == std::optional<float>(0.5f));
            REQUIRE(request.ispOutput);
            if(node == NodeType::SPATIAL_DETECTION_NETWORK && !undistortion.has_value()) {
                REQUIRE(request.enableUndistortion == std::optional<bool>(true));
            } else {
                REQUIRE(request.enableUndistortion == undistortion);
            }
            requireArchiveSizeAndType(request);
        }
    }
}

// | Condition                                             | Result                                                       |
// |-------------------------------------------------------|--------------------------------------------------------------|
// | Archive resize_mode is not CROP, STRETCH or LETTERBOX | The archive does not load: Invalid preprocessing resize_mode |
TEST_CASE("NNArchive does not load an archive with an unsupported resize_mode") {
    REQUIRE_THROWS_WITH(loadQuasiArchive("quasi.dlc", "FOO"), ContainsSubstring("Invalid preprocessing resize_mode: FOO"));
}

// Decision matrix: errors
// | Condition                                                        | Result                                                                        |
// |------------------------------------------------------------------|-------------------------------------------------------------------------------|
// | A program changes the loaded config to a value that is not valid | getInputResizeMode throws Unsupported preprocessing resize_mode in NNArchive  |
TEST_CASE("NNArchive getInputResizeMode rejects an unsupported resize_mode in the loaded config") {
    auto archive = loadQuasiArchive("quasi.dlc", "CROP");
    REQUIRE(archive.getInputResizeMode() == dai::ImgResizeMode::CROP);

    // The config parser rejects unsupported values at load, so change the loaded config to reach this check
    auto& config = const_cast<dai::nn_archive::v1::Config&>(archive.getConfig<dai::nn_archive::v1::Config>());
    config.model.inputs[0].preprocessing.resizeMode = "FOO";
    REQUIRE_THROWS_WITH(archive.getInputResizeMode(), ContainsSubstring("Unsupported preprocessing resize_mode in NNArchive: FOO"));
}

// Decision matrix: errors
// | Condition                                       | Result                                             |
// |-------------------------------------------------|----------------------------------------------------|
// | The archive platform is not the device platform | Platform not supported by the neural network model |
TEST_CASE("Camera builds reject an archive that does not support the device platform") {
    // OTHER model types (for example ONNX) are not supported on any device platform
    const auto archive = loadQuasiArchive("quasi.onnx", std::nullopt);

    dai::Pipeline pipeline;
    REQUIRE(pipeline.getDefaultDevice() != nullptr);
    auto camera = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_A);

    for(auto node : ALL_NODES) {
        INFO(nodeName(node));
        REQUIRE_THROWS_WITH(buildWithFps(pipeline, camera, node, archive, std::nullopt, std::nullopt),
                            ContainsSubstring("Platform not supported by the neural network model"));
        REQUIRE_THROWS_WITH(buildWithCapability(pipeline, camera, node, archive, dai::ImgFrameCapability()),
                            ContainsSubstring("Platform not supported by the neural network model"));
    }
}
