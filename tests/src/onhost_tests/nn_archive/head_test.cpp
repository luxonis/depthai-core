// This public header must provide ADL serialization without private includes.
#include <catch2/catch_test_macros.hpp>
#include <depthai/nn_archive/v1/Head.hpp>
#include <depthai/utility/Serialization.hpp>

namespace {
struct ParserOptions {
    dai::nn_archive::v1::Head head;
};
DEPTHAI_SERIALIZE_EXT(ParserOptions, head);
}  // namespace

TEST_CASE("NNArchive heads serialize through a public header") {
    const auto source = nlohmann::json::parse(R"({
        "name": "detections", "parser": "YOLO", "outputs": ["boxes", "masks"],
        "metadata": {
            "anchors": [[[10, 13], [16, 30]]], "classes": ["first", "second"],
            "conf_threshold": 0.25, "iou_threshold": 0.5, "max_det": 100, "n_classes": 2,
            "postprocessor_path": "custom.py", "is_softmax": false, "background_class": true,
            "boxes_outputs": "boxes", "scores_outputs": "scores", "angles_outputs": ["angles"],
            "keypoints_outputs": ["keypoints"], "mask_outputs": ["masks"], "n_keypoints": 17,
            "n_prototypes": 32, "protos_outputs": "protos", "subtype": "yolov8",
            "yolo_outputs": ["output"], "strides": [8, 16, 32],
            "custom_options": {"labels": [1, 3], "enabled": true}
        }
    })");
    const auto head = source.get<dai::nn_archive::v1::Head>();
    REQUIRE(nlohmann::json(head) == source);
    REQUIRE(head.metadata.nClasses == 2);
    REQUIRE(head.metadata.strides == std::vector<int64_t>{8, 16, 32});
    REQUIRE(head.metadata.extraParams.at("custom_options").at("enabled") == true);

    const ParserOptions options{head};
    std::vector<std::uint8_t> bytes;
    REQUIRE(dai::utility::serialize(options, bytes));
    REQUIRE(bytes.size() == nop::Encoding<ParserOptions>::Size(options));
    ParserOptions restored;
    REQUIRE(dai::utility::deserialize(bytes, restored));
    REQUIRE(nlohmann::json(restored.head) == source);

    for(const auto type : {dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
        CAPTURE(type);
        bytes.clear();
        REQUIRE(dai::utility::serialize(options, bytes, type));
        REQUIRE(dai::utility::deserialize(bytes, restored, type));
        REQUIRE(nlohmann::json(restored.head) == source);
    }

    // Parsed fields override their original values retained in extraParams.
    restored.head.metadata.nClasses = 3;
    REQUIRE(nlohmann::json(restored.head).at("metadata").at("n_classes") == 3);
    REQUIRE(nlohmann::json(restored.head).at("metadata").at("custom_options") == source.at("metadata").at("custom_options"));
}

TEST_CASE("NNArchive head optional fields can be missing or null") {
    auto head = nlohmann::json::parse(R"({"parser":"custom", "metadata":{}})").get<dai::nn_archive::v1::Head>();
    REQUIRE_FALSE(head.name.has_value());
    REQUIRE_FALSE(head.outputs.has_value());
    REQUIRE_FALSE(head.metadata.nClasses.has_value());
    REQUIRE_FALSE(head.metadata.classes.has_value());
    const nlohmann::json encoded = head;
    REQUIRE(encoded.at("name").is_null());
    REQUIRE(encoded.at("metadata").at("n_classes").is_null());

    const auto bytes = dai::utility::serialize(head);
    dai::nn_archive::v1::Head restored;
    REQUIRE(dai::utility::deserialize(bytes, restored));
    REQUIRE(nlohmann::json(restored) == encoded);

    head = encoded.get<dai::nn_archive::v1::Head>();
    REQUIRE_FALSE(head.name.has_value());
    REQUIRE_FALSE(head.metadata.nClasses.has_value());
    REQUIRE(nlohmann::json(head) == encoded);

    REQUIRE_THROWS(nlohmann::json::parse(R"({"metadata":{}})").get<dai::nn_archive::v1::Head>());
    REQUIRE_THROWS(nlohmann::json::parse(R"({"parser":"custom"})").get<dai::nn_archive::v1::Head>());
    REQUIRE_THROWS(nlohmann::json::parse(R"({"parser":"custom","metadata":{"n_classes":"two"}})").get<dai::nn_archive::v1::Head>());
}

TEST_CASE("NNArchive metadata preserves arbitrary JSON through libnop") {
    for(const auto& extraParams : {nlohmann::json(),
                                   nlohmann::json::object(),
                                   nlohmann::json::parse(R"({"custom_options":{"enabled":true,"labels":[1,3],"threshold":0.25}})"),
                                   nlohmann::json::array({1, false, nullptr, "custom"}),
                                   nlohmann::json::binary({0, 128, 255})}) {
        CAPTURE(extraParams);
        dai::nn_archive::v1::Metadata metadata;
        metadata.nClasses = 2;
        metadata.extraParams = extraParams;
        const auto bytes = dai::utility::serialize(metadata);
        dai::nn_archive::v1::Metadata restored;
        REQUIRE(dai::utility::deserialize(bytes, restored));
        REQUIRE(restored.nClasses == 2);
        REQUIRE_FALSE(restored.classes.has_value());
        REQUIRE(restored.extraParams == extraParams);
    }
}

TEST_CASE("NNArchive libnop serialization rejects truncated data") {
    const ParserOptions options{};
    auto bytes = dai::utility::serialize(options);
    bytes.pop_back();
    ParserOptions restored;
    nop::Deserializer<nop::BufferReader> deserializer{bytes.data(), bytes.size()};
    REQUIRE_FALSE(deserializer.Read(&restored));
}

TEST_CASE("Libnop JSON serialization rejects invalid MessagePack") {
    // 0xc1 is reserved and cannot start a MessagePack value.
    const auto bytes = dai::utility::serialize(std::vector<std::uint8_t>{0xc1});
    nlohmann::json restored = "unchanged";
    nop::Deserializer<nop::BufferReader> deserializer{bytes.data(), bytes.size()};
    REQUIRE(deserializer.Read(&restored).error() == nop::ErrorStatus::ProtocolError);
    REQUIRE(restored == "unchanged");
}
