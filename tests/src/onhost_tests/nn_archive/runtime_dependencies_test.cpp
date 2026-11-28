#include <archive.h>
#include <archive_entry.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <chrono>
#include <filesystem>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "../../../../src/nn_archive/v1/Generators.hpp"
#include "depthai/nn_archive/NNArchive.hpp"
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai/pipeline/node/SpatialDetectionNetwork.hpp"

namespace {
using Catch::Matchers::ContainsSubstring;
using nlohmann::json;

constexpr const char* REGISTRATION = "runtime/libTestReg.so";
constexpr const char* DSP_IMPLEMENTATION = "runtime/libTestImplDsp.so";
constexpr const char* CPU_IMPLEMENTATION = "runtime/libTestImplCpu.so";

json dependencies() {
    return {{"snpe_udo",
             {{{"registration", REGISTRATION},
               {"implementations", {{{"runtime", "dsp"}, {"path", DSP_IMPLEMENTATION}}, {{"runtime", "cpu"}, {"path", CPU_IMPLEMENTATION}}}}}}}};
}

json config(const json& runtimeDependencies = dependencies()) {
    return {{"config_version", "1.1"},
            {"model",
             {{"metadata", {{"name", "udo-test"}, {"path", "model.dlc"}}},
              {"inputs", json::array()},
              {"outputs", json::array()},
              {"runtime_dependencies", runtimeDependencies}}}};
}

struct ArchiveFixture {
    std::filesystem::path path =
        std::filesystem::temp_directory_path() / ("depthai-udo-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".tar");
    std::unique_ptr<struct archive, decltype(&archive_write_free)> writer{archive_write_new(), &archive_write_free};

    explicit ArchiveFixture(const json& metadata, bool compressed = false) {
        REQUIRE(writer);
        if(compressed) {
            path += ".xz";
            REQUIRE(archive_write_add_filter_xz(writer.get()) == ARCHIVE_OK);
        }
        REQUIRE(archive_write_set_format_ustar(writer.get()) == ARCHIVE_OK);
        REQUIRE(archive_write_open_filename(writer.get(), path.string().c_str()) == ARCHIVE_OK);
        add("config.json", metadata.dump());
        add("model.dlc", "dlc");
    }
    void add(const std::string& name, const std::string& data, bool symlink = false) {
        std::unique_ptr<struct archive_entry, decltype(&archive_entry_free)> entry{archive_entry_new(), &archive_entry_free};
        REQUIRE(entry);
        archive_entry_set_pathname(entry.get(), name.c_str());
        archive_entry_set_filetype(entry.get(), symlink ? AE_IFLNK : AE_IFREG);
        archive_entry_set_perm(entry.get(), 0644);
        archive_entry_set_size(entry.get(), symlink ? 0 : static_cast<la_int64_t>(data.size()));
        if(symlink) archive_entry_set_symlink(entry.get(), data.c_str());
        REQUIRE(archive_write_header(writer.get(), entry.get()) == ARCHIVE_OK);
        if(!symlink) REQUIRE(archive_write_data(writer.get(), data.data(), data.size()) == static_cast<la_ssize_t>(data.size()));
    }
    void addLibraries() {
        add(REGISTRATION, "registration");
        add(DSP_IMPLEMENTATION, "dsp implementation");
        add(CPU_IMPLEMENTATION, "cpu implementation");
    }
    void close() {
        REQUIRE(archive_write_close(writer.get()) == ARCHIVE_OK);
        writer.reset();
    }
    ~ArchiveFixture() {
        writer.reset();
        std::error_code error;
        std::filesystem::remove(path, error);
    }
};

std::set<std::string> assetKeys(dai::node::NeuralNetwork& network) {
    std::set<std::string> keys;
    for(const auto& asset : network.getAssetManager().getAll()) keys.insert(asset->key);
    return keys;
}

std::vector<uint8_t> bytes(const std::string& text) {
    return {text.begin(), text.end()};
}
}  // namespace

TEST_CASE("NNArchive sends the registration and DSP libraries as pipeline assets") {
    ArchiveFixture fixture(config(), GENERATE(false, true));
    fixture.addLibraries();
    fixture.add("undeclared.so", "ignored");
    fixture.close();
    const dai::NNArchive archive(fixture.path);
    const auto& libraries = archive.getRuntimeLibraries();
    REQUIRE(libraries.size() == 3);
    REQUIRE(libraries.at(REGISTRATION) == bytes("registration"));
    REQUIRE(libraries.at(CPU_IMPLEMENTATION) == bytes("cpu implementation"));
    REQUIRE(json(archive.getConfig<dai::nn_archive::v1::Config>().model.runtimeDependencies.value()) == dependencies());

    auto network = dai::node::NeuralNetwork::create(std::shared_ptr<dai::Device>{});
    network->setNNArchive(archive);
    const std::set<std::string> expectedAssets{"__model", std::string("__udo/") + REGISTRATION, std::string("__udo/") + DSP_IMPLEMENTATION};
    const auto checkConfigured = [&]() {
        const auto& properties = network->properties;
        REQUIRE(properties.udoPackages.size() == 1);
        REQUIRE(properties.udoPackages[0].registration == REGISTRATION);
        REQUIRE(properties.udoPackages[0].dspImplementations == std::vector<std::string>{DSP_IMPLEMENTATION});
        REQUIRE(properties.udoLibraryUris.size() == 2);
        REQUIRE(properties.udoLibraryUris.count(CPU_IMPLEMENTATION) == 0);
        REQUIRE(assetKeys(*network) == expectedAssets);
        for(const auto* path : {REGISTRATION, DSP_IMPLEMENTATION}) {
            REQUIRE(network->getAssetManager().get(std::string("__udo/") + path)->getData() == libraries.at(path));
        }
    };
    checkConfigured();
    REQUIRE(network->getNNArchive().has_value());

    SECTION("Properties keep UDO packages through serialization") {
        const auto& properties = network->properties;
        for(const auto type : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON}) {
            std::vector<uint8_t> data;
            REQUIRE(dai::utility::serialize(properties, data, type));
            dai::NeuralNetworkProperties restored;
            REQUIRE(dai::utility::deserialize(data, restored, type));
            REQUIRE(restored.udoPackages.size() == 1);
            REQUIRE(restored.udoPackages[0].registration == REGISTRATION);
            REQUIRE(restored.udoPackages[0].dspImplementations == properties.udoPackages[0].dspImplementations);
            REQUIRE(restored.udoLibraryUris == properties.udoLibraryUris);
        }
    }
    SECTION("Setting the archive again leaves one copy of each library") {
        network->setNNArchive(archive);
        checkConfigured();
        // The argument may be the stored archive itself.
        network->setNNArchive(network->getNNArchive()->get());
        checkConfigured();
    }
    SECTION("A missing model file keeps the current model") {
        REQUIRE_THROWS_WITH(network->setOtherModelFormat(std::filesystem::path("/nonexistent/model.dlc")), ContainsSubstring("doesn't exist"));
        checkConfigured();
        REQUIRE(network->getNNArchive().has_value());
    }
    SECTION("A directory as the model path keeps the current model") {
        REQUIRE_THROWS_WITH(network->setOtherModelFormat(std::filesystem::temp_directory_path()), ContainsSubstring("is not a readable file"));
        checkConfigured();
        REQUIRE(network->getNNArchive().has_value());
    }
    SECTION("A raw model replaces the archive libraries") {
        network->setOtherModelFormat(std::vector<uint8_t>{1, 2, 3});
        REQUIRE(network->properties.udoPackages.empty());
        REQUIRE(network->properties.udoLibraryUris.empty());
        REQUIRE(assetKeys(*network) == std::set<std::string>{"__model"});
    }
    SECTION("A new model removes the library assets even after the properties were edited") {
        network->properties.udoLibraryUris.clear();
        network->setOtherModelFormat(std::vector<uint8_t>{1, 2, 3});
        REQUIRE(assetKeys(*network) == std::set<std::string>{"__model"});
    }
    SECTION("A model file replaces the archive libraries") {
        network->setOtherModelFormat(fixture.path);
        REQUIRE(network->properties.udoPackages.empty());
        REQUIRE(network->properties.udoLibraryUris.empty());
        REQUIRE(assetKeys(*network) == std::set<std::string>{"__model"});
    }
    SECTION("A device model replaces the archive model and libraries") {
        network->setModelFromDeviceZoo(dai::DeviceModelZoo::NEURAL_DEPTH_384X240);
        REQUIRE(network->properties.udoPackages.empty());
        REQUIRE(network->properties.udoLibraryUris.empty());
        REQUIRE(network->properties.modelUri.empty());
        REQUIRE(assetKeys(*network).empty());
        network->setNNArchive(archive);
        REQUIRE_FALSE(network->properties.deviceModel);
        checkConfigured();
    }
}

TEST_CASE("Archive libraries with the same file name must be identical") {
    auto metadata = dependencies();
    metadata["snpe_udo"].push_back(
        {{"registration", "second/libTestReg.so"}, {"implementations", {{{"runtime", "dsp"}, {"path", "second/libSecondImplDsp.so"}}}}});
    ArchiveFixture fixture(config(metadata));
    fixture.addLibraries();
    fixture.add("second/libSecondImplDsp.so", "second implementation");
    const auto identical = GENERATE(true, false);
    fixture.add("second/libTestReg.so", identical ? "registration" : "other registration");
    fixture.close();
    const dai::NNArchive archive(fixture.path);
    auto network = dai::node::NeuralNetwork::create(std::shared_ptr<dai::Device>{});
    if(identical) {
        network->setNNArchive(archive);
        REQUIRE(network->properties.udoPackages.size() == 2);
        REQUIRE(network->properties.udoLibraryUris.size() == 4);
    } else {
        REQUIRE_THROWS_WITH(network->setNNArchive(archive), ContainsSubstring("Conflicting NNArchive UDO libraries"));
    }
}

TEST_CASE("A UDO package needs a DSP implementation") {
    auto metadata = dependencies();
    metadata["snpe_udo"][0]["implementations"] = {{{"runtime", "cpu"}, {"path", CPU_IMPLEMENTATION}}};
    ArchiveFixture fixture(config(metadata));
    fixture.add(REGISTRATION, "registration");
    fixture.add(CPU_IMPLEMENTATION, "cpu implementation");
    fixture.close();
    const dai::NNArchive archive(fixture.path);
    auto network = dai::node::NeuralNetwork::create(std::shared_ptr<dai::Device>{});
    network->setOtherModelFormat(std::vector<uint8_t>{1, 2, 3});
    REQUIRE_THROWS_WITH(network->setNNArchive(archive), ContainsSubstring("has no implementation for the SNPE DSP runtime"));
    // A rejected archive changes nothing.
    REQUIRE_FALSE(network->getNNArchive().has_value());
    REQUIRE(assetKeys(*network) == std::set<std::string>{"__model"});
    REQUIRE(network->getAssetManager().get("__model")->getData() == std::vector<uint8_t>{1, 2, 3});
}

TEST_CASE("Detection networks keep their parser settings when the network rejects an archive") {
    const auto detectionConfig = [](const std::vector<std::string>& classes, double threshold, const json& runtimeDependencies) {
        auto metadata = config(runtimeDependencies);
        metadata["model"]["heads"] = {
            {{"parser", "SSD"},
             {"metadata", {{"classes", classes}, {"n_classes", classes.size()}, {"conf_threshold", threshold}, {"iou_threshold", threshold}}}}};
        return metadata;
    };
    ArchiveFixture acceptedFixture(detectionConfig({"person"}, 0.25, dependencies()));
    acceptedFixture.addLibraries();
    acceptedFixture.close();
    const dai::NNArchive accepted(acceptedFixture.path);

    auto withoutDsp = dependencies();
    withoutDsp["snpe_udo"][0]["implementations"] = {{{"runtime", "cpu"}, {"path", CPU_IMPLEMENTATION}}};
    ArchiveFixture rejectedFixture(detectionConfig({"car", "bus"}, 0.75, withoutDsp));
    rejectedFixture.add(REGISTRATION, "registration");
    rejectedFixture.add(CPU_IMPLEMENTATION, "cpu implementation");
    rejectedFixture.close();
    const dai::NNArchive rejected(rejectedFixture.path);

    const auto check = [&](const auto& network) {
        network->setNNArchive(accepted);
        REQUIRE_THROWS_WITH(network->setNNArchive(rejected), ContainsSubstring("has no implementation for the SNPE DSP runtime"));
        // The parser must not keep the classes and thresholds of the rejected archive.
        REQUIRE(network->getClasses().value_or(std::vector<std::string>{}) == std::vector<std::string>{"person"});
        REQUIRE(network->getConfidenceThreshold() == 0.25f);
        REQUIRE(network->detectionParser->getIouThreshold() == 0.25f);
        REQUIRE(network->neuralNetwork->properties.udoPackages.size() == 1);
    };
    SECTION("DetectionNetwork") {
        check(dai::node::DetectionNetwork::create(std::shared_ptr<dai::Device>{}));
    }
    SECTION("SpatialDetectionNetwork") {
        check(dai::node::SpatialDetectionNetwork::create(std::shared_ptr<dai::Device>{}));
    }
}

TEST_CASE("The DSP runtime name is case-insensitive") {
    auto metadata = dependencies();
    metadata["snpe_udo"][0]["implementations"][0]["runtime"] = "DSP";
    ArchiveFixture fixture(config(metadata));
    fixture.addLibraries();
    fixture.close();
    const dai::NNArchive archive(fixture.path);
    auto network = dai::node::NeuralNetwork::create(std::shared_ptr<dai::Device>{});
    network->setNNArchive(archive);
    REQUIRE(network->properties.udoPackages.at(0).dspImplementations == std::vector<std::string>{DSP_IMPLEMENTATION});
}

TEST_CASE("Runtime dependencies preserve multiple packages and runtimes through JSON") {
    auto metadata = dependencies();
    metadata["snpe_udo"].push_back(
        {{"registration", "runtime/libSecondReg.so"}, {"implementations", {{{"runtime", "dsp"}, {"path", "runtime/libSecondImplDsp.so"}}}}});
    const auto parsed = metadata.get<dai::nn_archive::v1::RuntimeDependencies>();
    REQUIRE(parsed.snpeUdo.size() == 2);
    REQUIRE(parsed.snpeUdo.at(0).implementations.size() == 2);
    REQUIRE(json(parsed) == metadata);
}

TEST_CASE("NNArchive rejects missing, empty, duplicate and linked runtime libraries") {
    ArchiveFixture fixture(config());
    fixture.add(REGISTRATION, "registration");
    fixture.add(CPU_IMPLEMENTATION, "cpu implementation");
    std::string expectedError;
    SECTION("missing") {
        expectedError = "Missing NNArchive runtime library: runtime/libTestImplDsp.so";
    }
    SECTION("empty") {
        fixture.add(DSP_IMPLEMENTATION, "");
        expectedError = "Empty NNArchive runtime library: runtime/libTestImplDsp.so";
    }
    SECTION("duplicate") {
        fixture.add(DSP_IMPLEMENTATION, "first");
        fixture.add(DSP_IMPLEMENTATION, "second");
        expectedError = "Duplicate NNArchive runtime library: runtime/libTestImplDsp.so";
    }
    SECTION("symlink") {
        fixture.add(DSP_IMPLEMENTATION, "../other.so", true);
        expectedError = "NNArchive runtime library must be a regular file: runtime/libTestImplDsp.so";
    }
    fixture.close();
    REQUIRE_THROWS_WITH(dai::NNArchive(fixture.path), ContainsSubstring(expectedError));
}

TEST_CASE("Runtime dependency paths follow the luxonis-ml rules") {
    using Dependencies = dai::nn_archive::v1::RuntimeDependencies;
    const auto withPath = [](const std::string& path, bool registration) {
        auto metadata = dependencies();
        if(registration) {
            metadata["snpe_udo"][0]["registration"] = path;
        } else {
            metadata["snpe_udo"][0]["implementations"][0]["path"] = path;
        }
        return metadata;
    };
    for(const auto& path : std::vector<std::string>{"",
                                                    ".",
                                                    "../lib.so",
                                                    "a/../lib.so",
                                                    "/lib.so",
                                                    "//server/share/lib.so",
                                                    "C:/lib.so",
                                                    "C:lib.so",
                                                    "a\\lib.so",
                                                    "./lib.so",
                                                    "a/./lib.so",
                                                    "a//lib.so",
                                                    "a/",
                                                    "config.json",
                                                    std::string("runtime/lib\0.so", 15)}) {
        CAPTURE(path);
        for(const bool registration : {true, false}) {
            REQUIRE_THROWS_WITH(withPath(path, registration).get<Dependencies>(), ContainsSubstring("Invalid NNArchive runtime dependency path"));
        }
    }
    // luxonis-ml accepts these, so an archive it produced must load.
    for(const auto& path : std::vector<std::string>{"lib.so", "a/b/lib.so", "runtime/v1;2/lib:x.so", "ab:c/lib.so", "config.json/lib.so"}) {
        CAPTURE(path);
        for(const bool registration : {true, false}) {
            REQUIRE_NOTHROW(withPath(path, registration).get<Dependencies>());
        }
    }
}

TEST_CASE("Runtime dependency metadata rejects invalid shapes") {
    using Dependencies = dai::nn_archive::v1::RuntimeDependencies;
    for(const auto& metadata : {json::object(),
                                json{{"snpe_udo", json::array()}},
                                json{{"snpe_udo", "invalid"}},
                                json{{"snpe_udo", {json::object()}}},
                                json{{"snpe_udo", dependencies()["snpe_udo"]}, {"extra", 1}}}) {
        CAPTURE(metadata.dump());
        REQUIRE_THROWS(metadata.get<Dependencies>());
    }
    auto metadata = dependencies();
    metadata["snpe_udo"][0]["implementations"] = json::array();
    REQUIRE_THROWS_WITH(metadata.get<Dependencies>(), ContainsSubstring("Invalid NNArchive SNPE UDO package metadata"));
    metadata = dependencies();
    metadata["snpe_udo"][0]["implementations"][0]["runtime"] = "";
    REQUIRE_THROWS_WITH(metadata.get<Dependencies>(), ContainsSubstring("Invalid NNArchive runtime implementation metadata"));
    metadata = dependencies();
    metadata["snpe_udo"][0]["implementations"][0]["extra"] = 1;
    REQUIRE_THROWS_WITH(metadata.get<Dependencies>(), ContainsSubstring("Invalid NNArchive runtime implementation metadata"));
}

TEST_CASE("Ordinary archives need no runtime libraries") {
    auto metadata = config();
    metadata["config_version"] = "1.0";
    metadata["model"].erase("runtime_dependencies");
    ArchiveFixture fixture(metadata);
    fixture.close();
    const dai::NNArchive archive(fixture.path);
    REQUIRE_FALSE(archive.getConfig<dai::nn_archive::v1::Config>().model.runtimeDependencies);
    REQUIRE(archive.getRuntimeLibraries().empty());
    auto network = dai::node::NeuralNetwork::create(std::shared_ptr<dai::Device>{});
    network->setNNArchive(archive);
    REQUIRE(network->properties.udoPackages.empty());
    REQUIRE(assetKeys(*network) == std::set<std::string>{"__model"});
}

TEST_CASE("Null runtime dependencies preserve ordinary archive behavior") {
    auto metadata = config();
    metadata["model"]["runtime_dependencies"] = nullptr;
    ArchiveFixture fixture(metadata);
    fixture.close();
    const dai::NNArchive archive(fixture.path);
    REQUIRE_FALSE(archive.getConfig<dai::nn_archive::v1::Config>().model.runtimeDependencies);
    REQUIRE(archive.getRuntimeLibraries().empty());
}

TEST_CASE("Runtime dependencies require an RVC4 DLC archive") {
    auto metadata = config();
    metadata["model"]["metadata"]["path"] = "model.onnx";
    ArchiveFixture fixture(metadata);
    fixture.add("model.onnx", "onnx");
    fixture.close();
    REQUIRE_THROWS_WITH(dai::NNArchive(fixture.path), ContainsSubstring("require an RVC4 SNPE DLC model"));
}
