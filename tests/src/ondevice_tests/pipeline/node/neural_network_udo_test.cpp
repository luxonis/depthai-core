#include <archive.h>
#include <archive_entry.h>

#include <catch2/catch_all.hpp>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <magic_enum/magic_enum.hpp>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #include <opencv2/core.hpp>
#endif

namespace {
using Catch::Matchers::ContainsSubstring;

// An ELF header with the identity the device accepts, but no loadable program
// headers, so SNPE fails to register it. No compiled fixture is needed.
std::vector<uint8_t> unloadableElf(bool dsp) {
    std::vector<uint8_t> bytes(dsp ? 52 : 64);
    bytes[0] = 0x7f;
    bytes[1] = 'E';
    bytes[2] = 'L';
    bytes[3] = 'F';
    bytes[4] = dsp ? 1 : 2;       // ELFCLASS32 / ELFCLASS64
    bytes[5] = 1;                 // little endian
    bytes[6] = 1;                 // ELF version
    bytes[16] = 3;                // ET_DYN
    bytes[18] = dsp ? 164 : 183;  // EM_HEXAGON / EM_AARCH64
    return bytes;
}

// Writes an RVC4 NNArchive whose DLC and UDO libraries are placeholders. Each
// "dsp" implementation is a Hexagon library, or an AArch64 one if requested.
std::filesystem::path writeUdoArchive(const std::map<std::string, bool>& implementationIsAarch64) {
    const auto path = std::filesystem::temp_directory_path()
                      / ("depthai-udo-device-test-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".tar");
    auto implementations = nlohmann::json::array();
    for(const auto& implementation : implementationIsAarch64) implementations.push_back({{"runtime", "dsp"}, {"path", implementation.first}});
    const nlohmann::json config = {
        {"config_version", "1.1"},
        {"model",
         {{"metadata", {{"name", "udo-test"}, {"path", "model.dlc"}}},
          {"inputs", nlohmann::json::array()},
          {"outputs", nlohmann::json::array()},
          {"runtime_dependencies", {{"snpe_udo", {{{"registration", "runtime/libDaiTestUdoReg.so"}, {"implementations", implementations}}}}}}}}};
    const auto configText = config.dump();
    std::map<std::string, std::vector<uint8_t>> members{
        {"config.json", {configText.begin(), configText.end()}}, {"model.dlc", {1}}, {"runtime/libDaiTestUdoReg.so", unloadableElf(false)}};
    for(const auto& implementation : implementationIsAarch64) members.emplace(implementation.first, unloadableElf(!implementation.second));
    std::unique_ptr<struct archive, decltype(&archive_write_free)> writer{archive_write_new(), &archive_write_free};
    REQUIRE(writer);
    REQUIRE(archive_write_set_format_ustar(writer.get()) == ARCHIVE_OK);
    REQUIRE(archive_write_open_filename(writer.get(), path.string().c_str()) == ARCHIVE_OK);
    for(const auto& member : members) {
        std::unique_ptr<struct archive_entry, decltype(&archive_entry_free)> entry{archive_entry_new(), &archive_entry_free};
        REQUIRE(entry);
        archive_entry_set_pathname(entry.get(), member.first.c_str());
        archive_entry_set_filetype(entry.get(), AE_IFREG);
        archive_entry_set_perm(entry.get(), 0644);
        archive_entry_set_size(entry.get(), static_cast<la_int64_t>(member.second.size()));
        REQUIRE(archive_write_header(writer.get(), entry.get()) == ARCHIVE_OK);
        REQUIRE(archive_write_data(writer.get(), member.second.data(), member.second.size()) == static_cast<la_ssize_t>(member.second.size()));
    }
    REQUIRE(archive_write_close(writer.get()) == ARCHIVE_OK);
    return path;
}

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
// Layer name -> dequantized values, checked to be finite.
std::map<std::string, std::vector<float>> tensorValues(dai::NNData& data) {
    std::map<std::string, std::vector<float>> values;
    for(const auto& name : data.getAllLayerNames()) {
        const auto tensor = data.getTensor<float>(name, true);
        std::vector<float> layer(tensor.begin(), tensor.end());
        REQUIRE(std::all_of(layer.begin(), layer.end(), [](float value) { return std::isfinite(value); }));
        values.emplace(name, std::move(layer));
    }
    return values;
}

// Pearson correlation of two equally long series.
double correlation(const std::vector<float>& a, const std::vector<float>& b) {
    REQUIRE(a.size() == b.size());
    REQUIRE_FALSE(a.empty());
    const auto count = static_cast<double>(a.size());
    double meanA = 0.0;
    double meanB = 0.0;
    for(std::size_t i = 0; i < a.size(); ++i) {
        meanA += a[i] / count;
        meanB += b[i] / count;
    }
    double covariance = 0.0;
    double varianceA = 0.0;
    double varianceB = 0.0;
    for(std::size_t i = 0; i < a.size(); ++i) {
        covariance += (a[i] - meanA) * (b[i] - meanB);
        varianceA += (a[i] - meanA) * (a[i] - meanA);
        varianceB += (b[i] - meanB) * (b[i] - meanB);
    }
    // Two constant series are equal if their values are.
    if(varianceA == 0.0 || varianceB == 0.0) return a == b ? 1.0 : 0.0;
    return covariance / std::sqrt(varianceA * varianceB);
}
#endif
}  // namespace

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
// Hidden: it needs archives from the environment, and a skipped case would make
// ctest report the whole executable as skipped. Run it with the "[udo]" tag.
TEST_CASE("RVC4 NNArchive UDO inference matches reference in repeated sessions", "[.][rvc4][udo]") {
    const auto udoPath = std::getenv("DEPTHAI_TEST_UDO_ARCHIVE");
    const auto referencePath = std::getenv("DEPTHAI_TEST_UDO_REFERENCE_ARCHIVE");
    if(!udoPath || !referencePath) SKIP("Set DEPTHAI_TEST_UDO_ARCHIVE and DEPTHAI_TEST_UDO_REFERENCE_ARCHIVE to matching model archives");
    const dai::NNArchive archive{udoPath};
    const dai::NNArchive reference{referencePath};
    REQUIRE(archive.getConfig<dai::nn_archive::v1::Config>().model.runtimeDependencies.has_value());
    REQUIRE_FALSE(reference.getConfig<dai::nn_archive::v1::Config>().model.runtimeDependencies.has_value());
    REQUIRE(archive.getInputSize() == reference.getInputSize());
    REQUIRE(archive.getInputSize().has_value());
    const auto size = *archive.getInputSize();
    auto inputType = dai::ImgFrame::Type::BGR888i;
    const auto configuredType = archive.getConfig<dai::nn_archive::v1::Config>().model.inputs.at(0).preprocessing.daiType;
    if(configuredType) {
        const auto type = magic_enum::enum_cast<dai::ImgFrame::Type>(*configuredType);
        REQUIRE(type.has_value());
        inputType = *type;
    }

    // The reference may use another precision (an INT8 model against an FP32 UDO
    // model), so its outputs must only correlate. Outputs of the UDO model must be
    // identical across networks and sessions. A failed UDO yields unrelated outputs.
    constexpr double MIN_CORRELATION = 0.98;
    // Sample -> layer name -> output of the UDO model in the first session.
    std::vector<std::map<std::string, std::vector<float>>> firstSession;

    // Pipeline::stop() closes its device, so each pipeline runs in a new device
    // session. Within a session, two UDO networks share one registration.
    for(int session = 0; session < 4; ++session) {
        CAPTURE(session);
        auto device = std::make_shared<dai::Device>();
        if(device->getPlatform() != dai::Platform::RVC4) SKIP("RVC4-only test");
        dai::Pipeline pipeline(device);
        std::vector<std::shared_ptr<dai::InputQueue>> inputs;
        std::vector<std::shared_ptr<dai::MessageQueue>> outputs;
        for(const auto* model : {&reference, &archive, &archive}) {
            auto network = pipeline.create<dai::node::NeuralNetwork>();
            network->setNNArchive(*model);
            inputs.push_back(network->input.createInputQueue());
            outputs.push_back(network->out.createOutputQueue());
        }
        pipeline.start();
        cv::RNG random(42);
        for(std::size_t sample = 0; sample < 3; ++sample) {
            CAPTURE(sample);
            cv::Mat pixels(size.second, size.first, CV_8UC3);
            random.fill(pixels, cv::RNG::UNIFORM, 0, 256);
            auto frame = std::make_shared<dai::ImgFrame>();
            frame->setCvFrame(pixels, inputType);
            for(const auto& input : inputs) input->send(frame);
            std::vector<std::map<std::string, std::vector<float>>> results;  // reference, UDO, UDO
            for(const auto& output : outputs) {
                bool timedOut = false;
                auto result = output->get<dai::NNData>(std::chrono::seconds(30), timedOut);
                REQUIRE_FALSE(timedOut);
                REQUIRE(result);
                results.push_back(tensorValues(*result));
            }
            const auto& expected = results[0];
            const auto& udo = results[1];
            REQUIRE(results[2] == udo);
            if(session == 0) firstSession.push_back(udo);
            REQUIRE(firstSession.at(sample) == udo);
            REQUIRE(udo.size() == expected.size());
            for(const auto& layer : expected) {
                CAPTURE(layer.first);
                REQUIRE(udo.count(layer.first) == 1);
                REQUIRE(correlation(layer.second, udo.at(layer.first)) >= MIN_CORRELATION);
            }
        }
        pipeline.stop();
        pipeline.wait();
    }
}
#endif

TEST_CASE("RVC4 NNArchive UDO libraries reach SNPE registration", "[rvc4][udo]") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) SKIP("RVC4-only test");
    std::map<std::string, bool> implementations;
    std::string expectedError;
    SECTION("unique library names") {
        implementations = {{"runtime/dsp/libDaiTestUdoImplDsp.so", false}};
        expectedError = "Could not register UDO package libDaiTestUdoReg.so";
    }
    SECTION("AArch64 library that prepares the network") {
        // Staged on LD_LIBRARY_PATH; a missing directory would fail before registration.
        implementations = {{"runtime/libDaiTestUdoImplDsp.so", false}, {"runtime/arm/libDaiTestUdoImplDsp_AltPrep.so", true}};
        expectedError = "Could not register UDO package libDaiTestUdoReg.so";
    }
    SECTION("name of a DSP library on the device search path") {
        // The firmware package ships this library in a directory on ADSP_LIBRARY_PATH.
        implementations = {{"runtime/libStereoDepthDsp_skel.so", false}};
        expectedError = "conflicts with the device library";
    }
    const auto archivePath = writeUdoArchive(implementations);
    const dai::NNArchive archive(archivePath);
    std::filesystem::remove(archivePath);
    auto network = pipeline.create<dai::node::NeuralNetwork>();
    network->setNNArchive(archive);
    auto output = network->out.createOutputQueue();
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring(expectedError));
}

TEST_CASE("RVC4 NNArchive UDO errors identify invalid runtime assets", "[rvc4][udo]") {
    dai::Pipeline pipeline;
    if(pipeline.getDefaultDevice()->getPlatform() != dai::Platform::RVC4) SKIP("RVC4-only test");
    auto network = pipeline.create<dai::node::NeuralNetwork>();
    network->setOtherModelFormat(std::vector<uint8_t>{1});
    network->properties.udoPackages = {{"libReg.so", {"libImpl.so"}}};
    auto output = network->out.createOutputQueue();
    std::string expectedError;
    SECTION("unsupported CPU runtime") {
        network->setBackendProperties({{"runtime", "cpu"}});
        expectedError = "support only the SNPE DSP runtime";
    }
    SECTION("unsupported GPU runtime") {
        network->setBackendProperties({{"runtime", "gpu"}});
        expectedError = "support only the SNPE DSP runtime";
    }
    SECTION("package without a DSP implementation") {
        network->properties.udoPackages = {{"libReg.so", {}}};
        expectedError = "has no DSP implementation";
    }
    SECTION("path without a file name") {
        network->properties.udoPackages = {{"runtime/", {"libImpl.so"}}};
        expectedError = "Invalid NNArchive UDO library path";
    }
    SECTION("missing URI") {
        expectedError = "Missing NNArchive UDO asset URI";
    }
    SECTION("incompatible library") {
        const auto library = network->getAssetManager().set("bad-registration", std::vector<uint8_t>{1});
        network->properties.udoLibraryUris["libReg.so"] = library->getRelativeUri();
        expectedError = "not a compatible AArch64 ELF shared library";
    }
    const auto setLibrary = [&](const std::string& path, bool hexagon) {
        network->properties.udoLibraryUris[path] = network->getAssetManager().set(path, unloadableElf(hexagon))->getRelativeUri();
    };
    SECTION("AArch64 DSP library without the _AltPrep name") {
        setLibrary("libReg.so", false);
        setLibrary("libImpl.so", false);
        expectedError = "is an AArch64 library";
    }
    SECTION("package with only an _AltPrep library") {
        network->properties.udoPackages = {{"libReg.so", {"libImpl_AltPrep.so"}}};
        setLibrary("libReg.so", false);
        setLibrary("libImpl_AltPrep.so", false);
        expectedError = "has no Hexagon DSP implementation";
    }
    REQUIRE_THROWS_WITH(pipeline.start(), ContainsSubstring(expectedError));
}
