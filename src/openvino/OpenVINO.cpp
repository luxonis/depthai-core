
#include "depthai/openvino/OpenVINO.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "BlobReader.hpp"
#include "spdlog/spdlog.h"
#include "utility/Logging.hpp"
#include "utility/spdlog-fmt.hpp"

extern "C" {
#include "../bspatch/bspatch.h"
}

namespace dai {

constexpr OpenVINO::Version OpenVINO::DEFAULT_VERSION;

const std::map<std::pair<std::uint32_t, std::uint32_t>, OpenVINO::Version> OpenVINO::blobVersionToOpenvinoGuessMapping = {
    {{5, 0}, OpenVINO::VERSION_2020_3},
    {{6, 0}, OpenVINO::VERSION_2022_1},
    {{2020, 3}, OpenVINO::VERSION_2020_3},
    {{2020, 4}, OpenVINO::VERSION_2020_4},
    {{2021, 1}, OpenVINO::VERSION_2021_1},
    {{2021, 2}, OpenVINO::VERSION_2021_2},
    {{2021, 3}, OpenVINO::VERSION_2021_3},
    {{2021, 4}, OpenVINO::VERSION_2021_4},
    {{2022, 1}, OpenVINO::VERSION_2022_1},

};

const std::map<std::pair<std::uint32_t, std::uint32_t>, std::vector<OpenVINO::Version>> OpenVINO::blobVersionToOpenvinoMapping = {
    {{5, 0}, {OpenVINO::VERSION_2020_3, OpenVINO::VERSION_UNIVERSAL}},
    {{6, 0},
     {OpenVINO::VERSION_2020_4,
      OpenVINO::VERSION_2021_1,
      OpenVINO::VERSION_2021_2,
      OpenVINO::VERSION_2021_3,
      OpenVINO::VERSION_2021_4,
      OpenVINO::VERSION_2022_1,
      OpenVINO::VERSION_UNIVERSAL}},
    {{2020, 3}, {OpenVINO::VERSION_2020_3, OpenVINO::VERSION_UNIVERSAL}},
    {{2020, 4}, {OpenVINO::VERSION_2020_4, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 1}, {OpenVINO::VERSION_2021_1, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 2}, {OpenVINO::VERSION_2021_2, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 3}, {OpenVINO::VERSION_2021_3, OpenVINO::VERSION_UNIVERSAL}},
    {{2021, 4}, {OpenVINO::VERSION_2021_4, OpenVINO::VERSION_UNIVERSAL}},
    {{2022, 1}, {OpenVINO::VERSION_2022_1, OpenVINO::VERSION_UNIVERSAL}},

};

static uint64_t bigEndianToHost(uint64_t value) {
    const int num = 42;  // 0x0000002A when using big-endian byte order
    const bool hostIsBigEndian = *reinterpret_cast<const char*>(&num) == 0;

    if(hostIsBigEndian) {
        return value;
    }

    uint64_t result = 0;
    for(size_t i = 0; i < sizeof(uint64_t); ++i) {
        result |= (value & 0xFF) << (8 * (sizeof(uint64_t) - i - 1));
        value >>= 8;
    }
    return result;
}

static uint64_t readInt64(const uint8_t* data) {
    uint64_t value = *reinterpret_cast<const uint64_t*>(data);
    return bigEndianToHost(value);
}

OpenVINO::SuperBlob::SuperBlob(const std::filesystem::path& pathToSuperBlobFile) {
    data = readSuperBlobFile(pathToSuperBlobFile);
    loadAndCheckHeader();
    validateSuperblob();
}

OpenVINO::SuperBlob::SuperBlob(std::vector<uint8_t> data) {
    this->data = std::move(data);
    loadAndCheckHeader();
    validateSuperblob();
}

dai::OpenVINO::Blob OpenVINO::SuperBlob::getBlobWithNumShaves(int numShaves) {
    if(numShaves < 1 || numShaves > static_cast<int>(OpenVINO::SuperBlob::NUMBER_OF_PATCHES)) {
        throw std::out_of_range("Invalid number of shaves: " + std::to_string(numShaves) + " (expected 1 to "
                                + std::to_string(OpenVINO::SuperBlob::NUMBER_OF_PATCHES) + ")");
    }

    const uint8_t* blobData = getBlobDataPointer();
    int64_t blobSize = getBlobDataSize();

    const uint8_t* patchData = getPatchDataPointer(numShaves);
    int64_t patchSize = getPatchDataSize(numShaves);

    std::vector<uint8_t> patchedBlobData;

    if(patchSize != 0) {  // patchSize 0: the main blob is already compiled for numShaves
        int64_t patchedBlobSize = bspatch_mem_get_newsize(patchData, patchSize);
        patchedBlobData.resize(patchedBlobSize);

        bspatch_mem(blobData, blobSize, patchData, patchSize, patchedBlobData.data());
    } else {
        patchedBlobData.resize(blobSize);
        std::copy(blobData, blobData + blobSize, patchedBlobData.data());
    }

    dai::OpenVINO::Blob patchedBlob(patchedBlobData);
    return patchedBlob;
}

std::vector<uint8_t> OpenVINO::SuperBlob::readSuperBlobFile(const std::filesystem::path& path) {
    if(!std::filesystem::exists(path)) throw std::runtime_error("File does not exist: " + path.string());

    std::ifstream file(path, std::ios::binary);
    if(!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + path.string());
    }
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}

OpenVINO::SuperBlob::SuperBlobHeader OpenVINO::SuperBlob::SuperBlobHeader::fromData(const std::vector<uint8_t>& data) {
    SuperBlobHeader header;
    const uint8_t* ptr = data.data();
    header.blobSize = readInt64(ptr);
    ptr += sizeof(uint64_t);

    header.patchSizes.resize(OpenVINO::SuperBlob::NUMBER_OF_PATCHES);
    for(size_t i = 0; i < OpenVINO::SuperBlob::NUMBER_OF_PATCHES; ++i) {
        header.patchSizes[i] = readInt64(ptr);
        ptr += sizeof(uint64_t);
    }
    return header;
}

const uint8_t* OpenVINO::SuperBlob::getBlobDataPointer() {
    const uint64_t offset = SuperBlobHeader::HEADER_SIZE;
    return data.data() + offset;
}

int64_t OpenVINO::SuperBlob::getBlobDataSize() {
    return header.blobSize;
}

const uint8_t* OpenVINO::SuperBlob::getPatchDataPointer(int numShaves) {
    const uint64_t offset =
        SuperBlobHeader::HEADER_SIZE + header.blobSize + std::accumulate(header.patchSizes.begin(), header.patchSizes.begin() + numShaves - 1, 0);
    return data.data() + offset;
}

int64_t OpenVINO::SuperBlob::getPatchDataSize(int numShaves) {
    return header.patchSizes[numShaves - 1];
}

void OpenVINO::SuperBlob::loadAndCheckHeader() {
    if(data.size() < SuperBlobHeader::HEADER_SIZE) {
        throw std::invalid_argument("Invalid superblob data: not enough bytes for the header: " + std::to_string(data.size()) + " Data should be at least "
                                    + std::to_string(SuperBlobHeader::HEADER_SIZE) + " bytes long.");
    }
    header = SuperBlobHeader::fromData(data);
}

void OpenVINO::SuperBlob::validateSuperblob() {
    size_t expectedSize = SuperBlobHeader::HEADER_SIZE + header.blobSize + std::accumulate(header.patchSizes.begin(), header.patchSizes.end(), 0);
    if(expectedSize != data.size()) {
        throw std::invalid_argument("Invalid superblob data: size mismatch. Expected " + std::to_string(expectedSize) + " bytes, got "
                                    + std::to_string(data.size()) + " bytes.");
    }

    for(size_t numShaves = 1; numShaves <= OpenVINO::SuperBlob::NUMBER_OF_PATCHES; ++numShaves) {
        int64_t patchSize = getPatchDataSize(numShaves);
        if(patchSize == 0) continue;
        const uint8_t* patchDataPointer = getPatchDataPointer(numShaves);
        if(memcmp(patchDataPointer, "BSDIFF", 6) != 0) {
            throw std::invalid_argument("Invalid superblob data: patch " + std::to_string(numShaves) + " is not a valid bsdiff patch.");
        }
    }
}

std::vector<OpenVINO::Version> OpenVINO::getVersions() {
    return {OpenVINO::VERSION_2020_3,
            OpenVINO::VERSION_2020_4,
            OpenVINO::VERSION_2021_1,
            OpenVINO::VERSION_2021_2,
            OpenVINO::VERSION_2021_3,
            OpenVINO::VERSION_2021_4,
            OpenVINO::VERSION_2022_1};
}

std::string OpenVINO::getVersionName(OpenVINO::Version version) {
    switch(version) {
        case OpenVINO::VERSION_2020_3:
            return "2020.3";
        case OpenVINO::VERSION_2020_4:
            return "2020.4";
        case OpenVINO::VERSION_2021_1:
            return "2021.1";
        case OpenVINO::VERSION_2021_2:
            return "2021.2";
        case OpenVINO::VERSION_2021_3:
            return "2021.3";
        case OpenVINO::VERSION_2021_4:
            return "2021.4";
        case OpenVINO::VERSION_2022_1:
            return "2022.1";
        case OpenVINO::VERSION_UNIVERSAL:
            return "universal";
    }
    throw std::logic_error("OpenVINO - Unknown version enum specified");
}

OpenVINO::Version OpenVINO::parseVersionName(const std::string& versionString) {
    auto versions = getVersions();
    for(const auto& v : versions) {
        if(versionString == getVersionName(v)) {
            return v;
        }
    }
    throw std::logic_error("OpenVINO - Cannot parse version name: " + versionString);
}

std::vector<OpenVINO::Version> OpenVINO::getBlobSupportedVersions(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    std::pair<std::uint32_t, std::uint32_t> blobVersion;
    blobVersion.first = majorVersion;
    blobVersion.second = minorVersion;

    if(blobVersionToOpenvinoMapping.count(blobVersion) > 0) {
        return blobVersionToOpenvinoMapping.at(blobVersion);
    }
    return {};
}

OpenVINO::Version OpenVINO::getBlobVersion(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    std::pair<std::uint32_t, std::uint32_t> blobVersion;
    blobVersion.first = majorVersion;
    blobVersion.second = minorVersion;

    return blobVersionToOpenvinoGuessMapping.at(blobVersion);
}

OpenVINO::Version OpenVINO::getBlobLatestSupportedVersion(std::uint32_t majorVersion, std::uint32_t minorVersion) {
    (void)majorVersion;
    (void)minorVersion;
    return OpenVINO::VERSION_UNIVERSAL;
}

bool OpenVINO::areVersionsBlobCompatible(OpenVINO::Version v1, OpenVINO::Version v2) {
    if(v1 == VERSION_UNIVERSAL || v2 == VERSION_UNIVERSAL) {
        return true;
    }

    for(const auto& kv : blobVersionToOpenvinoMapping) {
        bool v1Found = false;
        bool v2Found = false;

        for(const auto& el : blobVersionToOpenvinoMapping.at(kv.first)) {
            if(el == v1) v1Found = true;
            if(el == v2) v2Found = true;
        }

        if(v1Found && v2Found) {
            return true;
        } else if(!v1Found && !v2Found) {
            continue;
        } else {
            return false;
        }
    }

    logger::error("OpenVINO - version compatibility check with invalid values or unknown blob version");
    return false;
}

static void blobInit(OpenVINO::Blob& blob, std::vector<uint8_t> data) {
    blob.data = std::move(data);

    std::vector<uint8_t> vpuxBlobSkip{0X18, 0X0, 0X0, 0X0};
    std::vector<uint8_t> vpuxBlobHeader{0X42, 0X4C, 0X4F, 0X42};

    if(std::equal(vpuxBlobHeader.begin(), vpuxBlobHeader.end(), blob.data.begin() + vpuxBlobSkip.size())) {
        blob.device = OpenVINO::Device::VPUX;     // VPUX blobs are parsed on the device
        blob.version = OpenVINO::VERSION_2022_1;  // TODO: parse blob to get the version
    } else {
        blob.device = OpenVINO::Device::VPU;
        BlobReader reader;
        reader.parse(blob.data);
        blob.networkInputs = reader.getNetworkInputs();
        blob.networkOutputs = reader.getNetworkOutputs();
        blob.stageCount = reader.getStageCount();
        blob.numShaves = reader.getNumberOfShaves();
        blob.numSlices = reader.getNumberOfSlices();
        blob.version = OpenVINO::getBlobVersion(reader.getVersionMajor(), reader.getVersionMinor());
    }
}

OpenVINO::Blob::Blob(std::vector<uint8_t> data) {
    blobInit(*this, std::move(data));
}

OpenVINO::Blob::Blob(const std::filesystem::path& path) {
    std::ifstream stream(path, std::ios::in | std::ios::binary);
    if(!stream.is_open()) {
        // Throw an error
        // TODO(themarpe) - Unify exceptions into meaningful groups
        throw std::runtime_error(fmt::format("Cannot load blob, file at path {} doesn't exist.", path));
    }
    blobInit(*this, std::vector<std::uint8_t>(std::istreambuf_iterator<char>(stream), {}));
}

}  // namespace dai
