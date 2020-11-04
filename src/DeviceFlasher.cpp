#include "DeviceFlasher.hpp"

// shared
#include "depthai-shared/Assets.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/SBR.h"


// project
#include "pipeline/Pipeline.hpp"
#include "BootloaderHelper.hpp"
#include "bootloader/Version.hpp"
extern "C" {
#include "bspatch/bspatch.h"
}

// Resource compiled assets (cmds)
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES
    #include "cmrc/cmrc.hpp"
CMRC_DECLARE(depthai);
#endif

namespace dai {


// static api

// First tries to find UNBOOTED device, then BOOTLOADER device
std::tuple<bool, DeviceInfo> DeviceFlasher::getFirstAvailableDevice(){
    bool found;
    DeviceInfo dev;
    std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_UNBOOTED);
    if(!found){
        std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_BOOTLOADER);
    }
    return {found, dev};
}

// Returns all devices which aren't already booted
std::vector<DeviceInfo> DeviceFlasher::getAllAvailableDevices(){
    std::vector<DeviceInfo> availableDevices;
    auto connectedDevices = XLinkConnection::getAllConnectedDevices();
    for(const auto& d : connectedDevices){
        if(d.state != X_LINK_BOOTED) availableDevices.push_back(d);
    }
    return availableDevices;
}

std::vector<uint8_t> DeviceFlasher::createDepthaiApplicationPackage(Pipeline& pipeline, std::string pathToCmd){

    // Prepare device firmware
    std::vector<uint8_t> deviceFirmware;
    if(pathToCmd != ""){
        std::ifstream fwStream(pathToCmd, std::ios::in | std::ios::binary);
        if(!blobStream.is_open()) throw std::runtime_error("Cannot create application package, device firmware at path: " + pathToCmd + " doesn't exist");
        deviceFirmware = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(blobStream), {});
    } else {
        deviceFirmware = getEmbeddedBootloaderBinary();
    }

    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    pipeline.serialize(schema, assets, assetStorage);

    std::vector<uint8_t> pipelineBinary, assetsBinary;

    // Create msgpacks
    {
        nlohmann::json j = schema;
        pipelineBinary = nlohmann::json::to_msgpack(j);
    }
    {
        nlohmann::json j = assets;
        assetsBinary = nlohmann::json::to_msgpack(j);
    }

    // Prepare SBR structure
    SBR sbr = {};
    SBR_SECTION* fwSection = &sbr.sections[0];
    SBR_SECTION* pipelineSection = &sbr.sections[1];
    SBR_SECTION* assetsSection = &sbr.sections[2];
    SBR_SECTION* assetStorageSection = &sbr.sections[3];
    SBR_SECTION* lastSection = assetStorageSection;


    // Alignup for easier updating
    constexpr long SECTION_ALIGNMENT_SIZE = 1 * 1024* 1024; // 1MiB for easier updating
    auto getSectionAlignedOffset = [SECTION_ALIGNMENT_SIZE](long S){
        return ((((S) + (SECTION_ALIGNMENT_SIZE) - 1)) & ~((SECTION_ALIGNMENT_SIZE) - 1));
    };

    // First section, MVCMD, name '__firmware'
    sbr_section_set_name(fwSection, "__firmware");
    sbr_section_set_bootable(fwSection, true);
    sbr_section_set_size(fwSection, deviceFirmware.size());
    sbr_section_set_checksum(fwSection, sbr_compute_checksum(deviceFirmware.data(), deviceFirmware.size()));
    sbr_section_set_offset(fwSection, SBR_RAW_SIZE);

    // Second section, pipeline schema, name 'pipeline'
    sbr_section_set_name(pipelineSection, "pipeline");
    sbr_section_set_size(pipelineSection, pipelineBinary.size());
    sbr_section_set_checksum(pipelineSection, sbr_compute_checksum(pipelineBinary.data(), pipelineBinary.size()));
    sbr_section_set_offset(pipelineSection, getSectionAlignedOffset(fwSection->offset + fwSection->size));

    // Third section, assets map, name 'assets'
    sbr_section_set_name(assetsSection, "assets");
    sbr_section_set_size(assetsSection, assetsBinary.size());
    sbr_section_set_checksum(assetsSection, sbr_compute_checksum(assetsBinary.data(), assetsBinary.size()));
    sbr_section_set_offset(assetsSection, getSectionAlignedOffset(pipelineSection->offset + pipelineSection->size));

    // Fourth section, asset storage, name 'asset_storage'
    sbr_section_set_name(assetStorageSection, "asset_storage");
    sbr_section_set_size(assetStorageSection, assetStorage.size());
    sbr_section_set_checksum(assetStorageSection, sbr_compute_checksum(assetStorage.data(), assetStorage.size()));
    sbr_section_set_offset(assetStorageSection, getSectionAlignedOffset(assetsSection->offset + assetsSection->size));

    // TODO(themarpe) - Add additional sections (in case blobs are loaded into separate sections)

    // Create a vector to hold whole dap package
    vector<uint8_t> fwPackage;
    fwPackage.resize(lastSection->offset + lastSection->size);

    // Serialize SBR
    sbr_serialize(&sbr, fwPackage.data(), fwPackage.size());

    // Write to fwPackage
    for(unsigned i = 0; i < deviceFirmware.size(); i++) fwPackage[fwSection->offset + i] = deviceFirmware[i];
    for(unsigned i = 0; i < pipelineBinary.size(); i++) fwPackage[pipelineSection->offset + i] = pipelineBinary[i];
    for(unsigned i = 0; i < assetsBinary.size(); i++) fwPackage[assetsSection->offset + i] = assetsBinary[i];
    for(unsigned i = 0; i < assetStorage.size(); i++) fwPackage[assetStorageSection->offset + i] = assetStorage[i];

    return fwPackage;
}



DeviceFlasher::DeviceFlasher(const DeviceInfo& devInfo) : deviceInfo(devInfo) {
    init(true, "");
}

DeviceFlasher::DeviceFlasher(const DeviceInfo& devInfo, const char* pathToBootloader) : deviceInfo(devInfo) {
    init(false, std::string(pathToBootloader));
}

DeviceFlasher::DeviceFlasher(const DeviceInfo& devInfo, const std::string& pathToBootloader) : deviceInfo(devInfo) {
    init(false, pathToBootloader);
}

DeviceFlasher::~DeviceFlasher() {
    // Stop watchdog first
    watchdogRunning = false;
    if(watchdogThread.joinable()) watchdogThread.join();
}


void DeviceFlasher::init(bool embeddedMvcmd, const std::string& pathToMvcmd) {
    
    // Init device (if bootloader, handle correctly - issue USB boot command)
    if(deviceInfo.state == X_LINK_UNBOOTED){
        // Unbooted device found, boot to BOOTLOADER and connect with XLinkConnection constructor
        if(embeddedMvcmd){
            connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedBootloaderBinary(), X_LINK_BOOTLOADER);
        } else{
            connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);
        }
    } else if(deviceInfo.state == X_LINK_BOOTLOADER){

        // Device already in bootloader mode. 
        // Connect without booting
        connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd, X_LINK_BOOTLOADER);

            // Open stream
            bootloaderConnection.openStream(bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);
        
        deviceInfo.state = X_LINK_BOOTLOADER;

    } else {
        throw std::runtime_error("Device not in UNBOOTED or BOOTLOADER state");
    }

    deviceInfo.state = X_LINK_BOOTLOADER;

    // prepare watchdog thread
    connection->openStream(bootloader::XLINK_CHANNEL_WATCHDOG, 64);

    // prepare watchdog thread, which will keep device alive
    watchdogThread = std::thread([this]() {
        std::shared_ptr<XLinkConnection> conn = this->connection;
        std::vector<uint8_t> dummyData = {0,0,0,0};
        while(watchdogRunning) {
            try {
                connection->writeToStream(bootloader::XLINK_CHANNEL_WATCHDOG, dummyData);
            } catch(const std::exception& ex) {
                break;
            }
            // Ping with a period half of that of the watchdog timeout
            std::this_thread::sleep_for(bootloader::XLINK_WATCHDOG_TIMEOUT / 2);
        }
    });

    // prepare bootloader stream
    connection->openStream(bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);

}


bootloader::Version DeviceFlasher::getEmbeddedBootloaderVersion(){
    return bootloader::Version(DEPTHAI_BOOTLOADER_VERSION);
}

bootloader::Version DeviceFlasher::getBootloaderVersion(){
    streamId_t streamId = bootloaderConnection.getStreamId(bootloader::XLINK_CHANNEL_BOOTLOADER);

    // Send request to jump to USB bootloader
    if(!sendBootloaderRequest(streamId, bootloader::request::GetBootloaderVersion{})){
        throw std::runtime_error("Couldn't get bootloader version");
    }

    // Receive response
    dai::bootloader::response::BootloaderVersion ver;
    if(!receiveBootloaderResponse(streamId, ver)){
        throw std::runtime_error("Couldn't get bootloader version");  
    }

    // Create bootloader::Version object and return
    return bootloader::Version(ver.major, ver.minor, ver.patch);
}


std::tuple<bool, std::string> DeviceFlasher::flash(std::function<void(float)> progressCb, Pipeline& pipeline){
    create
}


std::tuple<bool, std::string> DeviceFlasher::flashDepthaiApplicationPackage(std::function<void(float)> progressCb, std::vector<uint8_t> package){
    streamId_t streamId = connection->getStreamId(bootloader::XLINK_CHANNEL_BOOTLOADER);

    // send request to FLASH BOOTLOADER
    dai::bootloader::request::UpdateFlash updateFlash;
    updateFlash.storage = dai::bootloader::request::UpdateFlash::SBR;
    updateFlash.totalSize = package.size();
    updateFlash.numPackets = ((package.size() - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendBootloaderRequest(streamId, updateFlash)) return false;

    // After that send numPackets of data
    connection->writeToStreamSplit(bootloader::XLINK_CHANNEL_BOOTLOADER, package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    dai::bootloader::response::FlashComplete result;
    do {
        std::vector<uint8_t> data;
        if(!receiveBootloaderResponseData(streamId, data)) return false;
        
        dai::bootloader::response::FlashStatusUpdate update;
        if(parseBootloaderResponse(data, update)){
            // if progress callback is set
            if(progressCb != nullptr){
                progressCb(update.progress);
            }
        } else if(parseBootloaderResponse(data, result)){
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}


std::tuple<bool, std::string> DeviceFlasher::flashBootloader(std::function<void(float)> progressCb, std::string path){

    std::vector<uint8_t> package;
    if(path != ""){
        std::ifstream fwStream(path, std::ios::in | std::ios::binary);
        if(!blobStream.is_open()) throw std::runtime_error("Cannot flash bootloader, binary at path: " + path + " doesn't exist");
        package = std::vector<std::uint8_t>(std::istreambuf_iterator<char>(blobStream), {});
    } else {
        package = getEmbeddedBootloaderBinary();
    }

     // get streamId
    streamId_t streamId = connection->getStreamId(bootloader::XLINK_CHANNEL_BOOTLOADER);

    // send request to FLASH BOOTLOADER
    dai::bootloader::request::UpdateFlash updateFlash;
    updateFlash.storage = dai::bootloader::request::UpdateFlash::BOOTLOADER;
    updateFlash.totalSize = package.size();
    updateFlash.numPackets = ((package.size() - 1) / bootloader::XLINK_STREAM_MAX_SIZE) + 1;
    if(!sendBootloaderRequest(streamId, updateFlash)) return false;

    // After that send numPackets of data
    connection->writeToStreamSplit(bootloader::XLINK_CHANNEL_BOOTLOADER, package.data(), package.size(), bootloader::XLINK_STREAM_MAX_SIZE);

    // Then wait for response by bootloader
    // Wait till FLASH_COMPLETE response
    dai::bootloader::response::FlashComplete result;
    do {
        std::vector<uint8_t> data;
        if(!receiveBootloaderResponseData(streamId, data)) return false;
        
        dai::bootloader::response::FlashStatusUpdate update;
        if(parseBootloaderResponse(data, update)){
            // if progress callback is set
            if(progressCb != nullptr){
                progressCb(update.progress);
            }
        // if flash complete response arrived, break from while loop
        } else if(parseBootloaderResponse(data, result)){
            break;
        } else {
            // Unknown response, shouldn't happen
            return {false, "Unknown response from bootloader while flashing"};
        }

    } while(true);

    // Return if flashing was successful
    return {result.success, result.errorMsg};
}


bool Device::startPipeline(Pipeline& pipeline) {
    // first check if pipeline is not already started
    if(isPipelineRunning()) return false;

    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    pipeline.serialize(schema, assets, assetStorage);

    // if debug
    if(false) {
        nlohmann::json jSchema = schema;
        std::cout << std::endl << jSchema.dump(4) << std::endl;

        nlohmann::json jAssets = assets;
        std::cout << std::endl << jAssets.dump(4) << std::endl;
    }

    // Load pipelineDesc, assets, and asset storage
    client->call("setPipelineSchema", schema);

    // Transfer storage != empty
    if(!assetStorage.empty()) {
        client->call("setAssets", assets);

        // allocate, returns a pointer to memory on device side
        auto memHandle = client->call("memAlloc", static_cast<std::uint32_t>(assetStorage.size())).as<uint32_t>();

        // Transfer the whole assetStorage in a separate thread
        const std::string streamAssetStorage = "__stream_asset_storage";
        std::thread t1([this, &streamAssetStorage, &assetStorage]() {
            connection->openStream(streamAssetStorage, XLINK_USB_BUFFER_MAX_SIZE);
            int64_t offset = 0;
            do {
                int64_t toTransfer = std::min(static_cast<int64_t>(XLINK_USB_BUFFER_MAX_SIZE), static_cast<int64_t>(assetStorage.size() - offset));
                connection->writeToStream(streamAssetStorage, &assetStorage[offset], toTransfer);
                offset += toTransfer;
            } while(offset < static_cast<int64_t>(assetStorage.size()));
        });

        // Open a channel to transfer AssetStorage
        client->call("readFromXLink", streamAssetStorage, memHandle, assetStorage.size());
        t1.join();

        // After asset storage is transfers, set the asset storage
        client->call("setAssetStorage", memHandle, assetStorage.size());
    }

    // print assets on device side for test
    client->call("printAssets");

    // Build and start the pipeline
    bool success = false;
    std::string errorMsg;
    std::tie(success, errorMsg) = client->call("buildPipeline").as<std::tuple<bool, std::string>>();
    if(success) {
        client->call("startPipeline");
    } else {
        throw std::runtime_error(errorMsg);
        return false;
    }

    // client->call("startCamera");

    return true;
}

std::vector<std::uint8_t> Device::getEmbeddedBootloadereBinary() {

// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    constexpr static auto CMRC_DEPTHAI_BOOTLOADER_PATH = "depthai-bootloader-" DEPTHAI_BOOTLOADER_VERSION ".cmd";

    // Get binaries from internal sources
    auto fs = cmrc::depthai::get_filesystem();

    auto bootloaderBinary = fs.open(CMRC_DEPTHAI_CMD_PATH);
    return std::vector<std::uint8_t>(bootloaderBinary.begin(), bootloaderBinary.end());

#else
    assert(0 && "Unsupported");
    return {};
#endif

}

bool Device::startTestPipeline(int testId) {
    // first check if pipeline is not already started
    if(isPipelineRunning()) return false;

    /*

    // Create an AssetManager which the pipeline will use for assets
    AssetManager assetManager;
    pipeline.loadAssets(assetManager);

    // Serialize the pipeline
    auto pipelineDescription = pipeline.serialize();

    // Serialize the asset storage and assets
    auto assetStorage = assetManager.serialize();
    std::vector<std::uint8_t> assets;
    {
        nlohmann::json assetsJson;
        nlohmann::to_json(assetsJson, (Assets) assetManager);
        assets = nlohmann::json::to_msgpack(assetsJson);
    }


    */

    using namespace nlohmann;
    nlohmann::json pipelineDescJson;

    if(testId == 0) {
        pipelineDescJson = R"(
            {
                "globalProperties": {
                    "leonOsFrequencyHz": 600000000,
                    "pipelineVersion": "1",
                    "pipelineName": "1",
                    "leonRtFrequencyHz": 600000000
                },
                "nodes": [
                    {
                        "id": 1,
                        "name": "MyProducer",
                        "properties": {
                            "message": "HeiHoi",
                            "processorPlacement": 0
                        }
                    },
                    {
                        "id": 2,
                        "name": "MyConsumer",
                        "properties": {
                            "processorPlacement": 1
                        }
                    },
                    {
                        "id": 3,
                        "name": "MyConsumer",
                        "properties": {
                            "processorPlacement": 0
                        }
                    },
                    {
                        "id": 4,
                        "name": "MyConsumer",
                        "properties": {
                            "processorPlacement": 1
                        }
                    }
                ],
                "connections": [
                    {
                        "node1Id": 1,
                        "node2Id": 2,
                        "node1Output": "out",
                        "node2Input": "in"
                    },
                    {
                        "node1Id": 1,
                        "node2Id": 3,
                        "node1Output": "out",
                        "node2Input": "in"
                    },
                    {
                        "node1Id": 1,
                        "node2Id": 4,
                        "node1Output": "out",
                        "node2Input": "in"
                    }
                ]
            }
            )"_json;
    } else if(testId == 1) {
        pipelineDescJson = R"(
        {
            "globalProperties": {
                "leonOsFrequencyHz": 600000000,
                "pipelineVersion": "1",
                "pipelineName": "1",
                "leonRtFrequencyHz": 600000000
            },
            "nodes": [
                {
                    "id": 1,
                    "name": "MyProducer",
                    "properties": {
                        "message": "HeiHoi",
                        "processorPlacement": 0
                    }
                },
                {
                    "id": 2,
                    "name": "MyConsumer",
                    "properties": {
                        "processorPlacement": 1
                    }
                }
            ],
            "connections": [
                {
                    "node1Id": 1,
                    "node2Id": 2,
                    "node1Output": "out",
                    "node2Input": "in"
                }
            ]
        }
        )"_json;

    } else if(testId == 2) {
        pipelineDescJson = R"({
            "connections": [
                {
                    "node1Id": 0,
                    "node1Output": "out",
                    "node2Id": 1,
                    "node2Input": "in"
                }
            ],
            "globalProperties": {
                "leonOsFrequencyHz": 600000000.0,
                "leonRtFrequencyHz": 600000000.0,
                "pipelineName": null,
                "pipelineVersion": null
            },
            "nodes": [
                {
                    "id": 0,
                    "name": "XLinkIn",
                    "properties": {
                        "streamName": "nn_in"
                    }
                },
                {
                    "id": 1,
                    "name": "MyConsumer",
                    "properties": {
                        "processorPlacement": 1
                    }
                },
                {
                    "id": 2,
                    "name": "XLinkOut",
                    "properties": {
                        "maxFpsLimit": -1.0,
                        "streamName": "nn_out"
                    }
                }
            ]
        })"_json;
    }

    std::vector<std::uint8_t> assetStorage;
    Assets assets;
    PipelineSchema pipelineSchema = pipelineDescJson;

    // Load pipelineDesc, assets, and asset storage

    client->call("setPipelineSchema", pipelineSchema);

    // Transfer storage if size > 0
    if(!assetStorage.empty()) {
        client->call("setAssets", assets);

        // allocate, returns a pointer to memory on device side
        auto memHandle = client->call("memAlloc", static_cast<std::uint32_t>(assetStorage.size())).as<uint32_t>();

        // Transfer the whole assetStorage in a separate thread
        const std::string streamAssetStorage = "__stream_asset_storage";
        std::thread t1([this, &streamAssetStorage, &assetStorage]() {
            connection->openStream(streamAssetStorage, XLINK_USB_BUFFER_MAX_SIZE);
            uint64_t offset = 0;
            do {
                uint64_t toTransfer = std::min((uint64_t)XLINK_USB_BUFFER_MAX_SIZE, assetStorage.size() - offset);
                connection->writeToStream(streamAssetStorage, assetStorage.data() + offset, toTransfer);
                offset += toTransfer;
            } while(offset < assetStorage.size());
        });

        // Open a channel to transfer AssetStorage
        client->call("readFromXLink", streamAssetStorage, memHandle, assetStorage.size());
        t1.join();

        // After asset storage is transfers, set the asset storage
        client->call("setAssetStorage", memHandle, assetStorage.size());
    }

    // call test
    // client->call("test");

    // Build and start the pipeline
    bool success = false;
    std::string errorMsg;
    std::tie(success, errorMsg) = client->call("buildPipeline").as<std::tuple<bool, std::string>>();
    if(success) {
        client->call("startPipeline");
        return true;
    } else {
        throw std::runtime_error(errorMsg);
        return false;
    }

    // client->call("startCamera");
}



}  // namespace dai
