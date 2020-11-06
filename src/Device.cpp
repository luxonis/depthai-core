#include "Device.hpp"

// shared
#include "depthai-bootloader-shared/Bootloader.hpp"
#include "depthai-bootloader-shared/XLinkConstants.hpp"
#include "depthai-shared/Assets.hpp"
#include "depthai-shared/datatype/RawImgFrame.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "BootloaderHelper.hpp"
#include "pipeline/Pipeline.hpp"
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
std::tuple<bool, DeviceInfo> Device::getFirstAvailableDevice() {
    bool found;
    DeviceInfo dev;
    std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_UNBOOTED);
    if(!found) {
        std::tie(found, dev) = XLinkConnection::getFirstDevice(X_LINK_BOOTLOADER);
    }
    return {found, dev};
}

// Returns all devices which aren't already booted
std::vector<DeviceInfo> Device::getAllAvailableDevices() {
    std::vector<DeviceInfo> availableDevices;
    auto connectedDevices = XLinkConnection::getAllConnectedDevices();
    for(const auto& d : connectedDevices) {
        if(d.state != X_LINK_BOOTED) availableDevices.push_back(d);
    }
    return availableDevices;
}

/*
std::vector<DeviceInfo> Device::getAllConnectedDevices(){
    return XLinkConnection::getAllConnectedDevices();
}


std::tuple<bool, DeviceInfo> Device::getFirstDevice(){
    return XLinkConnection::getFirstAvailableDevice();
}
*/

Device::Device(const DeviceInfo& devInfo, bool usb2Mode) : deviceInfo(devInfo) {
    init(true, usb2Mode, "");
}

Device::Device(const DeviceInfo& devInfo, const char* pathToCmd) : deviceInfo(devInfo) {
    init(false, false, std::string(pathToCmd));
}

Device::Device(const DeviceInfo& devInfo, const std::string& pathToCmd) : deviceInfo(devInfo) {
    init(false, false, pathToCmd);
}

Device::Device() {
    // Default constructor, gets first unconnected device
    // First looks for UNBOOTED, then BOOTLOADER then BOOTED
    bool found = false;
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_BOOTED}) {
        std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(searchState);
        if(found) break;
    }

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(true, false, "");
}

Device::Device(const char* pathToCmd) {
    // Default constructor, gets first unconnected device
    // First looks for UNBOOTED, then BOOTLOADER then BOOTED
    bool found = false;
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_BOOTED}) {
        std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(searchState);
        if(found) break;
    }

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(false, false, std::string(pathToCmd));
}

Device::Device(const std::string& pathToCmd) {
    // Default constructor, gets first unconnected device
    // First looks for UNBOOTED, then BOOTLOADER then BOOTED
    bool found = false;
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_BOOTED}) {
        std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(searchState);
        if(found) break;
    }

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(false, false, pathToCmd);
}

Device::Device(bool usb2Mode) {
    // Default constructor, gets first unconnected device
    // First looks for UNBOOTED, then BOOTLOADER then BOOTED
    bool found = false;
    for(auto searchState : {X_LINK_UNBOOTED, X_LINK_BOOTLOADER, X_LINK_BOOTED}) {
        std::tie(found, deviceInfo) = XLinkConnection::getFirstDevice(searchState);
        if(found) break;
    }

    // If no device found, throw
    if(!found) throw std::runtime_error("No available devices");
    init(true, usb2Mode, "");
}

Device::~Device() {
    watchdogRunning = false;
    timesyncRunning = false;

    // Stop watchdog first (this resets and waits for link to fall down)
    if(watchdogThread.joinable()) watchdogThread.join();
    // Then stop timesync
    if(timesyncThread.joinable()) timesyncThread.join();
}

void Device::init(bool embeddedMvcmd, bool usb2Mode, const std::string& pathToMvcmd) {
    // Init device (if bootloader, handle correctly - issue USB boot command)
    if(deviceInfo.state == X_LINK_UNBOOTED) {
        // Unbooted device found, boot and connect with XLinkConnection constructor
        if(embeddedMvcmd) {
            connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedDeviceBinary(usb2Mode));
        } else {
            connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd);
        }

    } else if(deviceInfo.state == X_LINK_BOOTLOADER) {
        // Scope so bootloaderConnection is desctructed and XLink cleans its state
        {
            // Bootloader state, proceed by issuing a command to bootloader
            XLinkConnection bootloaderConnection(deviceInfo, X_LINK_BOOTLOADER);

            // Open stream
            bootloaderConnection.openStream(bootloader::XLINK_CHANNEL_BOOTLOADER, bootloader::XLINK_STREAM_MAX_SIZE);
            streamId_t streamId = bootloaderConnection.getStreamId(bootloader::XLINK_CHANNEL_BOOTLOADER);

            // // Send request for bootloader version
            // if(!sendBootloaderRequest(streamId, bootloader::request::GetBootloaderVersion{})){
            //     throw std::runtime_error("Error trying to connect to device");
            // }
            // // Receive response
            // dai::bootloader::response::BootloaderVersion ver;
            // if(!receiveBootloaderResponse(streamId, ver)) throw std::runtime_error("Error trying to connect to device");

            // Send request to jump to USB bootloader
            // Boot into USB ROM BOOTLOADER NOW
            if(!sendBootloaderRequest(streamId, dai::bootloader::request::UsbRomBoot{})) {
                throw std::runtime_error("Error trying to connect to device");
            }

            // Dummy read, until link falls down and it returns an error code
            streamPacketDesc_t* pPacket;
            XLinkReadData(streamId, &pPacket);
        }

        // After that the state is UNBOOTED
        deviceInfo.state = X_LINK_UNBOOTED;

        // Boot and connect with XLinkConnection constructor
        if(embeddedMvcmd) {
            connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedDeviceBinary(usb2Mode));
        } else {
            connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd);
        }

    } else if(deviceInfo.state == X_LINK_BOOTED) {
        // Connect without booting
        if(embeddedMvcmd) {
            connection = std::make_shared<XLinkConnection>(deviceInfo, getEmbeddedDeviceBinary(usb2Mode));
        } else {
            connection = std::make_shared<XLinkConnection>(deviceInfo, pathToMvcmd);
        }
    } else {
        assert(0 && "Unknown device state");
    }

    deviceInfo.state = X_LINK_BOOTED;

    // prepare rpc for both attached and host controlled mode
    connection->openStream(dai::XLINK_CHANNEL_MAIN_RPC, dai::XLINK_USB_BUFFER_MAX_SIZE);

    client = std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>>(
        new nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>([this](nanorpc::core::type::buffer request) {
            std::unique_lock<std::mutex>(this->rpcMutex);
            // Send request to device
            connection->writeToStream(dai::XLINK_CHANNEL_MAIN_RPC, std::move(request));

            // Receive response back
            // Send to nanorpc to parse
            return connection->readFromStream(dai::XLINK_CHANNEL_MAIN_RPC);
        }));

    // prepare watchdog thread, which will keep device alive
    watchdogThread = std::thread([this]() {
        std::shared_ptr<XLinkConnection> conn = this->connection;
        while(watchdogRunning) {
            try {
                client->call("watchdogKeepalive");
            } catch(const std::exception& ex) {
                break;
            }
            // Ping with a period half of that of the watchdog timeout
            std::this_thread::sleep_for(XLINK_WATCHDOG_TIMEOUT / 2);
        }

        // reset device
        // wait till link falls down
        try {
            client->call("reset");
        } catch(const std::runtime_error& err) {
            // ignore
        }

        // Sleep a bit, so device isn't available anymore
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    });

    // prepare timesync thread, which will keep device synchronized
    timesyncThread = std::thread([this]() {
        using namespace std::chrono;
        std::shared_ptr<XLinkConnection> conn = this->connection;
        std::string streamName = XLINK_CHANNEL_TIMESYNC;

        try {
            conn->openStream(streamName, 128);
            Timestamp timestamp = {};
            while(timesyncRunning) {
                // Block
                conn->readFromStream(streamName);

                // Timestamp
                auto d = std::chrono::steady_clock::now().time_since_epoch();
                timestamp.sec = duration_cast<seconds>(d).count();
                timestamp.nsec = duration_cast<nanoseconds>(d).count() % 1000000000;

                // Write timestamp back
                conn->writeToStream(streamName, &timestamp, sizeof(timestamp));
            }
            conn->closeStream(streamName);
        } catch(const std::exception& ex) {
            // ignore
        }

        timesyncRunning = false;
    });
}

std::shared_ptr<DataOutputQueue> Device::getOutputQueue(const std::string& name, unsigned int maxSize, bool overwrite) {
    // creates a dataqueue if not yet created
    if(outputQueueMap.count(name) == 0) {
        outputQueueMap[name] = std::make_shared<DataOutputQueue>(connection, name, maxSize, overwrite);
    }

    // else just return the shared ptr to this DataQueue
    return outputQueueMap.at(name);
}

std::shared_ptr<DataInputQueue> Device::getInputQueue(const std::string& name, unsigned int maxSize, bool overwrite) {
    // creates a dataqueue if not yet created
    if(inputQueueMap.count(name) == 0) {
        inputQueueMap[name] = std::make_shared<DataInputQueue>(connection, name, maxSize, overwrite);
    }

    // else just return the reference to this DataQueue
    return inputQueueMap.at(name);
}

void Device::setCallback(const std::string& name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb) {
    // creates a CallbackHandler if not yet created
    if(callbackMap.count(name) == 0) {
        // inserts (constructs in-place inside map at queues[name] = DataQueue(connection, name))
        callbackMap.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(connection, name, cb));
    } else {
        // already exists, replace the callback
        callbackMap.at(name).setCallback(cb);
    }
}

bool Device::isPipelineRunning() {
    return client->call("isPipelineRunning").as<bool>();
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

std::vector<std::uint8_t> Device::getEmbeddedDeviceBinary(bool usb2Mode) {
    std::vector<std::uint8_t> finalCmd;

// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    constexpr static auto CMRC_DEPTHAI_CMD_PATH = "depthai-" DEPTHAI_DEVICE_VERSION ".cmd";

    // Get binaries from internal sources
    auto fs = cmrc::depthai::get_filesystem();

    if(usb2Mode) {
    #ifdef DEPTHAI_PATCH_ONLY_MODE

        constexpr static auto CMRC_DEPTHAI_USB2_PATCH_PATH = "depthai-usb2-patch-" DEPTHAI_DEVICE_VERSION ".patch";

        // Get size of original
        auto depthaiBinary = fs.open(CMRC_DEPTHAI_CMD_PATH);

        // Open patch
        auto depthaiUsb2Patch = fs.open(CMRC_DEPTHAI_USB2_PATCH_PATH);

        // Get new size
        int64_t patchedSize = bspatch_mem_get_newsize(reinterpret_cast<const uint8_t*>(depthaiUsb2Patch.begin()), depthaiUsb2Patch.size());

        // Reserve space for patched binary
        finalCmd.resize(patchedSize);

        // Patch
        int error = bspatch_mem(reinterpret_cast<const uint8_t*>(depthaiBinary.begin()),
                                depthaiBinary.size(),
                                reinterpret_cast<const uint8_t*>(depthaiUsb2Patch.begin()),
                                depthaiUsb2Patch.size(),
                                finalCmd.data());

        // if patch not successful
        if(error > 0) throw std::runtime_error("Error while patching cmd for usb2 mode");

    #else

        constexpr static auto CMRC_DEPTHAI_USB2_CMD_PATH = "depthai-usb2-" DEPTHAI_DEVICE_VERSION ".cmd";
        auto depthaiUsb2Binary = fs.open(CMRC_DEPTHAI_USB2_CMD_PATH);
        finalCmd = std::vector<std::uint8_t>(depthaiUsb2Binary.begin(), depthaiUsb2Binary.end());

    #endif

    } else {
        auto depthaiBinary = fs.open(CMRC_DEPTHAI_CMD_PATH);
        finalCmd = std::vector<std::uint8_t>(depthaiBinary.begin(), depthaiBinary.end());
    }

#else
    // Binaries from default path (TODO)

#endif

    return finalCmd;
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
