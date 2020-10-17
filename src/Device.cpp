#include "Device.hpp"

// shared
#include "depthai-shared/Assets.hpp"
#include "depthai-shared/cnn_info.hpp"
#include "depthai-shared/depthai_constants.hpp"
#include "depthai-shared/json_helper.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"

// project
#include "pipeline/Pipeline.hpp"
//#include "pipeline/host_pipeline_config.hpp"
#include "nnet/tensor_info_helper.hpp"
#include "pipeline/host_pipeline_config.hpp"

extern "C" {
#include "bspatch/bspatch.h"
}

// Resource compiled assets (cmds)
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES
    #include "cmrc/cmrc.hpp"
CMRC_DECLARE(depthai);
#endif

namespace dai {

constexpr static auto CMRC_DEPTHAI_CMD_PATH = "depthai.cmd";
constexpr static auto CMRC_DEPTHAI_USB2_CMD_PATH = "depthai-usb2.cmd";
constexpr static auto CMRC_DEPTHAI_USB2_PATCH_PATH = "depthai-usb2-patch.patch";

// static api
/*
std::vector<DeviceInfo> Device::getAllConnectedDevices(){
    return XLinkConnection::getAllConnectedDevices();
}

std::tuple<bool, DeviceInfo> Device::getFirstAvailableDevice(){
    return XLinkConnection::getFirstAvailableDevice();
}

std::vector<DeviceInfo> Device::getAllAvailableDevices(){
    return XLinkConnection::getAllAvailableDevices();
}

std::tuple<bool, DeviceInfo> Device::getFirstDevice(){
    return XLinkConnection::getFirstAvailableDevice();
}
*/

Device::Device(const DeviceInfo& devInfo, bool usb2Mode) {
    connection = std::make_shared<XLinkConnection>(devInfo, getDefaultCmdBinary(usb2Mode));
    this->deviceInfo = devInfo;
    init();
}

Device::Device(const DeviceInfo& devInfo, const char* pathToCmd) {
    connection = std::make_shared<XLinkConnection>(devInfo, std::string(pathToCmd));
    this->deviceInfo = devInfo;
    init();
}

Device::Device(const DeviceInfo& devInfo, const std::string& pathToCmd) {
    connection = std::make_shared<XLinkConnection>(devInfo, pathToCmd);
    this->deviceInfo = devInfo;
    init();
}

Device::Device() {
    bool found = false;
    DeviceInfo devInfo;
    std::tie(found, devInfo) = XLinkConnection::getFirstDevice(X_LINK_UNBOOTED);
    if(!found) throw std::runtime_error("No unbooted devices available");
    connection = std::make_shared<XLinkConnection>(devInfo, getDefaultCmdBinary(false));
    this->deviceInfo = devInfo;
    init();
}

Device::~Device() {
    // Stop watchdog first
    watchdogRunning = false;
    if(watchdogThread.joinable()) watchdogThread.join();

    // Then stop timesync
    timesyncRunning = false;
    if(timesyncThread.joinable()) timesyncThread.join();
}

void Device::init() {
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
    });

    // prepare timesync thread, which will keep device synchronized
    timesyncThread = std::thread([this]() {
        using namespace std::chrono;
        std::shared_ptr<XLinkConnection> conn = this->connection;
        std::string streamName = XLINK_CHANNEL_TIMESYNC;

        try {
            conn->openStream(streamName, 128);
            struct ts {
                int64_t sec, nsec;
            } timestamp;
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

std::vector<std::uint8_t> Device::getDefaultCmdBinary(bool usb2Mode) {
    std::vector<std::uint8_t> finalCmd;

// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    // Get binaries from internal sources
    auto fs = cmrc::depthai::get_filesystem();

    if(usb2Mode) {
    #ifdef DEPTHAI_PATCH_ONLY_MODE

        // Get size of original
        auto depthai_binary = fs.open(CMRC_DEPTHAI_CMD_PATH);

        // Open patch
        auto depthai_usb2_patch = fs.open(CMRC_DEPTHAI_USB2_PATCH_PATH);

        // Get new size
        int64_t patched_size = bspatch_mem_get_newsize(reinterpret_cast<const uint8_t*>(depthai_usb2_patch.begin()), depthai_usb2_patch.size());

        // Reserve space for patched binary
        finalCmd.resize(patched_size);

        // Patch
        int error = bspatch_mem(reinterpret_cast<const uint8_t*>(depthai_binary.begin()),
                                depthai_binary.size(),
                                reinterpret_cast<const uint8_t*>(depthai_usb2_patch.begin()),
                                depthai_usb2_patch.size(),
                                finalCmd.data());

        // if patch not successful
        if(error > 0) throw std::runtime_error("Error while patching cmd for usb2 mode");

    #else

        auto depthai_usb2_binary = fs.open(CMRC_DEPTHAI_USB2_CMD_PATH);
        finalCmd = std::vector<std::uint8_t>(depthai_usb2_binary.begin(), depthai_usb2_binary.end());

    #endif

    } else {
        auto depthai_binary = fs.open(CMRC_DEPTHAI_CMD_PATH);
        finalCmd = std::vector<std::uint8_t>(depthai_binary.begin(), depthai_binary.end());
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
