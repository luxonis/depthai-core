#include "device.hpp"

// shared
#include "depthai-shared/json_helper.hpp"
#include "depthai-shared/depthai_constants.hpp"
#include "depthai-shared/cnn_info.hpp"
#include "depthai-shared/xlink/XLinkConstants.hpp"
#include "depthai-shared/Assets.hpp"

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


namespace dai
{
    


constexpr static auto cmrc_depthai_cmd_path = "depthai.cmd";
constexpr static auto cmrc_depthai_usb2_cmd_path = "depthai-usb2.cmd";
constexpr static auto cmrc_depthai_usb2_patch_path = "depthai-usb2-patch.patch";


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


Device::Device(const DeviceInfo& deviceInfo, bool usb2Mode){
   
    connection = std::make_shared<XLinkConnection>(deviceInfo, getDefaultCmdBinary(usb2Mode));
    this->deviceInfo = deviceInfo;
    init();

}

Device::Device(const DeviceInfo& deviceDesc, std::string pathToCmd){
    
    connection = std::make_shared<XLinkConnection>(deviceDesc, pathToCmd);
    this->deviceInfo = deviceInfo;
    init();

}

Device::Device(){
    auto ret = XLinkConnection::getFirstDevice(X_LINK_UNBOOTED);
    if(!std::get<0>(ret)) throw std::runtime_error("No unbooted devices available");
    connection = std::make_shared<XLinkConnection>(std::get<1>(ret), getDefaultCmdBinary(false));
    this->deviceInfo = deviceInfo;
    init();
}


Device::~Device(){
    deinit();
}


void Device::deinit(){

}


/*


void Device::wdog_thread(int& wd_timeout_ms)
{
    std::cout << "watchdog started " << wd_timeout_ms << std::endl;
    while(wdog_thread_alive)
    {
        wdog_keep = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(wd_timeout_ms));
        if(wdog_keep == 0 && wdog_thread_alive == 1)
        {
            std::cout << "watchdog triggered " << std::endl;
            deinit_device();
            bool init;
            for(int retry = 0; retry < 1; retry++)
            {
                init = init_device(cmd_backup, usb_device_backup, binary_backup, binary_size_backup);
                if(init)
                {
                    break;
                }
            }
            if(!init)
            {
                exit(9);
            }
            create_pipeline(config_backup);
        }
    }

}

int Device::wdog_start(void)
{
    static int once = 1;
    if(once)
    {
        wdog_thread_alive = 1;
        wd_thread = std::thread(&Device::wdog_thread, this, std::ref(wd_timeout_ms)); 
        once = 0;
    }
    return 0;
}
int Device::wdog_stop(void)
{
    if(wdog_thread_alive)
    {
        wdog_thread_alive = 0;
        wd_thread.join();
    }
    return 0;
}

//todo
extern "C" {
    void wdog_keepalive(void)
    {
        //wdog_keep = 1;
    }
};

*/

void Device::init()
{

    // prepare rpc for both attached and host controlled mode
    connection->openStream(dai::XLINK_CHANNEL_MAIN_RPC, dai::XLINK_USB_BUFFER_MAX_SIZE);

    client = std::unique_ptr<nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>>(new nanorpc::core::client<nanorpc::packer::nlohmann_msgpack>(
        [this](nanorpc::core::type::buffer request){
            // Send request to device
            connection->writeToStream(dai::XLINK_CHANNEL_MAIN_RPC, request);

            // Receive response back
            // Send to nanorpc to parse
            return connection->readFromStream(dai::XLINK_CHANNEL_MAIN_RPC);
        }
    ));


}


// asdfasdf
// Do we need a version of this that'll receive from SPI rather than XLINK?
std::shared_ptr<DataOutputQueue> Device::getOutputQueue(std::string name){

    // creates a dataqueue if not yet created
    if(outputQueueMap.count(name) == 0){
        outputQueueMap[name] = std::make_shared<DataOutputQueue>(connection, name);
    }

    // else just return the shared ptr to this DataQueue
    return outputQueueMap.at(name);
}

std::shared_ptr<DataInputQueue> Device::getInputQueue(std::string name){

    // creates a dataqueue if not yet created
    if(inputQueueMap.count(name) == 0){
        inputQueueMap[name] = std::make_shared<DataInputQueue>(connection, name);
    }

    // else just return the reference to this DataQueue
    return inputQueueMap.at(name);
}



void Device::setCallback(std::string name, std::function<std::shared_ptr<RawBuffer>(std::shared_ptr<RawBuffer>)> cb){

    // creates a CallbackHandler if not yet created
    if(callbackMap.count(name) == 0){
        // inserts (constructs in-place inside map at queues[name] = DataQueue(connection, name))
        callbackMap.emplace(std::piecewise_construct, std::forward_as_tuple(name), std::forward_as_tuple(connection, name, cb));
    } else {
        // already exists, replace the callback
        callbackMap.at(name).setCallback(cb);
    }

}



bool Device::isPipelineRunning(){
    return client->call("isPipelineRunning").as<bool>();
}


bool Device::startPipeline(Pipeline& pipeline){

    // first check if pipeline is not already started
    if(isPipelineRunning()) return false;


    // Serialize the pipeline
    PipelineSchema schema;
    Assets assets;
    std::vector<std::uint8_t> assetStorage;
    pipeline.serialize(schema, assets, assetStorage);

    // Load pipelineDesc, assets, and asset storage
    client->call("setPipelineSchema", schema);

    // Transfer storage if size > 0
    if(assetStorage.size() > 0){
        client->call("setAssets", assets);


        // allocate, returns a pointer to memory on device side
        auto memHandle = client->call("memAlloc", (std::uint32_t) assetStorage.size()).as<uint32_t>(); 

        // Transfer the whole assetStorage in a separate thread
        const std::string streamAssetStorage = "__stream_asset_storage";
        std::thread t1([this, &streamAssetStorage, &assetStorage](){
            connection->openStream(streamAssetStorage, XLINK_USB_BUFFER_MAX_SIZE);
            int64_t offset = 0;
            do{
                int64_t toTransfer = std::min( (int64_t) XLINK_USB_BUFFER_MAX_SIZE, (int64_t) assetStorage.size() - offset);
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

    // print assets on device side for test
    client->call("printAssets");

    // Build and start the pipeline
    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = client->call("buildPipeline").as<std::tuple<bool, std::string>>();
    if(success){
        client->call("startPipeline");
    } else {
        throw std::runtime_error(errorMsg);  
        return false;
    }

    client->call("startCamera");

    return true;

}




std::vector<std::uint8_t> Device::getDefaultCmdBinary(bool usb2Mode){
     
    std::vector<std::uint8_t> finalCmd;

    // Binaries are resource compiled
    #ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

        // Get binaries from internal sources
        auto fs = cmrc::depthai::get_filesystem();

        if(usb2Mode){

            #ifdef DEPTHAI_PATCH_ONLY_MODE
            
                // Get size of original
                auto depthai_binary = fs.open(cmrc_depthai_cmd_path);

                // Open patch
                auto depthai_usb2_patch = fs.open(cmrc_depthai_usb2_patch_path);

                // Get new size
                int64_t patched_size = bspatch_mem_get_newsize( (uint8_t*) depthai_usb2_patch.begin(), depthai_usb2_patch.size());

                // Reserve space for patched binary
                finalCmd.resize(patched_size);

                // Patch
                int error = bspatch_mem( (uint8_t*) depthai_binary.begin(), depthai_binary.size(), (uint8_t*) depthai_usb2_patch.begin(), depthai_usb2_patch.size(), finalCmd.data());

                // if patch not successful
                if(error) throw std::runtime_error("Error while patching cmd for usb2 mode");

            #else

                auto depthai_usb2_binary = fs.open(cmrc_depthai_usb2_cmd_path);
                finalCmd = std::vector<std::uint8_t>(depthai_usb2_binary.begin(), depthai_usb2_binary.end());

            #endif

        } else {

            auto depthai_binary = fs.open(cmrc_depthai_cmd_path);
            finalCmd = std::vector<std::uint8_t>(depthai_binary.begin(), depthai_binary.end());

        }


    #else
    // Binaries from default path (TODO)

    #endif

    return finalCmd;

}


std::vector<std::string> Device::get_available_streams()
{
    std::vector<std::string> result;

    if (g_config_d2h.is_object() &&
        g_config_d2h.contains("_available_streams") &&
        g_config_d2h.at("_available_streams").is_array()
        )
    {
        for (const auto &obj : g_config_d2h.at("_available_streams"))
        {
            result.push_back(obj.get<std::string>());
        }
    }

    return result;
}

/*

std::shared_ptr<CNNHostPipeline> Device::create_pipeline(
    const std::string &config_json_str
)
{
    using json = nlohmann::json;

    config_backup = config_json_str;

    bool init_ok = false;
    do
    {
        

        // str -> json
        json config_json;
        if (!getJSONFromString(config_json_str, config_json))
        {
            std::cout << "Error: Cant parse json config :" << config_json_str << "\n";
            break;
        }

        // json -> configurations
        HostPipelineConfig config;
        if (!config.initWithJSON(config_json))
        {
            std::cout << "Error: Cant init configs with json: " << config_json.dump() << "\n";
            break;
        }

        // read tensor info
        std::vector<TensorInfo>       tensors_info;
        if (parseTensorInfosFromJsonFile(config.ai.blob_file_config, tensors_info))
        {
            std::cout << "CNN configurations read: " << config.ai.blob_file_config.c_str() << "\n";
        }
        else
        {
            std::cout << "There is no cnn configuration file or error in it\'s parsing: " << config.ai.blob_file_config.c_str() << "\n";
        }


        // pipeline configurations json
        // homography
        std::vector<float> homography_buff = {
            // default for BW0250TG:
             9.8806816e-01,  2.9474013e-03,  5.0676174e+00,
            -8.7650679e-03,  9.9214733e-01, -8.7952757e+00,
            -8.4495878e-06, -3.6034894e-06,  1.0000000e+00
        };
        bool stereo_center_crop = false;

        if (config.depth.calibration_file.empty())
        {
            std::cout << "depthai: Calibration file is not specified, will use default setting;\n";
        }
        else
        {
            HostDataReader calibration_reader;
            if (!calibration_reader.init(config.depth.calibration_file))
            {
                std::cout << "depthai: Error opening calibration file: " << config.depth.calibration_file << "\n";
                break;
            }

            const int homography_size = sizeof(float) * 9;
            int sz = calibration_reader.getSize();
            assert(sz >= homography_size);
            calibration_reader.readData(reinterpret_cast<unsigned char*>(homography_buff.data()), homography_size);
            int flags_size = sz - homography_size;
            if (flags_size > 0)
            {
                assert(flags_size == 1);
                calibration_reader.readData(reinterpret_cast<unsigned char*>(&stereo_center_crop), 1);
            }
        }

        json json_config_obj;

        // Add video configuration if specified
        if(config_json.count("video_config") > 0){
            json_config_obj["video_config"] = config_json["video_config"]; 
        }

        json_config_obj["board"]["clear-eeprom"] = config.board_config.clear_eeprom;
        json_config_obj["board"]["store-to-eeprom"] = config.board_config.store_to_eeprom;
        json_config_obj["board"]["override-eeprom"] = config.board_config.override_eeprom;
        json_config_obj["board"]["swap-left-and-right-cameras"] = config.board_config.swap_left_and_right_cameras;
        json_config_obj["board"]["left_fov_deg"] = config.board_config.left_fov_deg;
        json_config_obj["board"]["rgb_fov_deg"] = config.board_config.rgb_fov_deg;
        json_config_obj["board"]["left_to_right_distance_m"] = config.board_config.left_to_right_distance_m;
        json_config_obj["board"]["left_to_rgb_distance_m"] = config.board_config.left_to_rgb_distance_m;
        json_config_obj["board"]["stereo_center_crop"] = config.board_config.stereo_center_crop || stereo_center_crop;
        json_config_obj["board"]["name"] = config.board_config.name;
        json_config_obj["board"]["revision"] = config.board_config.revision;
        json_config_obj["_board"] =
        {
            {"_homography_right_to_left", homography_buff}
        };
        json_config_obj["depth"]["padding_factor"] = config.depth.padding_factor;
        json_config_obj["depth"]["depth_limit_mm"] = (int)(config.depth.depth_limit_m * 1000);
        json_config_obj["depth"]["confidence_threshold"] = config.depth.confidence_threshold;

        json_config_obj["_load_inBlob"] = true;
        json_config_obj["_pipeline"] =
        {
            {"_streams", json::array()}
        };

        json_config_obj["ai"]["calc_dist_to_bb"] = config.ai.calc_dist_to_bb;
        json_config_obj["ai"]["keep_aspect_ratio"] = config.ai.keep_aspect_ratio;
        json_config_obj["ai"]["camera_input"] = config.ai.camera_input;

        json_config_obj["ot"]["max_tracklets"] = config.ot.max_tracklets;
        json_config_obj["ot"]["confidence_threshold"] = config.ot.confidence_threshold;

        bool add_disparity_post_processing_color = false;
        bool temp_measurement = false;

        std::vector<std::string> pipeline_device_streams;

        for (const auto &stream : config.streams)
        {
            if (stream.name == "depth_color_h")
            {
                add_disparity_post_processing_color = true;
                json obj = { {"name", "disparity"} };
                if (0.f != stream.max_fps)     { obj["max_fps"]   = stream.max_fps;   };
                json_config_obj["_pipeline"]["_streams"].push_back(obj);
            }
            else
            {
                if (stream.name == "meta_d2h")
                {
                    temp_measurement = true;
                }
                json obj = { {"name" ,stream.name} };

                if (!stream.data_type.empty()) { obj["data_type"] = stream.data_type; };
                if (0.f != stream.max_fps)     { obj["max_fps"]   = stream.max_fps;   };

                // TODO: temporary solution
                if (stream.name == "depth_sipp")
                        // {
                        //     obj["data_type"] = "uint8";
                        //     c_streams_myriad_to_pc["depth_sipp"] = StreamInfo("depth_sipp",     0, { 720, 1280}  );
                        // }
                        {
                            obj["data_type"] = "uint16";
                            c_streams_myriad_to_pc["depth_sipp"] = StreamInfo("depth_sipp",     0, { 720, 1280}, 2  );
                        }
                        // {
                        //     obj["data_type"] = "rgb";
                        //     c_streams_myriad_to_pc["depth_sipp"] = StreamInfo("depth_sipp",     2764800, { 720, 1280, 3} );
                        // }

                json_config_obj["_pipeline"]["_streams"].push_back(obj);
                pipeline_device_streams.push_back(stream.name);
            }
        }


        // host -> "config_h2d" -> device
        std::string pipeline_config_str_packed = json_config_obj.dump();
        std::cout << "config_h2d json:\n" << pipeline_config_str_packed << "\n";
        // resize, as xlink expects exact;y the same size for input:
        assert(pipeline_config_str_packed.size() < g_streams_pc_to_myriad.at("config_h2d").size);
        pipeline_config_str_packed.resize(g_streams_pc_to_myriad.at("config_h2d").size, 0);

        if (!g_xlink->openWriteAndCloseStream(
                g_streams_pc_to_myriad.at("config_h2d"),
                pipeline_config_str_packed.data())
            )
        {
            std::cout << "depthai: pipelineConfig write error;\n";
            break;
        }

        // host -> "host_capture" -> device
        auto stream = g_streams_pc_to_myriad.at("host_capture");
        g_host_capture_command = std::unique_ptr<HostCaptureCommand>(new HostCaptureCommand((stream)));
        g_xlink->observe(*g_host_capture_command, stream);


        // read & pass blob file
        if (config.ai.blob_file.empty())
        {
            std::cout << "depthai: Blob file is not specified, will use default setting;\n";
        }
        else
        {
            HostDataReader _blob_reader;
            if (!_blob_reader.init(config.ai.blob_file))
            {
                std::cout << "depthai: Error opening blob file: " << config.ai.blob_file << "\n";
                break;
            }
            int size_blob = _blob_reader.getSize();

            std::vector<uint8_t> buff_blob(size_blob);

            std::cout << "Read: " << _blob_reader.readData(buff_blob.data(), size_blob) << std::endl;

            // inBlob
            StreamInfo blobInfo;
            blobInfo.name = "inBlob";
            blobInfo.size = size_blob;

            if (!g_xlink->openWriteAndCloseStream(blobInfo, buff_blob.data()))
            {
                std::cout << "depthai: pipelineConfig write error;\n";
                break;
            }
            printf("depthai: done sending Blob file %s\n", config.ai.blob_file.c_str());

            // outBlob
            StreamInfo outBlob;
            outBlob.name = "outBlob";
            //TODO: remove asserts considering StreamInfo size
            outBlob.size = 1;

            cnn_info cnn_input_info;

            static char cnn_info_arr[sizeof(cnn_info)];
            g_xlink->openReadAndCloseStream(
                outBlob,
                (void*)cnn_info_arr,
                sizeof(cnn_info)
                );

            memcpy(&cnn_input_info, &cnn_info_arr, sizeof(cnn_input_info));

            printf("CNN input width: %d\n", cnn_input_info.cnn_input_width);
            printf("CNN input height: %d\n", cnn_input_info.cnn_input_height);
            printf("CNN input num channels: %d\n", cnn_input_info.cnn_input_num_channels);
            printf("CNN to depth bounding-box mapping: start(%d, %d), max_size(%d, %d)\n",
                    cnn_input_info.nn_to_depth.offset_x,
                    cnn_input_info.nn_to_depth.offset_y,
                    cnn_input_info.nn_to_depth.max_width,
                    cnn_input_info.nn_to_depth.max_height);
            nn_to_depth_mapping["off_x"] = cnn_input_info.nn_to_depth.offset_x;
            nn_to_depth_mapping["off_y"] = cnn_input_info.nn_to_depth.offset_y;
            nn_to_depth_mapping["max_w"] = cnn_input_info.nn_to_depth.max_width;
            nn_to_depth_mapping["max_h"] = cnn_input_info.nn_to_depth.max_height;

            // update tensor infos
            assert(!(tensors_info.size() > (sizeof(cnn_input_info.offsets)/sizeof(cnn_input_info.offsets[0]))));

            for (int i = 0; i < tensors_info.size(); i++)
            {
                tensors_info[i].nnet_input_width  = cnn_input_info.cnn_input_width;
                tensors_info[i].nnet_input_height = cnn_input_info.cnn_input_height;
                tensors_info[i].offset = cnn_input_info.offsets[i];
            }

            c_streams_myriad_to_pc["previewout"].dimensions = {
                                                               cnn_input_info.cnn_input_num_channels,
                                                               cnn_input_info.cnn_input_height,
                                                               cnn_input_info.cnn_input_width
                                                               };

            // check CMX slices & used shaves
            int device_cmx_for_nnet = g_config_d2h.at("_resources").at("cmx").at("for_nnet").get<int>();
            if (cnn_input_info.number_of_cmx_slices != device_cmx_for_nnet)
            {
                std::cout << "Error: Blob is compiled for " << cnn_input_info.number_of_cmx_slices
                          << " cmx slices but device can calculate on " << device_cmx_for_nnet << "\n";
                break;
            }

            int device_shaves_for_nnet = g_config_d2h.at("_resources").at("shaves").at("for_nnet").get<int>();
            if (cnn_input_info.number_of_shaves != device_shaves_for_nnet)
            {
                std::cout << "Error: Blob is compiled for " << cnn_input_info.number_of_shaves
                          << " shaves but device can calculate on " << device_shaves_for_nnet << "\n";
                break;
            }
        }


        // sort streams by device specified order
        {
            // mapping: stream name -> array index
            std::vector<std::string> available_streams_ordered = get_available_streams();
            std::unordered_map<std::string, int> stream_name_to_idx;
            for (int i = 0; i < available_streams_ordered.size(); ++i)
            {
                stream_name_to_idx[ available_streams_ordered[i] ] = i;
            }

            // check requested streams are in available streams
            bool wrong_stream_name = false;
            for (const auto &stream_name : pipeline_device_streams)
            {
                if (stream_name_to_idx.find(stream_name) == stream_name_to_idx.end())
                {
                    std::cout << "Error: device does not provide stream: " << stream_name << "\n";
                    wrong_stream_name = true;
                }
            }

            if (wrong_stream_name)
            {
                break;
            }

            // sort
            std::sort(std::begin(pipeline_device_streams), std::end(pipeline_device_streams),
                [&stream_name_to_idx]
                (const std::string &a, const std::string &b)
                {
                    return stream_name_to_idx[a] < stream_name_to_idx[b];
                }
            );
        }


        // pipeline
        if(gl_result == nullptr)
            gl_result = std::shared_ptr<CNNHostPipeline>(new CNNHostPipeline(tensors_info));

        for (const std::string &stream_name : pipeline_device_streams)
        {
            std::cout << "Host stream start:" << stream_name << "\n";

            if (g_xlink->openStreamInThreadAndNotifyObservers(c_streams_myriad_to_pc.at(stream_name)))
            {
                gl_result->makeStreamPublic(stream_name);
                gl_result->observe(*g_xlink.get(), c_streams_myriad_to_pc.at(stream_name));
            }
            else
            {
                std::cout << "depthai: " << stream_name << " error;\n";
                // TODO: rollback correctly!
                break;
            }
        }


        // disparity post processor
        if (add_disparity_post_processing_color)
        {
            g_disparity_post_proc = std::unique_ptr<DisparityStreamPostProcessor>(
                new DisparityStreamPostProcessor(
                    add_disparity_post_processing_color));

            const std::string stream_in_name = "disparity";
            const std::string stream_out_color_name = "depth_color_h";

            if (g_xlink->openStreamInThreadAndNotifyObservers(c_streams_myriad_to_pc.at(stream_in_name)))
            {
                g_disparity_post_proc->observe(*g_xlink.get(), c_streams_myriad_to_pc.at(stream_in_name));

                if (add_disparity_post_processing_color)
                {
                    gl_result->makeStreamPublic(stream_out_color_name);
                    gl_result->observe(*g_disparity_post_proc.get(), c_streams_myriad_to_pc.at(stream_out_color_name));
                }
            }
            else
            {
                std::cout << "depthai: stream open error " << stream_in_name << " (2)\n";
                // TODO: rollback correctly!
                break;
            }
        }

        if(temp_measurement)
        {
            // device support listener
            g_device_support_listener = std::unique_ptr<DeviceSupportListener>(new DeviceSupportListener);

            g_device_support_listener->observe(
                *g_xlink.get(),
                c_streams_myriad_to_pc.at("meta_d2h")
                );
        }

        init_ok = true;
        std::cout << "depthai: INIT OK!\n";
    }
    while (false);

    if (!init_ok)
    {
        gl_result = nullptr;
    }

    return gl_result;
}


*/


void Device::request_jpeg(){
if(g_host_capture_command != nullptr){
        g_host_capture_command->capture();
    }
}

void Device::request_af_trigger(){
    if(g_host_capture_command != nullptr){
        g_host_capture_command->afTrigger();
    }
}

void Device::request_af_mode(CaptureMetadata::AutofocusMode mode){
    if(g_host_capture_command != nullptr){
        g_host_capture_command->afMode(mode);
    }
}

std::map<std::string, int> Device::get_nn_to_depth_bbox_mapping(){
    return nn_to_depth_mapping;
}





bool Device::startTestPipeline(){

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
    nlohmann::json pipelineDescJson = R"(
    {
        "globalProperties": {
            "leonOsFrequencyKhz": 600000,
            "pipelineVersion": "1",
            "pipelineName": "1",
            "leonRtFrequencyKhz": 600000
        },
        "nodes": [
            {
                "id": 1,
                "name": "MyProducer",
                "properties": {
                    "message": "HeiHoi",
                    "processorPlacement": "LRT"
                }
            },
            {
                "id": 2,
                "name": "MyConsumer",
                "properties": {
                    "processorPlacement": "LRT"
                }
            },
            {
                "id": 3,
                "name": "MyConsumer",
                "properties": {
                    "processorPlacement": "LOS"
                }
            },
            {
                "id": 4,
                "name": "MyConsumer",
                "properties": {
                    "processorPlacement": "LRT"
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

    std::vector<std::uint8_t> assetStorage;
    Assets assets;
    auto pipelineDescription = nlohmann::json::to_msgpack(pipelineDescJson);


    // Load pipelineDesc, assets, and asset storage

    client->call("parsePipeline", pipelineDescription); 

    // Transfer storage if size > 0
    if(assetStorage.size() > 0){
        client->call("parseAssets", assets);


        // allocate, returns a pointer to memory on device side
        auto memHandle = client->call("memAlloc", (std::uint32_t) assetStorage.size()).as<uint32_t>(); 

        // Transfer the whole assetStorage in a separate thread
        const std::string streamAssetStorage = "__stream_asset_storage";
        std::thread t1([this, &streamAssetStorage, &assetStorage](){
            connection->openStream(streamAssetStorage, XLINK_USB_BUFFER_MAX_SIZE);
            int64_t offset = 0;
            do{
                int64_t toTransfer = std::min( (int64_t) XLINK_USB_BUFFER_MAX_SIZE, (int64_t) assetStorage.size() - offset);
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
    //client->call("test");


    // Build and start the pipeline
    bool success;
    std::string errorMsg;
    std::tie(success, errorMsg) = client->call("buildPipeline").as<std::tuple<bool, std::string>>();
    if(success){
        client->call("startPipeline");
        return true;
    } else {
        throw std::runtime_error(errorMsg);  
        return false;
    }

    client->call("startCamera");

}






} // namespace dai
