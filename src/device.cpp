#include "device.hpp"

// shared
#include "depthai-shared/json_helper.hpp"
#include "depthai-shared/depthai_constants.hpp"
#include "depthai-shared/cnn_info.hpp"

// project
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

#define WARNING "\033[1;5;31m"
#define ENDC "\033[0m"

constexpr static auto cmrc_depthai_cmd_path = "depthai.cmd";
constexpr static auto cmrc_depthai_usb2_cmd_path = "depthai-usb2.cmd";
constexpr static auto cmrc_depthai_usb2_patch_path = "depthai-usb2-patch.patch";

// GLOBAL
static XLinkGlobalHandler_t g_xlink_global_handler = {};

Device::Device(std::string usb_device, bool usb2_mode){
    
    // Binaries are resource compiled
    #ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

        // Get binaries from internal sources
        auto fs = cmrc::depthai::get_filesystem();

        if(usb2_mode){

            #ifdef DEPTHAI_PATCH_ONLY_MODE
            
                // Get size of original
                auto depthai_binary = fs.open(cmrc_depthai_cmd_path);

                // Open patch
                auto depthai_usb2_patch = fs.open(cmrc_depthai_usb2_patch_path);

                // Get new size
                int64_t patched_size = bspatch_mem_get_newsize( (uint8_t*) depthai_usb2_patch.begin(), depthai_usb2_patch.size());

                // Reserve space for patched binary
                patched_cmd.resize(patched_size);

                // Patch
                int error = bspatch_mem( (uint8_t*) depthai_binary.begin(), depthai_binary.size(), (uint8_t*) depthai_usb2_patch.begin(), depthai_usb2_patch.size(), patched_cmd.data());

                // if patch successful
                if(!error){
                    // Boot
                    init_device("", usb_device, patched_cmd.data(), patched_size);
                } else {
                    std::cout << "Error while patching..." << std::endl;
                    // TODO handle error (throw most likely)
                }

            #else
                auto depthai_usb2_binary = fs.open(cmrc_depthai_usb2_cmd_path);
                uint8_t* binary = (uint8_t*) depthai_usb2_binary.begin();
                long size = depthai_usb2_binary.size();
                init_device("", usb_device, binary, size);

            #endif

        } else {

            auto depthai_binary = fs.open(cmrc_depthai_cmd_path);
            uint8_t* binary = (uint8_t*) depthai_binary.begin();
            long size = depthai_binary.size();
            init_device("", usb_device, binary, size);

        }


    #else
    // Binaries from default path (TODO)

    #endif


}

Device::Device() : Device("", false){

}

Device::Device(std::string cmd_file, std::string usb_device){
        
    if(!init_device(cmd_file, usb_device)){
        throw std::runtime_error("Cannot initialize device");
    }
}

Device::~Device(){
    deinit_device();
}



void Device::wdog_thread(std::chrono::milliseconds& wd_timeout)
{
    std::cout << "watchdog started " << std::endl;
    const std::chrono::milliseconds poll_rate(100);
    const auto sleep_nr = wd_timeout / poll_rate;
    while(wdog_thread_alive)
    {
        wdog_keep = 0;
        for(int i = 0; i < sleep_nr; i++)
        {
            std::this_thread::sleep_for(poll_rate);
            if(wdog_thread_alive == 0)
            {
                break;
            }
        }
        if(wdog_keep == 0 && wdog_thread_alive == 1)
        {
            std::cout << "watchdog triggered " << std::endl;
            soft_deinit_device();
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
        wd_thread = std::thread(&Device::wdog_thread, this, std::ref(wd_timeout)); 
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

void Device::wdog_keepalive(void)
{
    wdog_keep = 1;
}

bool Device::init_device(
    const std::string &device_cmd_file,
    const std::string &usb_device,
    uint8_t* binary,
    long binary_size
)
{
    cmd_backup = device_cmd_file;
    usb_device_backup = usb_device;
    binary_backup = binary;
    binary_size_backup = binary_size;
    bool result = false;
    std::string error_msg;


    do
    {
        // xlink
        if (nullptr != g_xlink)
        {
            error_msg = "Device is already initialized.";
            std::cout << error_msg << "\n";
            break;
        }

        g_xlink = std::unique_ptr<XLinkWrapper>(new XLinkWrapper(true));

        
        if(binary != nullptr && binary_size != 0){
            if (!g_xlink->initFromHostSide(
                &g_xlink_global_handler,
                &g_xlink_device_handler,
                binary, binary_size,
                usb_device,
                true)
            )
            {
                std::cout << "depthai: Error initializing xlink\n";
                break;
            }
        } else {
            if (!g_xlink->initFromHostSide(
                &g_xlink_global_handler,
                &g_xlink_device_handler,
                device_cmd_file,
                usb_device,
                true)
            )
            {
                std::cout << "depthai: Error initializing xlink\n";
                break;
            }
        }

        g_xlink->setWatchdogUpdateFunction(std::bind(&Device::wdog_keepalive, this));
        wdog_start();

        // config_d2h
        {
            printf("Loading config file\n");

            std::string config_d2h_str;
            StreamInfo si("config_d2h", 102400);

            int config_file_length = g_xlink->openReadAndCloseStream(
                    si,
                    config_d2h_str
                    );
            if(config_file_length == -1)
            {
                break;
            }
            if (!getJSONFromString(config_d2h_str, g_config_d2h))
            {
                std::cout << "depthai: error parsing config_d2h\n";
            }
        }

        bool rgb_connected = g_config_d2h.at("_cams").at("rgb").get<bool>();
        bool left_connected = g_config_d2h.at("_cams").at("left").get<bool>();
        bool right_connected = g_config_d2h.at("_cams").at("right").get<bool>();
        if(!rgb_connected && (left_connected ^ right_connected))
        {
            std::cerr << WARNING "FATAL ERROR: No cameras detected on the board. \n" ENDC;
            break;
        }

        // check version
        {

            /* TODO: review in new refactored way
            std::string device_version = g_config_d2h.at("_version").get<std::string>();
            if (device_version != c_depthai_version)
            {
                printf("Version does not match (%s & %s)\n",
                    device_version.c_str(), c_depthai_version);
                break;
            }

            std::string device_dev_version = g_config_d2h.at("_dev_version").get<std::string>();
            if (device_dev_version != c_depthai_dev_version)
            {
                printf("WARNING: Version (dev) does not match (%s & %s)\n",
                    device_dev_version.c_str(), c_depthai_dev_version);
            }
            */
        }

        uint32_t version = g_config_d2h.at("eeprom").at("version").get<int>();
        printf("EEPROM data:");
        if (version == -1) {
            printf(" invalid / unprogrammed\n");
        } else {
            printf(" valid (v%d)\n", version);
            std::string board_name;
            std::string board_rev;
            float rgb_fov_deg = 0;
            bool stereo_center_crop = false;
            if (version >= 2) {
                board_name = g_config_d2h.at("eeprom").at("board_name").get<std::string>();
                board_rev  = g_config_d2h.at("eeprom").at("board_rev").get<std::string>();
                rgb_fov_deg= g_config_d2h.at("eeprom").at("rgb_fov_deg").get<float>();
            }
            if (version >= 3) {
                stereo_center_crop = g_config_d2h.at("eeprom").at("stereo_center_crop").get<bool>();
            }
            float left_fov_deg = g_config_d2h.at("eeprom").at("left_fov_deg").get<float>();
            float left_to_right_distance_m = g_config_d2h.at("eeprom").at("left_to_right_distance_m").get<float>();
            float left_to_rgb_distance_m = g_config_d2h.at("eeprom").at("left_to_rgb_distance_m").get<float>();
            bool swap_left_and_right_cameras = g_config_d2h.at("eeprom").at("swap_left_and_right_cameras").get<bool>();
            std::vector<float> calib = g_config_d2h.at("eeprom").at("calib").get<std::vector<float>>();
            printf("  Board name     : %s\n", board_name.empty() ? "<NOT-SET>" : board_name.c_str());
            printf("  Board rev      : %s\n", board_rev.empty()  ? "<NOT-SET>" : board_rev.c_str());
            printf("  HFOV L/R       : %g deg\n", left_fov_deg);
            printf("  HFOV RGB       : %g deg\n", rgb_fov_deg);
            printf("  L-R   distance : %g cm\n", 100 * left_to_right_distance_m);
            printf("  L-RGB distance : %g cm\n", 100 * left_to_rgb_distance_m);
            printf("  L/R swapped    : %s\n", swap_left_and_right_cameras ? "yes" : "no");
            printf("  L/R crop region: %s\n", stereo_center_crop ? "center" : "top");
            printf("  Calibration homography:\n");
            for (int i = 0; i < 9; i++) {
                printf(" %11.6f,", calib.at(i));
                if (i % 3 == 2) printf("\n");
            }
        }


        result = true;
    } while (false);

    if (!result)
    {
        g_xlink = nullptr;
        // TODO: add custom python exception for passing messages
        // throw std::exception();
    }

    return result;
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


std::shared_ptr<CNNHostPipeline> Device::create_pipeline(
    const std::string &config_json_str
)
{
    using json = nlohmann::json;

    config_backup = config_json_str;

    bool init_ok = false;
    do
    {
        // check xlink
        if (nullptr == g_xlink)
        {
            std::cerr << WARNING "device is not initialized\n" ENDC;
            break;
        }

        // str -> json
        json config_json;
        if (!getJSONFromString(config_json_str, config_json))
        {
            std::cerr << WARNING "Error: Cant parse json config :" << config_json_str << "\n" ENDC;
            break;
        }

        // json -> configurations
        HostPipelineConfig config;
        if (!config.initWithJSON(config_json))
        {
            std::cerr << "Error: Cant init configs with json: " << config_json.dump() << "\n";
            break;
        }

        int num_stages = config.ai.blob_file2.empty() ? 1 : 2;

        // read tensor info
        std::vector<TensorInfo>       tensors_info;
        if (parseTensorInfosFromJsonFile(config.ai.blob_file_config, tensors_info))
        {
            std::cout << "CNN configurations read: " << config.ai.blob_file_config.c_str() << "\n";
        }
        else
        {
            std::cerr << WARNING "ERROR: There is no cnn configuration file or error in it\'s parsing: " << config.ai.blob_file_config.c_str() << "\n";
            break;
        }

        if (num_stages > 1)
        {
            if (parseTensorInfosFromJsonFile(config.ai.blob_file_config2, tensors_info))
            {
                std::cout << "CNN configurations read: " << config.ai.blob_file_config2.c_str() << "\n";
            }
            else
            {
                std::cout << "There is no cnn configuration file or error in it\'s parsing: " << config.ai.blob_file_config2.c_str() << "\n";
            }
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
                std::cerr << WARNING "depthai: Error opening calibration file: " << config.depth.calibration_file << "\n" ENDC;
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

        bool rgb_connected = g_config_d2h.at("_cams").at("rgb").get<bool>();
        bool left_connected = g_config_d2h.at("_cams").at("left").get<bool>();
        bool right_connected = g_config_d2h.at("_cams").at("right").get<bool>();
        if(config.board_config.swap_left_and_right_cameras)
        {
            bool temp = left_connected;
            left_connected = right_connected;
            right_connected = temp;
        }


        if(!rgb_connected)
        {
            std::cout << "RGB camera (IMX378) is not detected on board! \n";
            if(config.ai.camera_input == "rgb")
            {
                std::cerr << WARNING "WARNING: NN inference was requested on RGB camera (IMX378), defaulting to right stereo camera (OV9282)! \n" ENDC;
                config.ai.camera_input = "right";
            }
        }
        if(left_connected ^ right_connected)
        {
            std::string cam_not_connected = (left_connected == false) ? "Left" : "Right";
            std::cerr << WARNING "WARNING: "<< cam_not_connected << " stereo camera (OV9282) is not detected on board! \n" ENDC;
            if(config.ai.camera_input != "rgb")
            {
                std::cerr << WARNING "WARNING: NN inference was requested on " << config.ai.camera_input << " stereo camera (OV9282), defaulting to RGB camera (IMX378)! \n" ENDC;
                config.ai.camera_input = "rgb";
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

        json_config_obj["camera"]["rgb"]["resolution_w"]  = config.rgb_cam_config.resolution_w;
        json_config_obj["camera"]["rgb"]["resolution_h"]  = config.rgb_cam_config.resolution_h;
        json_config_obj["camera"]["rgb"]["fps"]           = config.rgb_cam_config.fps;
        json_config_obj["camera"]["mono"]["resolution_w"] = config.mono_cam_config.resolution_w;
        json_config_obj["camera"]["mono"]["resolution_h"] = config.mono_cam_config.resolution_h;
        json_config_obj["camera"]["mono"]["fps"]          = config.mono_cam_config.fps;

        std::string blob_file[] = {config.ai.blob_file, config.ai.blob_file2};

        std::vector <HostDataReader> _blob_reader(num_stages);
        std::vector <int> size_blob(num_stages);
        for (int stage = 0; stage < num_stages; stage++)
        {
            if (!blob_file[stage].empty())
            {
                if (!_blob_reader[stage].init(blob_file[stage]))
                {
                    std::cerr << WARNING "depthai: Error opening blob file: " << blob_file[stage] << "\n" ENDC;
                    break;
                }
                size_blob[stage] = _blob_reader[stage].getSize();
            }
        }

        json_config_obj["ai"]["blob0_size"] = size_blob[0];
        json_config_obj["ai"]["blob1_size"] = (num_stages > 1) ? size_blob[1] : 0;
        json_config_obj["ai"]["calc_dist_to_bb"] = config.ai.calc_dist_to_bb;
        json_config_obj["ai"]["keep_aspect_ratio"] = config.ai.keep_aspect_ratio;
        json_config_obj["ai"]["shaves"] = config.ai.shaves;
        json_config_obj["ai"]["cmx_slices"] = config.ai.cmx_slices;
        json_config_obj["ai"]["NCEs"] = config.ai.NN_engines;
        json_config_obj["ai"]["camera_input"] = config.ai.camera_input;
        json_config_obj["ai"]["num_stages"] = num_stages;

        json_config_obj["ot"]["max_tracklets"] = config.ot.max_tracklets;
        json_config_obj["ot"]["confidence_threshold"] = config.ot.confidence_threshold;

        json_config_obj["app"]["sync_video_meta_streams"] = config.app_config.sync_video_meta_streams;

        bool add_disparity_post_processing_color = false;
        bool temp_measurement = false;

        std::vector<std::string> pipeline_device_streams;

        for (const auto &stream : config.streams)
        {
            if (c_streams_myriad_to_pc[stream.name].dimensions[0] == MONO_RES_AUTO) {
                c_streams_myriad_to_pc[stream.name].dimensions[0] = config.mono_cam_config.resolution_h;
                c_streams_myriad_to_pc[stream.name].dimensions[1] = config.mono_cam_config.resolution_w;
            }

            if (stream.name == "disparity_color")
            {
                c_streams_myriad_to_pc["disparity"].dimensions[0] = c_streams_myriad_to_pc[stream.name].dimensions[0];
                c_streams_myriad_to_pc["disparity"].dimensions[1] = c_streams_myriad_to_pc[stream.name].dimensions[1];
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

                if (stream.name == "depth_raw"){obj["data_type"] = "uint16"; }

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
            std::cerr << WARNING "depthai: pipelineConfig write error\n" ENDC;
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
            for (int stage = 0; stage < num_stages; stage++)
            {
                std::vector<uint8_t> buff_blob(size_blob[stage]);

                std::cout << "Read: " << _blob_reader[stage].readData(buff_blob.data(), size_blob[stage]) << std::endl;

                // inBlob
                StreamInfo blobInfo;
                blobInfo.name = "inBlob";
                blobInfo.size = size_blob[stage];

                if (!g_xlink->openWriteAndCloseStream(blobInfo, buff_blob.data()))
                {
                    std::cout << "depthai: pipelineConfig write error: Blob size too big: " << size_blob[stage] << "\n";
                    break;
                }
                printf("depthai: done sending Blob file %s\n", blob_file[stage].c_str());

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
                if (stage == 0)
                {
                    printf("CNN to depth bounding-box mapping: start(%d, %d), max_size(%d, %d)\n",
                            cnn_input_info.nn_to_depth.offset_x,
                            cnn_input_info.nn_to_depth.offset_y,
                            cnn_input_info.nn_to_depth.max_width,
                            cnn_input_info.nn_to_depth.max_height);
                    nn_to_depth_mapping["off_x"] = cnn_input_info.nn_to_depth.offset_x;
                    nn_to_depth_mapping["off_y"] = cnn_input_info.nn_to_depth.offset_y;
                    nn_to_depth_mapping["max_w"] = cnn_input_info.nn_to_depth.max_width;
                    nn_to_depth_mapping["max_h"] = cnn_input_info.nn_to_depth.max_height;
                }
                // update tensor infos
                assert(!(tensors_info.size() > (sizeof(cnn_input_info.offsets)/sizeof(cnn_input_info.offsets[0]))));

                if (stage == 0) {
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
                }
                // check CMX slices & used shaves
                if (cnn_input_info.number_of_cmx_slices > config.ai.cmx_slices)
                {
                    std::cerr << WARNING "Error: Blob is compiled for " << cnn_input_info.number_of_cmx_slices
                              << " cmx slices but device is configured to calculate on " << config.ai.cmx_slices << "\n" ENDC;
                    break;
                }

                if (cnn_input_info.number_of_shaves > config.ai.shaves)
                {
                    std::cerr << WARNING "Error: Blob is compiled for " << cnn_input_info.number_of_shaves
                              << " shaves but device is configured to calculate on " << config.ai.shaves << "\n" ENDC;
                    break;
                }

                if(!cnn_input_info.satisfied_resources)
                {
                    std::cerr << WARNING "ERROR: requested CNN resources overlaps with RGB camera \n" ENDC;
                    break;
                }

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
            const std::string stream_out_color_name = "disparity_color";

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

void Device::send_disparity_confidence_threshold(uint8_t confidence){
    if(g_host_capture_command != nullptr){
        g_host_capture_command->sendDisparityConfidenceThreshold(confidence);
    }
}

std::map<std::string, int> Device::get_nn_to_depth_bbox_mapping(){
    return nn_to_depth_mapping;
}
