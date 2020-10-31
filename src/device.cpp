#include "device.hpp"
#include "matrix_ops.hpp"
// shared
#include "depthai-shared/json_helper.hpp"
#include "depthai-shared/depthai_constants.hpp"
#include "depthai-shared/cnn_info.hpp"

// project
//#include "pipeline/host_pipeline_config.hpp"
#include "pipeline/host_pipeline_config.hpp"
#include "host_data_reader.hpp"

extern "C"
{
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

Device::Device(std::string usb_device, bool usb2_mode)
{

// Binaries are resource compiled
#ifdef DEPTHAI_RESOURCE_COMPILED_BINARIES

    // Get binaries from internal sources
    auto fs = cmrc::depthai::get_filesystem();

    if (usb2_mode)
    {

#ifdef DEPTHAI_PATCH_ONLY_MODE

        // Get size of original
        auto depthai_binary = fs.open(cmrc_depthai_cmd_path);

        // Open patch
        auto depthai_usb2_patch = fs.open(cmrc_depthai_usb2_patch_path);

        // Get new size
        int64_t patched_size = bspatch_mem_get_newsize(
            (uint8_t*)depthai_usb2_patch.begin(), depthai_usb2_patch.size());

        // Reserve space for patched binary
        patched_cmd.resize(patched_size);

        // Patch
        int error = bspatch_mem(
            (uint8_t*)depthai_binary.begin(), depthai_binary.size(),
            (uint8_t*)depthai_usb2_patch.begin(), depthai_usb2_patch.size(), patched_cmd.data());

        // if patch successful
        if (!error)
        {
            // Boot
            init_device("", usb_device, patched_cmd.data(), patched_size);
        }
        else
        {
            std::cout << "Error while patching..." << std::endl;
            // TODO handle error (throw most likely)
        }

#else
        auto depthai_usb2_binary = fs.open(cmrc_depthai_usb2_cmd_path);
        uint8_t* binary = (uint8_t*)depthai_usb2_binary.begin();
        long size = depthai_usb2_binary.size();
        init_device("", usb_device, binary, size);

#endif
    }
    else
    {

        auto depthai_binary = fs.open(cmrc_depthai_cmd_path);
        uint8_t* binary = (uint8_t*)depthai_binary.begin();
        long size = depthai_binary.size();
        init_device("", usb_device, binary, size);
    }

#else
    // Binaries from default path (TODO)

#endif
}

Device::Device() : Device("", false) {}

Device::Device(std::string cmd_file, std::string usb_device)
{

    if (!init_device(cmd_file, usb_device))
    {
        throw std::runtime_error("Cannot initialize device");
    }
}

Device::~Device() { deinit_device(); }

void Device::wdog_thread(std::chrono::milliseconds& wd_timeout)
{
    std::cout << "watchdog started " << std::endl;
    const std::chrono::milliseconds poll_rate(100);
    const auto sleep_nr = wd_timeout / poll_rate;
    while (wdog_thread_alive)
    {
        wdog_keep = 0;
        for (int i = 0; i < sleep_nr; i++)
        {
            std::this_thread::sleep_for(poll_rate);
            if (wdog_thread_alive == 0)
            {
                break;
            }
        }
        if (wdog_keep == 0 && wdog_thread_alive == 1)
        {
            std::cout << "watchdog triggered " << std::endl;
            soft_deinit_device();
            bool init;
            for (int retry = 0; retry < 1; retry++)
            {
                init =
                    init_device(cmd_backup, usb_device_backup, binary_backup, binary_size_backup);
                if (init)
                {
                    break;
                }
            }
            if (!init)
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
    if (once)
    {
        wdog_thread_alive = 1;
        wd_thread = std::thread(&Device::wdog_thread, this, std::ref(wd_timeout));
        once = 0;
    }
    return 0;
}
int Device::wdog_stop(void)
{
    if (wdog_thread_alive)
    {
        wdog_thread_alive = 0;
        wd_thread.join();
    }
    return 0;
}

void Device::wdog_keepalive(void) { wdog_keep = 1; }

bool Device::init_device(
    const std::string& device_cmd_file, const std::string& usb_device, uint8_t* binary,
    long binary_size)
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

        if (binary != nullptr && binary_size != 0)
        {
            if (!g_xlink->initFromHostSide(
                    &g_xlink_global_handler, &g_xlink_device_handler, binary, binary_size,
                    usb_device, true))
            {
                std::cout << "depthai: Error initializing xlink\n";
                break;
            }
        }
        else
        {
            if (!g_xlink->initFromHostSide(
                    &g_xlink_global_handler, &g_xlink_device_handler, device_cmd_file, usb_device,
                    true))
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

            int config_file_length = g_xlink->openReadAndCloseStream(si, config_d2h_str);
            if (config_file_length == -1)
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
        if (!rgb_connected && (left_connected ^ right_connected))
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

        version = g_config_d2h.at("eeprom").at("version").get<decltype(version)>();
        printf("EEPROM data:");
        H1_l.clear();
        H2_r.clear();
        R1_l.clear();
        R2_r.clear();
        M1_l.clear();
        M2_r.clear();
        R.clear();
        T.clear();
        d1_l.clear();
        d2_r.clear();
        std::vector<float> temp;

        if (version == -1)
        {
            printf(" invalid / unprogrammed\n");
        }
        else
        {
            printf(" valid (v%d)\n", version);
            std::string board_name;
            std::string board_rev;
            float rgb_fov_deg = 0;
            bool stereo_center_crop = false;
            if (version >= 2)
            {
                board_name = g_config_d2h.at("eeprom").at("board_name").get<std::string>();
                board_rev = g_config_d2h.at("eeprom").at("board_rev").get<std::string>();
                rgb_fov_deg = g_config_d2h.at("eeprom").at("rgb_fov_deg").get<float>();
            }
            if (version >= 3)
            {
                stereo_center_crop = g_config_d2h.at("eeprom").at("stereo_center_crop").get<bool>();
            }
            float left_fov_deg = g_config_d2h.at("eeprom").at("left_fov_deg").get<float>();
            float left_to_right_distance_m =
                g_config_d2h.at("eeprom").at("left_to_right_distance_m").get<float>();
            float left_to_rgb_distance_m =
                g_config_d2h.at("eeprom").at("left_to_rgb_distance_m").get<float>();
            bool swap_left_and_right_cameras =
                g_config_d2h.at("eeprom").at("swap_left_and_right_cameras").get<bool>();
            std::vector<float> calib;
            printf(
                "  Board name     : %s\n", board_name.empty() ? "<NOT-SET>" : board_name.c_str());
            printf("  Board rev      : %s\n", board_rev.empty() ? "<NOT-SET>" : board_rev.c_str());
            printf("  HFOV L/R       : %g deg\n", left_fov_deg);
            printf("  HFOV RGB       : %g deg\n", rgb_fov_deg);
            printf("  L-R   distance : %g cm\n", 100 * left_to_right_distance_m);
            printf("  L-RGB distance : %g cm\n", 100 * left_to_rgb_distance_m);
            printf("  L/R swapped    : %s\n", swap_left_and_right_cameras ? "yes" : "no");
            printf("  L/R crop region: %s\n", stereo_center_crop ? "center" : "top");
            // std::cout << "H2 matrix of right came----------------> " << H2_r.size() << std::endl;

            if (version <= 3)
            {
                printf("  Calibration homography right to left (legacy, please consider "
                       "recalibrating):\n");
                calib = g_config_d2h.at("eeprom").at("calib_old_H").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        H2_r.push_back(temp);
                        temp.clear();
                    }
                }
                // std::cout << "H2 matrix of right came----------------> " << H2_r.size() <<
                // std::endl;
            }
            else if (version == 4)
            {

                std::vector<std::vector<float>> temp_inv;

                printf("  Calibration inverse homography H1 (left):\n");
                calib = g_config_d2h.at("eeprom").at("calib_H1_L").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        temp_inv.push_back(temp);
                        temp.clear();
                    }
                }

                mat_inv(temp_inv, H1_l);
                temp_inv.clear();

                for (int i = 0; i < 9; ++i)
                {
                }
                printf("  Calibration inverse homography H2 (right):\n");
                calib = g_config_d2h.at("eeprom").at("calib_H2_R").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        temp_inv.push_back(temp);
                        temp.clear();
                    }
                }

                mat_inv(temp_inv, H2_r);
                temp_inv.clear();

                printf("  Calibration intrinsic matrix M1 (left):\n");
                calib = g_config_d2h.at("eeprom").at("calib_M1_L").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        M1_l.push_back(temp);
                        temp.clear();
                    }
                }

                printf("  Calibration intrinsic matrix M2 (right):\n");
                calib = g_config_d2h.at("eeprom").at("calib_M2_R").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        M2_r.push_back(temp);
                        temp.clear();
                    }
                }

                printf("  Calibration rotation matrix R:\n");
                calib = g_config_d2h.at("eeprom").at("calib_R").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        R.push_back(temp);
                        temp.clear();
                    }
                }

                printf("  Calibration translation matrix T:\n");
                calib = g_config_d2h.at("eeprom").at("calib_T").get<std::vector<float>>();
                for (int i = 0; i < 3; i++)
                {
                    printf(" %11.6f,\n", calib.at(i));
                }
                T = calib;

                std::cout << "H2 matrix of right came----------------> " << H2_r.size()
                          << std::endl;
            }
            else if (version == 5)
            {
                printf("  Rectification Rotation R1 (left):\n");
                calib = g_config_d2h.at("eeprom").at("calib_R1_L").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        R1_l.push_back(temp);
                        temp.clear();
                    }
                }
                for (int i = 0; i < 9; ++i)
                {
                }
                printf("  Rectification Rotation R2 (right):\n");
                calib = g_config_d2h.at("eeprom").at("calib_R2_R").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        R2_r.push_back(temp);
                        temp.clear();
                    }
                }

                printf("  Calibration intrinsic matrix M1 (left):\n");
                calib = g_config_d2h.at("eeprom").at("calib_M1_L").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        M1_l.push_back(temp);
                        temp.clear();
                    }
                }

                printf("  Calibration intrinsic matrix M2 (right):\n");
                calib = g_config_d2h.at("eeprom").at("calib_M2_R").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        M2_r.push_back(temp);
                        temp.clear();
                    }
                }

                printf("  Calibration rotation matrix R:\n");
                calib = g_config_d2h.at("eeprom").at("calib_R").get<std::vector<float>>();
                for (int i = 0; i < 9; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    temp.push_back(calib.at(i));
                    if (i % 3 == 2)
                    {
                        printf("\n");
                        R.push_back(temp);
                        temp.clear();
                    }
                }

                printf("  Calibration translation matrix T:\n");
                calib = g_config_d2h.at("eeprom").at("calib_T").get<std::vector<float>>();
                for (int i = 0; i < 3; i++)
                {
                    printf(" %11.6f,\n", calib.at(i));
                }
                T = calib;

                printf("  Calibration Distortion Coeff d1 (Left):\n");
                calib = g_config_d2h.at("eeprom").at("calib_d1_L").get<std::vector<float>>();
                for (int i = 0; i < 14; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    if (i % 7 == 6)
                        printf("\n");
                }
                d1_l = calib;

                printf("  Calibration Distortion Coeff d2 (Right):\n");
                calib = g_config_d2h.at("eeprom").at("calib_d2_R").get<std::vector<float>>();
                for (int i = 0; i < 14; i++)
                {
                    printf(" %11.6f,", calib.at(i));
                    if (i % 7 == 6)
                        printf("\n");
                }
                d2_r = calib;

                std::vector<std::vector<float>> M2_r_inv;
                std::vector<std::vector<float>> M1_l_inv;
                std::vector<std::vector<float>> left_matrix = mat_mul(M2_r, R2_r);

                mat_inv(M2_r, M2_r_inv);
                H2_r = mat_mul(left_matrix, M2_r_inv);

                left_matrix = mat_mul(M2_r, R1_l);
                mat_inv(M1_l, M1_l_inv);
                H1_l = mat_mul(left_matrix, M1_l_inv);
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

    if (g_config_d2h.is_object() && g_config_d2h.contains("_available_streams") &&
        g_config_d2h.at("_available_streams").is_array())
    {
        for (const auto& obj : g_config_d2h.at("_available_streams"))
        {
            result.push_back(obj.get<std::string>());
        }
    }

    return result;
}

std::vector<std::vector<float>> Device::get_left_intrinsic()
{
    if (version < 4)
    {
        std::cerr << "legacy, get_left_intrinsic() is not available in version " << version
                  << "\n recalibrate and load the new calibration to the device. \n";
        abort();
    }
    return M1_l;
}

std::vector<std::vector<float>> Device::get_left_homography()
{
    if (version < 4)
    {
        std::cerr << "legacy, get_left_homography() is not available in version " << version
                  << "\n recalibrate and load the new calibration to the device. \n";
        abort();
    }
    else
    {
        return H1_l;
    }
}

std::vector<std::vector<float>> Device::get_right_intrinsic()
{
    if (version < 4)
    {
        std::cerr << "legacy, get_right_intrinsic() is not available in version " << version
                  << "\n recalibrate and load the new calibration to the device. \n";
        abort();
    }
    return M2_r;
}

std::vector<std::vector<float>> Device::get_right_homography() { return H2_r; }

std::vector<std::vector<float>> Device::get_rotation()
{
    if (version < 4)
    {
        std::cerr << "legacy, get_rotation() is not available in version " << version
                  << "\n recalibrate and load the new calibration to the device. \n";
        abort();
    }
    return R;
}

std::vector<float> Device::get_translation()
{
    if (version < 4)
    {
        std::cerr << "legacy, get_Translation() is not available in version " << version
                  << "\n recalibrate and load the new calibration to the device. \n";
        abort();
    }
    return T;
}

std::shared_ptr<CNNHostPipeline> Device::create_pipeline(const std::string& config_json_str)
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
        std::vector<dai::TensorInfo>       tensors_info_output, tensors_info_input;
        std::vector<nlohmann::json>        NN_config;

        std::cout << config.ai.blob_file_config << std::endl;
        std::ifstream jsonFile(config.ai.blob_file_config);

        nlohmann::json json_NN_meta;
        nlohmann::json json_NN_;
        if (jsonFile.is_open()) {
            json_NN_ = nlohmann::json::parse(jsonFile);
        }

        if(!json_NN_.contains("NN_config"))
        {
            std::cout << "No NN config provided, defaulting to \"raw\" output format!" << std::endl;
            json_NN_meta["output_format"] = "raw";
        }
        else
        {
            json_NN_meta = json_NN_["NN_config"];
        }

        //stage 2 is always "raw"
        nlohmann::json json_NN_meta_stage2;
        json_NN_meta_stage2["output_format"] = "raw";

        nlohmann::json NN_config_stage[2] = {json_NN_meta, json_NN_meta_stage2};

        if(num_stages == 2)
        {
            if(NN_config_stage[0]["output_format"] != std::string("detection"))
            {
                throw std::runtime_error("In case of 2 stage inference the first stage network must have [\"NN_config\"][\"output_format\"] set to detection!");
            }
        }

        // pipeline configurations json
        // homography
        const int homography_count =
            9 * 7 + 3 * 2 + 14 * 3; /*R1,R2,M1,M2,R,T,M3,R_rgb,T_rgb,d1,d2,d3*/
        std::vector<float> calibration_buff(homography_count);
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
                std::cerr << WARNING "depthai: Error opening calibration file: "
                          << config.depth.calibration_file << "\n" ENDC;
                break;
            }

            const int homography_size = sizeof(float) * homography_count;
            int sz = calibration_reader.getSize();
            std::cout << homography_size << std::endl;
            std::cout << sz << std::endl;

            if (sz < homography_size)
            {
                if (version < 5 && config.board_config.store_to_eeprom)
                {
                    std::cerr << WARNING "Calibration file is outdated. Recalibration required "
                                         "before writing to EEPROM."
                              << std::endl
                              << "To continue using old calibration disable('-e') write to EEPROM.";
                    abort();
                }
                else
                {
                    std::cerr << WARNING "Calibration file size " << sz
                              << ENDC
                        " < smaller than expected, data ignored. May need to recalibrate\n";
                }
            }
            else
            {
                calibration_reader.readData(
                    reinterpret_cast<unsigned char*>(calibration_buff.data()), homography_size);
                int flags_size = sz - homography_size;
                if (flags_size > 0)
                {
                    assert(flags_size == 1);
                    calibration_reader.readData(
                        reinterpret_cast<unsigned char*>(&stereo_center_crop), 1);
                }
            }
        }

        if (version > 4)
        {
            if (config.mono_cam_config.resolution_h == 800)
            {
                // create_mesh()
            }
            else if (config.mono_cam_config.resolution_h == 720)
            {
                // adjusting y axis of the image center since it was cancluated for width of 800.

                M1_l[1][2] -= 40;
                M2_r[1][2] -= 40;
            }
            else if (config.mono_cam_config.resolution_h == 400)
            {
                /* adjusting intrinsic matrix by multiplying everything by multiplying all the
                 *intrinisc parameters by 0.5 except the right bottom corner which is a scale.
                 */

                M1_l[0][0] *= 0.5;
                M1_l[0][2] *= 0.5;
                M1_l[1][1] *= 0.5;
                M1_l[1][2] *= 0.5;

                M2_r[0][0] *= 0.5;
                M2_r[0][2] *= 0.5;
                M2_r[1][1] *= 0.5;
                M2_r[1][2] *= 0.5;
            }

            std::vector<std::vector<float>> M2_r_inv;
            std::vector<std::vector<float>> M1_l_inv;
            std::vector<std::vector<float>> left_matrix = mat_mul(M2_r, R2_r);

            mat_inv(M2_r, M2_r_inv);
            H2_r = mat_mul(left_matrix, M2_r_inv);

            left_matrix = mat_mul(M2_r, R1_l);
            mat_inv(M1_l, M1_l_inv);
            H1_l = mat_mul(left_matrix, M1_l_inv);
        }
        std::vector<float> left_mesh_buff(1, 0);
        std::vector<float> right_mesh_buff(1, 0);
        config.depth.warp.use_mesh = false;
        if (config.depth.warp.use_mesh)
        {

            const int map_size = 1280 * 800;
            std::vector<float> left_x_map_buff(map_size, 0);
            std::vector<float> right_x_map_buff(map_size, 0);
            std::vector<float> left_y_map_buff(map_size, 0);
            std::vector<float> right_y_map_buff(map_size, 0);

            std::cout << "left map file: " << config.depth.left_mesh_file << std::endl;
            std::cout << "right map file: " << config.depth.right_mesh_file << std::endl;

            if (config.depth.left_mesh_file.empty() && config.depth.right_mesh_file.empty())
            {
                std::cout << "depthai: mesh file is not specified, will use Homography;\n";
            }
            else if (config.depth.left_mesh_file.empty())
            {
                std::cout << "depthai: Only right camera mesh file is specified, Left camera mesh "
                             "file not specified;\n";
            }
            else if (config.depth.right_mesh_file.empty())
            {
                std::cout << "depthai: Only left camera mesh file is specified, Right camera mesh "
                             "file not specified;\n";
            }
            else
            {

                HostDataReader mesh_reader;
                const int expectec_mesh_size = sizeof(float) * map_size * 2;

                // Reading left mesh into the vector
                if (!mesh_reader.init(config.depth.left_mesh_file))
                {
                    std::cerr << WARNING "depthai: Error opening left camera mesh file: " ENDC
                              << config.depth.left_mesh_file << std::endl;
                    // break;
                }
                else
                {
                    int file_sz = mesh_reader.getSize();
                    assert(file_sz == expectec_mesh_size);
                    mesh_reader.readData(
                        reinterpret_cast<unsigned char*>(left_x_map_buff.data()),
                        expectec_mesh_size);
                    mesh_reader.readData(
                        reinterpret_cast<unsigned char*>(left_y_map_buff.data()),
                        expectec_mesh_size);
                    mesh_reader.closeFile();
                    int32_t res = config.mono_cam_config.resolution_h;
                    // create_mesh(left_x_map_buff, left_y_map_buff, left_mesh_buff, res);
                    // std::cout << "left mesh loaded with size :" << left_map_buff.size() << " File
                    // size: " << file_sz << " expectec_mesh_size ->" << expectec_mesh_size <<
                    // std::endl;
                }

                // Reading right mesh into the vector
                if (!mesh_reader.init(config.depth.right_mesh_file))
                {
                    std::cerr << WARNING "depthai: Error opening right camera mesh file: " ENDC
                              << config.depth.right_mesh_file << std::endl;
                    // break;
                }
                else
                {
                    int file_sz = mesh_reader.getSize();
                    assert(file_sz == expectec_mesh_size);
                    mesh_reader.readData(
                        reinterpret_cast<unsigned char*>(right_x_map_buff.data()),
                        expectec_mesh_size);
                    mesh_reader.readData(
                        reinterpret_cast<unsigned char*>(right_y_map_buff.data()),
                        expectec_mesh_size);
                    mesh_reader.closeFile();
                }
            }
        }
        // else {
        //     left_mesh_buff.resize(1);
        //     right_mesh_buff.resize(1);
        //     }

        bool rgb_connected = g_config_d2h.at("_cams").at("rgb").get<bool>();
        bool left_connected = g_config_d2h.at("_cams").at("left").get<bool>();
        bool right_connected = g_config_d2h.at("_cams").at("right").get<bool>();
        if (config.board_config.swap_left_and_right_cameras)
        {
            bool temp = left_connected;
            left_connected = right_connected;
            right_connected = temp;
        }

        if (!rgb_connected)
        {
            std::cout << "RGB camera (IMX378) is not detected on board! \n";
            if (config.ai.camera_input == "rgb")
            {
                std::cerr << WARNING "WARNING: NN inference was requested on RGB camera (IMX378), "
                                     "defaulting to right stereo camera (OV9282)! \n" ENDC;
                config.ai.camera_input = "right";
            }
        }
        if (left_connected ^ right_connected)
        {
            std::string cam_not_connected = (left_connected == false) ? "Left" : "Right";
            std::cerr << WARNING "WARNING: " << cam_not_connected
                      << " stereo camera (OV9282) is not detected on board! \n" ENDC;
            if (config.ai.camera_input != "rgb")
            {
                std::cerr << WARNING "WARNING: NN inference was requested on "
                          << config.ai.camera_input
                          << " stereo camera (OV9282), defaulting to RGB camera (IMX378)! \n" ENDC;
                config.ai.camera_input = "rgb";
            }
        }

        json json_config_obj;

        // Add video configuration if specified
        if (config_json.count("video_config") > 0)
        {
            json_config_obj["video_config"] = config_json["video_config"];
        }

        json_config_obj["board"]["clear-eeprom"] = config.board_config.clear_eeprom;
        json_config_obj["board"]["store-to-eeprom"] = config.board_config.store_to_eeprom;
        json_config_obj["board"]["override-eeprom"] = config.board_config.override_eeprom;
        json_config_obj["board"]["swap-left-and-right-cameras"] =
            config.board_config.swap_left_and_right_cameras;
        json_config_obj["board"]["left_fov_deg"] = config.board_config.left_fov_deg;
        json_config_obj["board"]["rgb_fov_deg"] = config.board_config.rgb_fov_deg;
        json_config_obj["board"]["left_to_right_distance_m"] =
            config.board_config.left_to_right_distance_m;
        json_config_obj["board"]["left_to_rgb_distance_m"] =
            config.board_config.left_to_rgb_distance_m;
        json_config_obj["board"]["stereo_center_crop"] =
            config.board_config.stereo_center_crop || stereo_center_crop;
        json_config_obj["board"]["name"] = config.board_config.name;
        json_config_obj["board"]["revision"] = config.board_config.revision;
        json_config_obj["_board"] = {{"calib_data", calibration_buff},
                                     {"mesh_left", left_mesh_buff},
                                     {"mesh_right", right_mesh_buff}};
        json_config_obj["depth"]["padding_factor"] = config.depth.padding_factor;
        json_config_obj["depth"]["depth_limit_mm"] = (int)(config.depth.depth_limit_m * 1000);
        json_config_obj["depth"]["median_kernel_size"] = config.depth.median_kernel_size;
        json_config_obj["depth"]["lr_check"] = config.depth.lr_check;
        json_config_obj["depth"]["warp_rectify"] = {
            {"use_mesh", config.depth.warp.use_mesh},
            {"mirror_frame", config.depth.warp.mirror_frame},
            {"edge_fill_color", config.depth.warp.edge_fill_color},
        };

        json_config_obj["_load_inBlob"] = true;
        json_config_obj["_pipeline"] = {{"_streams", json::array()}};

        json_config_obj["camera"]["rgb"]["resolution_w"] = config.rgb_cam_config.resolution_w;
        json_config_obj["camera"]["rgb"]["resolution_h"] = config.rgb_cam_config.resolution_h;
        json_config_obj["camera"]["rgb"]["fps"] = config.rgb_cam_config.fps;
        json_config_obj["camera"]["mono"]["resolution_w"] = config.mono_cam_config.resolution_w;
        json_config_obj["camera"]["mono"]["resolution_h"] = config.mono_cam_config.resolution_h;
        json_config_obj["camera"]["mono"]["fps"] = config.mono_cam_config.fps;

        std::string blob_file[] = {config.ai.blob_file, config.ai.blob_file2};

        std::vector<HostDataReader> _blob_reader(num_stages);
        std::vector<int> size_blob(num_stages);
        for (int stage = 0; stage < num_stages; stage++)
        {
            if (!blob_file[stage].empty())
            {
                if (!_blob_reader[stage].init(blob_file[stage]))
                {
                    std::cerr << WARNING "depthai: Error opening blob file: " << blob_file[stage]
                              << "\n" ENDC;
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

        json_config_obj["ai"]["NN_config"] = json_NN_meta;
        json_config_obj["ot"]["max_tracklets"] = config.ot.max_tracklets;
        json_config_obj["ot"]["confidence_threshold"] = config.ot.confidence_threshold;

        json_config_obj["app"]["sync_video_meta_streams"] = config.app_config.sync_video_meta_streams;
        json_config_obj["app"]["sync_sequence_numbers"] = config.app_config.sync_sequence_numbers;
        json_config_obj["app"]["usb_chunk_KiB"] = config.app_config.usb_chunk_KiB;

        bool add_disparity_post_processing_color = false;
        bool temp_measurement = false;

        std::vector<std::string> pipeline_device_streams;

        for (const auto& stream : config.streams)
        {
            if (c_streams_myriad_to_pc[stream.name].dimensions[0] == MONO_RES_AUTO)
            {
                c_streams_myriad_to_pc[stream.name].dimensions[0] =
                    config.mono_cam_config.resolution_h;
                c_streams_myriad_to_pc[stream.name].dimensions[1] =
                    config.mono_cam_config.resolution_w;
            }

            if (stream.name == "disparity_color")
            {
                c_streams_myriad_to_pc["disparity"].dimensions[0] =
                    c_streams_myriad_to_pc[stream.name].dimensions[0];
                c_streams_myriad_to_pc["disparity"].dimensions[1] =
                    c_streams_myriad_to_pc[stream.name].dimensions[1];
                add_disparity_post_processing_color = true;
                json obj = {{"name", "disparity"}};
                if (0.f != stream.max_fps)
                {
                    obj["max_fps"] = stream.max_fps;
                };
                json_config_obj["_pipeline"]["_streams"].push_back(obj);
            }
            else
            {
                if (stream.name == "meta_d2h")
                {
                    temp_measurement = true;
                }
                json obj = {{"name", stream.name}};

                if (!stream.data_type.empty())
                {
                    obj["data_type"] = stream.data_type;
                };
                if (0.f != stream.max_fps)
                {
                    obj["max_fps"] = stream.max_fps;
                };

                if (stream.name == "depth")
                {
                    obj["data_type"] = "uint16";
                }

                json_config_obj["_pipeline"]["_streams"].push_back(obj);
                pipeline_device_streams.push_back(stream.name);
            }
        }

        // host -> "config_h2d" -> device
        std::string pipeline_config_str_packed = json_config_obj.dump();
        std::cout << "config_h2d json:\n" << pipeline_config_str_packed << "\n";
        // resize, as xlink expects exact;y the same size for input:
        std::cout << "size of input string json_config_obj to config_h2d is ->"
                  << pipeline_config_str_packed.size() << std::endl;

        std::cout << "size of json_config_obj that is expected to be sent to config_h2d is ->"
                  << g_streams_pc_to_myriad.at("config_h2d").size << std::endl;

        assert(pipeline_config_str_packed.size() < g_streams_pc_to_myriad.at("config_h2d").size);
        pipeline_config_str_packed.resize(g_streams_pc_to_myriad.at("config_h2d").size, 0);

        if (!g_xlink->openWriteAndCloseStream(
                g_streams_pc_to_myriad.at("config_h2d"), pipeline_config_str_packed.data()))
        {
            std::cerr << WARNING "depthai: pipelineConfig write error\n" ENDC;
            break;
        }

        // host -> "host_capture" -> device
        auto stream = g_streams_pc_to_myriad.at("host_capture");
        g_host_capture_command =
            std::unique_ptr<HostCaptureCommand>(new HostCaptureCommand((stream)));
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

                std::cout << "Read: "
                          << _blob_reader[stage].readData(buff_blob.data(), size_blob[stage])
                          << std::endl;

                // inBlob
                StreamInfo blobInfo;
                blobInfo.name = "inBlob";
                blobInfo.size = size_blob[stage];

                if (!g_xlink->openWriteAndCloseStream(blobInfo, buff_blob.data()))
                {
                    std::cout << "depthai: pipelineConfig write error: Blob size too big: "
                              << size_blob[stage] << "\n";
                    break;
                }
                printf("depthai: done sending Blob file %s\n", blob_file[stage].c_str());

                // outBlob
                StreamInfo outBlob("outBlob", 102400);
                                
               
                std::string blob_info_str;

                int out_blob_length = g_xlink->openReadAndCloseStream(
                    outBlob,
                    blob_info_str
                    );
                if(out_blob_length == -1)
                {
                    break;
                }
                nlohmann::json blob_info;
                if (!getJSONFromString(blob_info_str, blob_info))
                {
                    std::cout << "depthai: error parsing blob_info\n";
                    break;
                }
                // std::cout << blob_info << std::endl;

                std::vector<nlohmann::json> input_layers = blob_info["input_layers"].get<std::vector<nlohmann::json>>();
                std::vector<nlohmann::json> output_layers = blob_info["output_layers"].get<std::vector<nlohmann::json>>();

                for(auto input_json : input_layers)
                {
                    dai::TensorInfo _tensors_info_input(input_json);
                    std::cout << "Input layer : " << std::endl;
                    std::cout << _tensors_info_input << std::endl;

                    tensors_info_input.push_back(_tensors_info_input);
                }

                for(auto output_json : output_layers)
                {
                    dai::TensorInfo _tensors_info_output(output_json);
                    std::cout << "Output layer : " << std::endl;
                    std::cout << _tensors_info_output << std::endl;
                    
                    tensors_info_output.push_back(_tensors_info_output);

                    NN_config.push_back(NN_config_stage[stage]);
                }

                int satisfied_resources = blob_info["metadata"]["satisfied_resources"];
                int number_of_shaves = blob_info["metadata"]["number_of_shaves"];
                int number_of_cmx_slices = blob_info["metadata"]["number_of_cmx_slices"];

                if (stage == 0)
                {
                    nn_to_depth_mapping["off_x"] = blob_info["metadata"]["nn_to_depth"]["offset_x"];
                    nn_to_depth_mapping["off_y"] = blob_info["metadata"]["nn_to_depth"]["offset_y"];
                    nn_to_depth_mapping["max_w"] = blob_info["metadata"]["nn_to_depth"]["max_width"];
                    nn_to_depth_mapping["max_h"] = blob_info["metadata"]["nn_to_depth"]["max_height"];
                    printf("CNN to depth bounding-box mapping: start(%d, %d), max_size(%d, %d)\n",
                            nn_to_depth_mapping["off_x"],
                            nn_to_depth_mapping["off_y"],
                            nn_to_depth_mapping["max_w"],
                            nn_to_depth_mapping["max_h"]);
                }

                if (stage == 0) {
     
                    c_streams_myriad_to_pc["previewout"].dimensions = {
                                                                       (int)tensors_info_input[0].get_dimension(dai::TensorInfo::Dimension::C),
                                                                       (int)tensors_info_input[0].get_dimension(dai::TensorInfo::Dimension::H),
                                                                       (int)tensors_info_input[0].get_dimension(dai::TensorInfo::Dimension::W),
                                                                       };
                }
                // check CMX slices & used shaves
                if (number_of_cmx_slices > config.ai.cmx_slices)
                {
                    std::cerr << WARNING "Error: Blob is compiled for " << number_of_cmx_slices
                              << " cmx slices but device is configured to calculate on " << config.ai.cmx_slices << "\n" ENDC;
                    break;
                }

                if (number_of_shaves > config.ai.shaves)
                {
                    std::cerr << WARNING "Error: Blob is compiled for " << number_of_shaves
                              << " shaves but device is configured to calculate on " << config.ai.shaves << "\n" ENDC;
                    break;
                }

                if(!satisfied_resources)
                {
                    std::cerr << WARNING "ERROR: requested CNN resources overlaps with RGB camera \n" ENDC;
                    return nullptr;
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
                stream_name_to_idx[available_streams_ordered[i]] = i;
            }

            // check requested streams are in available streams
            bool wrong_stream_name = false;
            for (const auto& stream_name : pipeline_device_streams)
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
            std::sort(
                std::begin(pipeline_device_streams), std::end(pipeline_device_streams),
                [&stream_name_to_idx](const std::string& a, const std::string& b) {
                    return stream_name_to_idx[a] < stream_name_to_idx[b];
                });
        }

        // pipeline
        if(gl_result == nullptr)
            gl_result = std::shared_ptr<CNNHostPipeline>(new CNNHostPipeline(tensors_info_input, tensors_info_output, NN_config));

        for (const std::string& stream_name : pipeline_device_streams)
        {
            std::cout << "Host stream start:" << stream_name << "\n";

            if (g_xlink->openStreamInThreadAndNotifyObservers(
                    c_streams_myriad_to_pc.at(stream_name)))
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
                new DisparityStreamPostProcessor(add_disparity_post_processing_color));

            const std::string stream_in_name = "disparity";
            const std::string stream_out_color_name = "disparity_color";

            if (g_xlink->openStreamInThreadAndNotifyObservers(
                    c_streams_myriad_to_pc.at(stream_in_name)))
            {
                g_disparity_post_proc->observe(
                    *g_xlink.get(), c_streams_myriad_to_pc.at(stream_in_name));

                if (add_disparity_post_processing_color)
                {
                    gl_result->makeStreamPublic(stream_out_color_name);
                    gl_result->observe(
                        *g_disparity_post_proc.get(),
                        c_streams_myriad_to_pc.at(stream_out_color_name));
                }
            }
            else
            {
                std::cout << "depthai: stream open error " << stream_in_name << " (2)\n";
                // TODO: rollback correctly!
                break;
            }
        }

        if (temp_measurement)
        {
            // device support listener
            g_device_support_listener =
                std::unique_ptr<DeviceSupportListener>(new DeviceSupportListener);

            g_device_support_listener->observe(
                *g_xlink.get(), c_streams_myriad_to_pc.at("meta_d2h"));
        }

        init_ok = true;
        std::cout << "depthai: INIT OK!\n";
    } while (false);

    if (!init_ok)
    {
        gl_result = nullptr;
    }

    return gl_result;
}

// create_mesh(std::vector<std::vector<float>>& M, std::vector<std::vector<float>>& R,
// std::vector<float>d, int bin_size){
//     float fx = M[0][0], fy = M[1][1], u0 = M[0][2], v0 = M[1][2];
//     float k1 = d[0], k2 = d[1] , p1 = d[2] , p2 = d[3] , k3 = d[4];
//     float s4 = d[11], k4 = d[5], k5 = d[6], k6 = d[7], s1 = d[8];
//     float s2 = d[9] , s3 = d[10], tauX = d[12] , tauY = d[13];

//     std::vector<std::vector<float>> matTilt{{1,0,0},
//                                             {0,1,0},
//                                             {0,0,1}};

//     std::vector<std::vector<float>> _x = 0, _y = 0, _w = 0, ir; // _x, _y, _w are 2D coordinates
//     in homogeneous coordinates mat_inv(mat_mul(self.M2, R), ir); // TODO: Change it to using LU
//     later to reduce the computation load if necessary std::cout << "Printing Res for creating
//     mesh " << width << " Height: " << height << "  " << std::endl;

//     for(int i = 0; i < height; ++i){

//         for(int j = 0; j < width; ++j){

//         }

//     }

// }

// -40 for
// create_mesh(std::vector<float>& x_map_buff, std::vector<float>& y_map_buff, std::vector<float>&
// mesh, int32_t res){
//
// }

void Device::request_jpeg()
{
    if (g_host_capture_command != nullptr)
    {
        g_host_capture_command->capture();
    }
}

void Device::request_af_trigger()
{
    if (g_host_capture_command != nullptr)
    {
        g_host_capture_command->afTrigger();
    }
}

void Device::request_af_mode(CaptureMetadata::AutofocusMode mode)
{
    if (g_host_capture_command != nullptr)
    {
        g_host_capture_command->afMode(mode);
    }
}

void Device::send_disparity_confidence_threshold(uint8_t confidence)
{
    if (g_host_capture_command != nullptr)
    {
        g_host_capture_command->sendDisparityConfidenceThreshold(confidence);
    }
}

void Device::send_camera_control(CameraControl::CamId camera_id,
        CameraControl::Command command_id,
        const std::string &extra_args) {
    if(g_host_capture_command != nullptr) {
        g_host_capture_command->sendCameraControl(camera_id, command_id, extra_args.c_str());
    }
}

std::map<std::string, int> Device::get_nn_to_depth_bbox_mapping(){
    return nn_to_depth_mapping;
}
