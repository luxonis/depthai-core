#include "depthai/depthai_wrapper.hpp"
#include <fstream>
#include <sstream>

#define WARNING "\033[1;5;31m" // Adds color to error statements
#define ENDC "\033[0m"

namespace DepthAI {

const std::unordered_map<int, int> DepthAI::width_map_ = { { 720, 1280 }, { 800, 1280 }, { 400, 640 } };

DepthAI::DepthAI(const std::string& usb_device, const std::string& config_file, bool usb2_mode)
    : Device(usb_device, usb2_mode)
{
    std::ifstream file(config_file);
    std::ostringstream file_stream;
    if (file) {
        file_stream << file.rdbuf();
    } else {
        std::cerr << WARNING "config file provided is not found" ENDC << std::endl;
        abort();
    }
    std::string config_str = file_stream.str();

    pipeline_ = create_pipeline(config_str);
    config_json_ = nlohmann::json::parse(config_str);

    set_resolution();
    create_frame_holders();

    // width_map_[720] = 1280;
    // width_map_[800] = 1280;
    // width_map_[400] = 640;
}

void DepthAI::set_resolution()
{
    // if camera is defined check if resolution of stereo or rgb is defined. else set to default
    if (config_json_.contains("camera")) {
        auto& camera_conf_obj = config_json_.at("camera");
        /*
         * if rgb res is set in config_json_ fetch it to set 
         * the height of the image else set the default
         */
        if (camera_conf_obj.contains("rgb")) {
            auto& rgb_camera_conf_obj = camera_conf_obj.at("rgb");
            rgb_height_ = rgb_camera_conf_obj.at("resolution_h").get<int32_t>();
            // _rgb_width = width_map_[_rgb_height];
        } else {
            rgb_height_ = 3040;
            rgb_width_ = 4056;
        }
        /*
         * if stereo camera res is set in config_json_ fetch it to set 
         * the height of the image else set the default
         */
        if (camera_conf_obj.contains("mono")) {
            auto& mono_camera_conf_obj = camera_conf_obj.at("mono");
            mono_height_ = mono_camera_conf_obj.at("resolution_h").get<int32_t>();
            auto it = width_map_.find(mono_height_);
            mono_width_ = it->second;
        } else {
            mono_height_ = 720;
            mono_width_ = 1280;
        }
    } else {
        rgb_height_ = 3040;
        rgb_width_ = 4056;
        mono_height_ = 720;
        mono_width_ = 1280;
    }
}

void DepthAI::create_frame_holders()
{
    // creating place holders for all the streams enabled in config
    if (config_json_.contains("streams")) {
        for (const auto& it : config_json_.at("streams")) {
            if (it.is_string()) {
                if (it.get<std::string>() == "metaout")
                    continue;
                stream_names_.push_back(it.get<std::string>());
                if (it.get<std::string>() == "previewout") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(rgb_height_, rgb_width_, CV_8UC3);
                    image_streams_.push_back(img);
                    image_stream_holder["previewout"] = img;
                }
            } else {
                const auto& name = it.at("name").get<std::string>();
                stream_names_.push_back(name);
                if (name == "previewout") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(rgb_height_, rgb_width_, CV_8UC3);
                    image_streams_.push_back(img);
                    image_stream_holder["previewout"] = img;
                } else if (name == "depth") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(mono_height_, mono_width_, CV_16UC1);
                    image_streams_.push_back(img);
                    image_stream_holder["depth"] = img;
                } else if (name == "disparity_color") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(mono_height_, mono_width_, CV_16UC3);
                    image_streams_.push_back(img);
                    image_stream_holder["disparity_color"] = img;
                }
            }
        }
    }
}

// How about creating streams for everything in the vector in constructor and then updating them
// each time get frames is called ? How will it receive if it is not called for a small span of time ?

// TODO(sachin): Crash on size mismatch. DO this later.
// TODO(sachin): modify it using a struct to include time stamps and not loose frames. 

void DepthAI::get_frames(std::unordered_map<std::string, CV_mat_ptr>& output_streams)
{
    int count = image_stream_holder.size();
    std::set<std::string> dirty_check;
    while (count) { // count and dirty check is used incase same stream appears twice before other streams. 
        packets_ = pipeline_->getAvailableNNetAndDataPackets(true);

        for (const auto& sub_packet : std::get<1>(packets_)) {
        std::unordered_map<std::string, CV_mat_ptr>::iterator it = image_stream_holder.find(sub_packet->stream_name);
            if (it != image_stream_holder.end()) {
                unsigned char* img_ptr = reinterpret_cast<unsigned char*>((it->second)->data);
                const auto& received_data = sub_packet->getData();

                memcpy(img_ptr, received_data, sub_packet->size());
                if (dirty_check.find(sub_packet->stream_name) == dirty_check.end()) {
                    dirty_check.insert(sub_packet->stream_name);
                    count--;
                }
            }
        }
    }

    output_streams = image_stream_holder;
}

} // namespace DepthAI
