#include "depthai/depthai_wrapper.hpp"
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace DepthAI {

const std::unordered_map<int, int> DepthAI::height_to_width_map_ = { { 720, 1280 }, { 800, 1280 }, { 400, 640 }, { 2160, 3840 }, { 1080, 1920 } };

DepthAI::DepthAI(const std::string& usb_device, const std::string& config_file, bool usb2_mode)
    : Device(usb_device, usb2_mode)
{
    // Device(std::string(""), std::string(""))
    // opening config file
    std::ifstream file(config_file);
    std::ostringstream file_stream;
    if (file) {
        file_stream << file.rdbuf();
    } else {
        throw std::runtime_error("Config file could not be found at " + config_file);
    }
    std::string config_str = file_stream.str();
    // creating pipeline using config and saving the json of config.
    pipeline_ = create_pipeline(config_str);
    config_json_ = nlohmann::json::parse(config_str);

    // extract resolution of the frames and create cv::Mat placeholders
    set_resolution();
    create_frame_holders();

    std::cout << "RGB Camera resolution : " << rgb_width_ << "x" << rgb_height_ << std::endl;
    std::cout << "Mono Camera resolution : " << mono_width_ << "x" << mono_height_ << std::endl;
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
            rgb_width_ = height_to_width_map_.at(rgb_height_);
        } else {
            rgb_height_ = DefaultRGBHeight;
            rgb_width_ = DefaultRGBWidth;
        }
        /*
         * if stereo camera res is set in config_json_ fetch it to set 
         * the height of the image else set the default
         */
        if (camera_conf_obj.contains("mono")) {
            auto& mono_camera_conf_obj = camera_conf_obj.at("mono");
            mono_height_ = mono_camera_conf_obj.at("resolution_h").get<int32_t>();
            mono_width_ = height_to_width_map_.at(mono_height_);
        } else {
            mono_height_ = DefaultMonoHeight;
            mono_width_ = DefaultMonoWidth;
        }
    } else {
        rgb_height_ = DefaultRGBHeight;
        rgb_width_ = DefaultRGBWidth;
        mono_height_ = DefaultMonoHeight;
        mono_width_ = DefaultMonoWidth;
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
                if (it.get<std::string>() == "previewout") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(rgb_height_, rgb_width_, CV_8UC3);
                    image_stream_holder_["previewout"] = img;
                }
            } else {
                const auto& name = it.at("name").get<std::string>();
                if (name == "previewout") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(rgb_height_, rgb_width_, CV_8UC3);
                    image_stream_holder_["previewout"] = img;
                }
                if (name == "color") {
                    std::cout << "alloc failed ->" << rgb_height_ << " " << rgb_width_ << std::endl;
                    CV_mat_ptr img = std::make_shared<cv::Mat>(rgb_height_, rgb_width_, CV_8UC3);
                    image_stream_holder_["color"] = img;
                } else if (name == "depth") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(mono_height_, mono_width_, CV_16UC1);
                    image_stream_holder_["depth"] = img;
                } else if (name == "disparity_color") {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(mono_height_, mono_width_, CV_16UC3);
                    image_stream_holder_["disparity_color"] = img;
                } else {
                    CV_mat_ptr img = std::make_shared<cv::Mat>(mono_height_, mono_width_, CV_8U);
                    image_stream_holder_[name] = img;
                }
            }
        }
    }
}

// How about creating streams for everything in the vector in constructor and then updating them
// each time get frames is called ? How will it receive if it is not called for a small span of time ?

// TODO(sachin): Crash on size mismatch. DO this later.
// TODO(sachin): modify it using a struct to include time stamps and not loose frames.

void DepthAI::get_streams(std::unordered_map<std::string, CV_mat_ptr>& output_streams)
{
    int count = image_stream_holder_.size();
    std::unordered_set<std::string> dirty_check;
    while (count) { // count and dirty check is used incase same stream appears twice before other streams.
        PacketsTuple packets = pipeline_->getAvailableNNetAndDataPackets(true);

        // iterating over the packets to extract all the image streams specified in the config
        for (const auto& sub_packet : std::get<1>(packets)) {
            std::unordered_map<std::string, CV_mat_ptr>::iterator it = image_stream_holder_.find(sub_packet->stream_name);
            if (it != image_stream_holder_.end()) {

                const auto& received_data = sub_packet->getData();
                if (sub_packet->stream_name == "color") {

                    cv::Mat yuv(rgb_height_ * 3 / 2, rgb_width_, CV_8UC1);
                    unsigned char* img_ptr = reinterpret_cast<unsigned char*>(yuv.data);
                    memcpy(img_ptr, received_data, sub_packet->size());
                    cv::cvtColor(yuv, *(it->second), cv::COLOR_YUV2BGR_IYUV);

                } else {
                    unsigned char* img_ptr = reinterpret_cast<unsigned char*>((it->second)->data);
                    memcpy(img_ptr, received_data, sub_packet->size());
                }
                if (dirty_check.find(sub_packet->stream_name) == dirty_check.end()) {
                    dirty_check.insert(sub_packet->stream_name);
                    count--;
                }
            }
        }
    }
    std::cout << "hhihii 222-->>" << std::endl;

    output_streams = image_stream_holder_;
    std::cout << "hhihii finished" << std::endl;
}

} // namespace DepthAI
