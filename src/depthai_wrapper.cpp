#include "depthai/depthai_wrapper.hpp"

namespace DepthAI
{

DepthAI::DepthAI(
                std::string usb_device,
                std::string config_file, 
                bool usb2_mode) : Device(usb_device, usb2_mode) {

    std::ifstream file(config_file);
    std::ostringstream file_stream;
    file_stream << file.rdbuf();
    std::string config_str = file_stream.str();

    pipeline_ = this->create_pipeline(config_str);
    config_json_ = nlohmann::json::parse(config_str);

    this->set_resolution();
    this->create_frame_holders();

    width_map_[720] = 1280;
    width_map_[800] = 1280;
    width_map_[400] = 640;

}

void DepthAI::set_resolution(){
    
        if(_config_json.contains("camera")){
            auto& camera_conf_obj = _config_json.at("camera");
            if(camera_conf_obj.contains("rgb")){
                auto& rgb_camera_conf_obj = camera_conf_obj.at("rgb");
                rgb_height_ = rgb_camera_conf_obj.at("resolution_h").get<int32_t>();
                // _rgb_width = width_map[_rgb_height];
            }
            else{
                rgb_height_ = 3040;
                rgb_width_  = 4056;
            }

            if(camera_conf_obj.contains("mono")){
                auto& mono_camera_conf_obj = camera_conf_obj.at("mono");
                mono_height_ = mono_camera_conf_obj.at("resolution_h").get<int32_t>();
                mono_width_ = width_map[mono_height_];
            }
            else{
                mono_height_ = 720;
                mono_width_  = 1280;
            }
        }
        else{
            rgb_height_  = 3040;
            rgb_width_   = 4056;
            mono_height_ = 720;
            mono_width_  = 1280;
        }

}


void DepthAI::create_frame_holders(){
    
    if (config_json_.contains("streams"))
    {
        for (auto it : config_json_.at("streams"))
        {
            // "metaout"
            if (it.is_string())
            {
                if(it.get<std::string>() == "metaout")
                    continue;
                stream_names_.push_back(it.get<std::string>());
                if(it.get<std::string>() == "previewout"){
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(rgb_height_, rgb_width_, CV_8UC3);
                    image_streams_.push_back(img);
                }
                else{
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(mono_height_, mono_width_, CV_8UC3);
                    image_streams_.push_back(img);
                }

            }
            else
            {
                const auto &name = it.at("name").get<std::string>();
                // streams.emplace_back(name);
                // auto &stream = streams.back();
                std::cout << name << " -streams" << std::endl;
                stream_names_.push_back(name);
                if(name == "previewout"){
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(rgb_height_,rgb_width_, CV_8UC3);
                    image_streams_.push_back(img);
                }
                else if(name == "depth_raw"){
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(mono_height_,mono_width_, CV_16UC1);
                    image_streams_.push_back(img);
                }
                else if(name == "disparity_color"){
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(mono_height_,mono_width_, CV_16UC1);
                    image_streams_.push_back(img);
                }

            }
        }
    }
}


~DepthAI::DepthAI() = default;

// How about creating streams for everything in the vector in constructor and then updating them each time get frames is called ?


// TODO: Crash on size mismatch. DO this later.
// TODO: save config as json and check if the streams are in the config and crash if they aren't
                
void DepthAI::get_frames(std::unordered_map<std::string, CV_mat_ptr>& output_streams){

    int count = stream_names.size();
    std::vector<std::string>::iterator it;
    std::vector<bool> dirty_check(stream_names.size(), false);
    while(count){
        packets_ = pipeline_->getAvailableNNetAndDataPackets(true);
        
        for(auto sub_packet : std::get<1>(packets_)){
            it = std::find(stream_names_.begin(), stream_names_.end(), sub_packet->stream_name); 
            if(it != stream_names.end()){
                int index = it - stream_names.begin();
                unsigned char* img_ptr = reinterpret_cast<unsigned char*>(_image_streams[index]->data);
                auto received_data = sub_packet->getData();
                
                // assert() 
                memcpy(img_ptr, received_data, sub_packet->size());
                if(!dirty_check[index]){
                    dirty_check[index] = true;
                    count--;
                }
            }
         }
    }

    for(int i = 0; i < stream_names.size(); ++i){
        output_streams[stream_names[i]] = _image_streams[i];
    }

}

}