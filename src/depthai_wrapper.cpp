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
    _config_str = file_stream.str();

    _pipeline = this->create_pipeline(_config_str);
    _config_json = nlohmann::json::parse(_config_str);

    this->set_resolution();
    this->create_frame_holders();
}

void DepthAI::set_resolution(){
    
        if(_config_json.contains("camera")){
            auto& camera_conf_obj = _config_json.at("camera");
            if(camera_conf_obj.contains("rgb")){
                auto& rgb_camera_conf_obj = camera_conf_obj.at("rgb");
                _rgb_height = rgb_camera_conf_obj.at("resolution_h").get<int32_t>();
                // _rgb_width = width_map[_rgb_height];
            }
            else{
                _rgb_height = 3040;
                _rgb_width  = 4056;
            }

            if(camera_conf_obj.contains("mono")){
                auto& mono_camera_conf_obj = camera_conf_obj.at("mono");
                _mono_height = mono_camera_conf_obj.at("resolution_h").get<int32_t>();
                // _mono_width = width_map[_mono_height];
            }
            else{
                _mono_height = 720;
                _mono_width  = 1280;
            }
        }
        else{
            _rgb_height  = 3040;
            _rgb_width   = 4056;
            _mono_height = 720;
            _mono_width  = 1280;
        }

}


void DepthAI::create_frame_holders(){
    
    if (_config_json.contains("streams"))
    {
        for (auto it : _config_json.at("streams"))
        {
            // "metaout"
            if (it.is_string())
            {
                if(it.get<std::string>() == "metaout")
                    continue;
                stream_names.push_back(it.get<std::string>());
                if(it.get<std::string>() == "previewout"){
                    // std::shared_ptr<cv::Mat> img = std::make_shared(cv::Mat(_rgb_height,_rgb_width, CV_8UC3));
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(_rgb_height,_rgb_width, CV_8UC3);
                    _image_streams.push_back(img);
                }
                else{
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(_mono_height,_mono_width, CV_8UC3);
                    // std::shared_ptr<cv::Mat> img(new cv::Mat(_mono_height,_mono_width, CV_8UC3));
                    _image_streams.push_back(img);
                }

            }
            else
            // {"name": "depth", "data_type": "uint16", "max_fps": 4.0}
            {
                const auto &name = it.at("name").get<std::string>();
                // streams.emplace_back(name);
                // auto &stream = streams.back();
                std::cout << name << " -streams" << std::endl;
                stream_names.push_back(name);
                if(name == "previewout"){
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(_rgb_height,_rgb_width, CV_8UC3);
                    _image_streams.push_back(img);
                }
                else if(name == "depth_raw"){
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(_mono_height,_mono_width, CV_16UC1);
                    _image_streams.push_back(img);
                }
                else if(name == "disparity_color"){
                    std::shared_ptr<cv::Mat> img = std::make_shared<cv::Mat>(_mono_height,_mono_width, CV_16UC1);
                    _image_streams.push_back(img);
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
        _packets = _pipeline->getAvailableNNetAndDataPackets(true);
        
        for(auto sub_packet : std::get<1>(_packets)){
            it = std::find(stream_names.begin(), stream_names.end(), sub_packet->stream_name); 
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