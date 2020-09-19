#include "depthai/device.hpp"
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <sstream>
#include "nlohmann/json.hpp"
#include <vector>



namespace DepthAI
{

using CV_mat_ptr = std::shared_ptr<cv::Mat>;

class DepthAI: public Device{


public:
    DepthAI(
            std::string usb_device,
            std::string config_file, 
            bool usb2_mode = false);

void get_frames(std::unordered_map<std::string, CV_mat_ptr>& output_streams);

~DepthAI();

private:
    using PacketsTuple = std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>>;

    std::shared_ptr<CNNHostPipeline> _pipeline;

    PacketsTuple _packets;
    std::string _config_str;
    nlohmann::json _config_json;

    int _rgb_width;
    int _rgb_height;
    int _mono_width;
    int _mono_height; 

    std::vector<std::string> stream_names;
    std::vector<CV_mat_ptr> _image_streams;

    void set_resolution();
    void create_frame_holders();
};


}