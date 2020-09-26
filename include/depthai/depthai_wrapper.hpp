#include "depthai/device.hpp"
#include "nlohmann/json.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace DepthAI {

using CV_mat_ptr = std::shared_ptr<cv::Mat>; // shared ptr for cv::Mat
using PacketsTuple = std::tuple<
    std::list<std::shared_ptr<NNetPacket>>,
    std::list<std::shared_ptr<HostDataPacket>>>; // tuple of data packets on which Depthai is
// publishing the streams

/**  DepthAI:
 *   This is a wrapper on the host side of the device which 
 *   helps in initializing device, creating pipeline
 *   and fetching the frames as cv::Mat. 
 */
class DepthAI : public Device {

public:
    /**  Constructor:
     *   initializing device and creating pipeline
     *  @param usb_device (std::string): Provide the path of the device
     *  or pass empty string to choose the default device.
     *  @param config_file (std::string): Provides the json file which is
     *  used by the Depthai to configure the device
     *  @param usb2_mode (bool): set to true to connect over usb2
     */
    DepthAI(const std::string& usb_device, const std::string& config_file, bool usb2_mode = false);

    /* API to stream output frames from device.
     * AI models results and metadata is WIP
     * @param output_streams (std::unordered_map<std::string, CV_mat_ptr>): output_streams is a placeholder 
     * to extract all the streams being sent by the device. 
     */
    void get_frames(std::unordered_map<std::string, CV_mat_ptr>& output_streams);

    /* Destructor */
    ~DepthAI() = default;

    static const std::unordered_map<int, int> width_to_height_map_; // mapping of width with height

private:
    std::shared_ptr<CNNHostPipeline> pipeline_; // Depthai's pipeline object.

    PacketsTuple packets_;
    nlohmann::json config_json_; // Config file's json holder

    int rgb_width_; //  represents the width and height of the rgb frame
    int rgb_height_; //  represents the width and height of the rgb frame
    int mono_width_; //  represents the width and height of the stereo camera frame
    int mono_height_; //  represents the width and height of the stereo camera frame

    std::unordered_map<std::string, CV_mat_ptr> image_stream_holder;

    /* 
     * Fetches the width and height of the color and stereo 
     * frames from the config.
     */
    void set_resolution();

    /* sets the shape of the placeholders of image streams */
    void create_frame_holders();
};

} // namespace DepthAI
