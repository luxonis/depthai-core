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

using CV_mat_ptr = std::shared_ptr<cv::Mat>;  // shared ptr for cv::Mat
using PacketsTuple = std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>>;  // tuple of data packets on which Depthai is
                                                      // publishing the streams

/**  DepthAI:
 *   This is a wrapper on the host side of the device which 
 *   helps in initializing OAK device, creating pipeline
 *   and fetching the frames as cv::Mat. 
 */

class DepthAI : public Device
{

public:
    /**  Constructor:
     *   initializing OAK device and creating pipeline
     *  @param usb_device (std::string): Provide the path of the device
     *  or pass empty string to choose the default device.
     *  @param config_file (std::string): Provides the json file which is
     *  used by the Depthai to configure the OAK-D.
     *  @param usb2_mode (bool): set to true to connect over usb2
     */
    DepthAI(const std::string& usb_device, const std::string& config_file, bool usb2_mode = false);

    /* API to stream output frames from OAK.
     * AI data and metadata is WIP
     */
    void get_frames(std::unordered_map<std::string, CV_mat_ptr>& output_streams);

    /* Destructor */
    ~DepthAI();

private:
    
    std::shared_ptr<CNNHostPipeline> pipeline_;  // Depthai's pipeline object.

    PacketsTuple packets_;
    nlohmann::json config_json_;

    int rgb_width_;
    int rgb_height_;
    int mono_width_;
    int mono_height_;

    std::vector<std::string> stream_names_;
    std::vector<CV_mat_ptr> image_streams_;
    std::unordered_map<int, int> width_map_;
    void set_resolution();
    void create_frame_holders();
};

}  // namespace DepthAI

