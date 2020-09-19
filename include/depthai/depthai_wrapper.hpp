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

class DepthAI : public Device
{

public:
    /**  Constructor:
     *   initializing OAK device and creating pipeline
     *   to capture camera streams and AI outputs.
     */
    DepthAI(std::string usb_device, std::string config_file, bool usb2_mode = false);

    /* API to stream output frames from OAK.
     * AI data and metadata is WIP
     */

    void get_frames(std::unordered_map<std::string, CV_mat_ptr>& output_streams);

    /* Destructor */
    ~DepthAI();

private:
    using PacketsTuple = std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>>;  // tuple of data packets on which Depthai is
                                                      // publishing the streams

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