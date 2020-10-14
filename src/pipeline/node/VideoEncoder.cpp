#include "depthai/pipeline/node/VideoEncoder.hpp"

namespace dai {
namespace node {

    VideoEncoder::VideoEncoder(const std::shared_ptr<PipelineImpl>& par, int64_t nodeId) : Node(par, nodeId) {}

    std::string VideoEncoder::getName() {
        return "VideoEncoder";
    }

    std::vector<Node::Input> VideoEncoder::getInputs() {
        return {input};
    }

    std::vector<Node::Output> VideoEncoder::getOutputs() {
        return {bitstream};
    }

    nlohmann::json VideoEncoder::getProperties() {
        nlohmann::json j;
        nlohmann::to_json(j, properties);
        return j;
    }

    std::shared_ptr<Node> VideoEncoder::clone() {
        return std::make_shared<std::decay<decltype(*this)>::type>(*this);
    }

    void VideoEncoder::setDefaultProfilePreset(int width, int height, VideoEncoderProperties::Profile profile) {
        properties.width = width;
        properties.height = height;
        properties.profile = profile;

        // TODO(themarpe) - add some meaningful defaults for each preset
        switch(profile) {
            case VideoEncoderProperties::Profile::MJPEG:
                properties.quality = 95;
                break;

            case VideoEncoderProperties::Profile::H264_BASELINE:

                break;

            case VideoEncoderProperties::Profile::H264_HIGH:

                break;

            case VideoEncoderProperties::Profile::H264_MAIN:

                break;

            case VideoEncoderProperties::Profile::H265_MAIN:

                break;

            default:
                break;
        }
    }

}  // namespace node
}  // namespace dai
