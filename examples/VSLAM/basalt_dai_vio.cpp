#include <rerun.hpp>

#include "depthai/basalt/BasaltVIO.hpp"
#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/host/HostNode.hpp"
#include "depthai/pipeline/datatype/TransformData.hpp"

class VideoDecoder : public dai::NodeCRTP<dai::node::HostNode, VideoDecoder> {
   public:
    constexpr static const char* NAME = "VideoDecoder";

   public:
    void build() {
        // inputImg.setBlocking(false);
        // inputImg.setMaxSize(0);
    }
    Input& inputLeft = inputs["left"];
    Input& inputRight = inputs["right"];
    Output outputImg{*this, {.name = "outImg", .types = {{dai::DatatypeEnum::MessageGroup, true}}}};
    void convertAndAddToGroup(const std::string& name, std::shared_ptr<dai::ImgFrame> imgFrame, std::shared_ptr<dai::MessageGroup> group) {
        std::vector<uint8_t> vecData(imgFrame->getData().begin(), imgFrame->getData().end());
        cv::Mat decodedFrame = cv::imdecode(cv::Mat(vecData), cv::IMREAD_GRAYSCALE);
        std::shared_ptr<dai::ImgFrame> outFrame = std::make_shared<dai::ImgFrame>();
        outFrame->setTimestamp(imgFrame->getTimestamp());
        outFrame->setCvFrame(decodedFrame, dai::ImgFrame::Type::GRAY8);
        group->add(name, outFrame);
    }
    std::shared_ptr<dai::Buffer> runOnce(std::shared_ptr<dai::MessageGroup> in) override {
            auto outGroup = std::make_shared<dai::MessageGroup>();
            auto left = in->get<dai::ImgFrame>("left");
            auto right = in->get<dai::ImgFrame>("right");
            convertAndAddToGroup("left", left, outGroup);
            convertAndAddToGroup("right", right, outGroup);
            outputImg.send(outGroup);
        
    }
};

rerun::Collection<rerun::TensorDimension> tensor_shape(const cv::Mat& img) {
    return {img.rows, img.cols, img.channels()};
};
class RerunStreamer : public dai::NodeCRTP<dai::node::ThreadedHostNode, RerunStreamer> {
   public:
    constexpr static const char* NAME = "RerunStreamer";

   public:
    void build() {}

    /**
     * Input for any ImgFrame messages
     * Default queue is blocking with size 8
     */
    Input inputTrans{*this, {.name = "inTrans", .types = {{dai::DatatypeEnum::TransformData, true}}}};
    Input inputImg{*this, {.name = "inImg", .types = {{dai::DatatypeEnum::ImgFrame, true}}}};
    void run() override {
        const auto rec = rerun::RecordingStream("rerun");
        rec.spawn().exit_on_failure();
        rec.log_timeless("world", rerun::ViewCoordinates::RDF);
        rec.log("world/ground", rerun::Boxes3D::from_half_sizes({{3.f, 3.f, 0.00001f}}));

        while(isRunning()) {
            std::shared_ptr<dai::TransformData> transData = inputTrans.get<dai::TransformData>();
            auto imgFrame = inputImg.get<dai::ImgFrame>();
            if(transData != nullptr) {
                double x, y, z, qx, qy, qz, qw;
                transData->getTranslation(x, y, z);
                transData->getQuaternion(qx, qy, qz, qw);
                auto position = rerun::Vec3D(x, y, z);

                rec.log("world/camera", rerun::Transform3D(position, rerun::datatypes::Quaternion::from_xyzw(qx, qy, qz, qw)));
                positions.push_back(position);
                rerun::LineStrip3D lineStrip(positions);
                rec.log("world/trajectory", rerun::LineStrips3D(lineStrip));
                rec.log("world/camera/image", rerun::Pinhole::from_focal_length_and_resolution({398.554f, 398.554f}, {640.0f, 400.0f}));
                rec.log("world/camera/image/rgb",
                        rerun::Image(tensor_shape(imgFrame->getCvFrame()), reinterpret_cast<const uint8_t*>(imgFrame->getCvFrame().data)));
            }
        }
    }
    std::vector<rerun::Vec3D> positions;
};

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    int fps = 60;
    int width = 640;
    int height = 400;
    // Define sources and outputs
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto imu = pipeline.create<dai::node::IMU>();
    // auto sync = pipeline.create<dai::node::Sync>();
    auto odom = pipeline.create<dai::node::BasaltVIO>();
    auto videncL = pipeline.create<dai::node::VideoEncoder>();
    auto videncR = pipeline.create<dai::node::VideoEncoder>();
    auto decoder = pipeline.create<VideoDecoder>();


    auto rerun = pipeline.create<RerunStreamer>();
    imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(5);
    odom->setImuUpdateRate(200);

    left->setCamera("left");
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    left->setFps(fps);
    right->setCamera("right");
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    right->setFps(fps);
    videncL->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
    videncR->setProfile(dai::VideoEncoderProperties::Profile::MJPEG);
    videncL->setQuality(50);
    videncR->setQuality(50);
    // videncL->setDefaultProfilePreset(60, dai::VideoEncoderProperties::Profile::MJPEG);
    // videncR->setDefaultProfilePreset(60, dai::VideoEncoderProperties::Profile::MJPEG);

    // Linking

    left->out.link(videncL->input);
    right->out.link(videncR->input);
    videncL->bitstream.link(decoder->inputLeft);
    videncR->bitstream.link(decoder->inputRight);

    decoder->outputImg.link(odom->inputStereo);


    // left->out.link(sync->inputs["left"]);
    // right->out.link(sync->inputs["right"]);
    // sync->out.link(odom->inputStereo);
    imu->out.link(odom->inputImu);
    odom->transform.link(rerun->inputTrans);
    odom->passthrough.link(rerun->inputImg);

    pipeline.start();
    pipeline.wait();
}
