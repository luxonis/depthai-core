// Includes common necessary includes for development using depthai library
#include <google/protobuf/descriptor.pb.h>
#include <google/protobuf/util/time_util.h>

#include <foxglove/websocket/base64.hpp>
#include <foxglove/websocket/server_factory.hpp>
#include <foxglove/websocket/websocket_notls.hpp>
#include <foxglove/websocket/websocket_server.hpp>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/datatype/EncodedFrame.hpp"
#include "depthai/schemas/daischemas/SceneUpdate.pb.h"
#include "depthai/schemas/daischemas//CompressedVideo.pb.h"

static uint64_t nanosecondsSinceEpoch() {
    return uint64_t(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

// Writes the FileDescriptor of this descriptor and all transitive dependencies
// to a string, for use as a channel schema.
static std::string SerializeFdSet(const google::protobuf::Descriptor* toplevelDescriptor) {
    google::protobuf::FileDescriptorSet fdSet;
    std::queue<const google::protobuf::FileDescriptor*> toAdd;
    toAdd.push(toplevelDescriptor->file());
    std::unordered_set<std::string> seenDependencies;
    while(!toAdd.empty()) {
        const google::protobuf::FileDescriptor* next = toAdd.front();
        toAdd.pop();
        next->CopyTo(fdSet.add_file());
        for(int i = 0; i < next->dependency_count(); ++i) {
            const auto& dep = next->dependency(i);
            if(seenDependencies.find(dep->name()) == seenDependencies.end()) {
                seenDependencies.insert(dep->name());
                toAdd.push(dep);
            }
        }
    }
    return fdSet.SerializeAsString();
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline(true);

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    camRgb->video.link(videoEnc->input);
    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setVideoSize(1920, 1080);

    videoEnc->setDefaultProfilePreset(1920, 1080, 30, dai::VideoEncoderProperties::Profile::H264_MAIN);
    auto videoEncQueue = videoEnc->out.createOutputQueue();
    auto outputQueue = camRgb->video.createOutputQueue();
    const auto logHandler = [](foxglove::WebSocketLogLevel, char const* msg) { std::cout << msg << std::endl; };
    foxglove::ServerOptions serverOptions;
    auto server = foxglove::ServerFactory::createServer<websocketpp::connection_hdl>("C++ Protobuf example server", logHandler, serverOptions);
    foxglove::ServerHandlers<foxglove::ConnHandle> hdlrs;
    hdlrs.subscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " subscribed to " << chanId << std::endl;
    };
    hdlrs.unsubscribeHandler = [&](foxglove::ChannelId chanId, foxglove::ConnHandle clientHandle) {
        const auto clientStr = server->remoteEndpointString(clientHandle);
        std::cout << "Client " << clientStr << " unsubscribed from " << chanId << std::endl;
    };

    server->setHandlers(std::move(hdlrs));
    server->start("0.0.0.0", 8765);

    const auto channelIds = server->addChannels({
        {
            .topic = "example_msg",
            .encoding = "protobuf",
            .schemaName = dai::schemas::SceneUpdate::descriptor()->full_name(),
            .schema = foxglove::base64Encode(SerializeFdSet(dai::schemas::SceneUpdate::descriptor())),
        },
        {
            .topic = "example_video",
            .encoding = "protobuf",
            .schemaName = dai::schemas::CompressedVideo::descriptor()->full_name(),
            .schema = foxglove::base64Encode(SerializeFdSet(dai::schemas::CompressedVideo::descriptor())),
        },});
    const auto chanId = channelIds.front();
    const auto chanIdVideo = channelIds.back();
    std::cout << "Channel ID: " << chanId << std::endl;
    std::cout << "Channel ID Video: " << chanIdVideo << std::endl;
    pipeline.start();
    while(pipeline.isRunning()) {
        auto videoIn = outputQueue->get<dai::ImgFrame>();
        auto encodedIn = videoEncQueue->get<dai::EncodedFrame>();
        if(encodedIn) {
            // Get BGR frame from NV12 encoded video frame to show with opencv
            // Visualizing the frame on slower hosts might have overhead
            std::cout << "Received encoded frame: " << encodedIn->getData().size() << " bytes" << std::endl;
        }
        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::imshow("video", videoIn->getCvFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            pipeline.stop();
        }
        const auto now = nanosecondsSinceEpoch();
        dai::schemas::SceneUpdate msg;
        auto* entity = msg.add_entities();
        *entity->mutable_timestamp() = google::protobuf::util::TimeUtil::NanosecondsToTimestamp(now);
        entity->set_frame_id("root");
        auto* cube = entity->add_cubes();
        auto* size = cube->mutable_size();
        size->set_x(1);
        size->set_y(1);
        size->set_z(1);
        auto* position = cube->mutable_pose()->mutable_position();
        position->set_x(2);
        position->set_y(0);
        position->set_z(0);
        auto* orientation = cube->mutable_pose()->mutable_orientation();
        // setAxisAngle(orientation, 0, 0, 1, double(now) / 1e9 * 0.5);
        auto* color = cube->mutable_color();
        color->set_r(0.6);
        color->set_g(0.2);
        color->set_b(1);
        color->set_a(1);

        const auto serializedMsg = msg.SerializeAsString();

        dai::schemas::CompressedVideo videoMsg;
        // videoMsg.set_data(std::span<unsigned char>(reinterpret_cast<unsigned char*>(encodedIn->getData().data()), encodedIn->getData().size()));
        videoMsg.set_data(std::string(reinterpret_cast<const char*>(encodedIn->getData().data()), encodedIn->getData().size()));

        videoMsg.set_format("h264");
        videoMsg.set_frame_id("root");
        videoMsg.mutable_timestamp()->CopyFrom(google::protobuf::util::TimeUtil::NanosecondsToTimestamp(now));
        const auto serializedVideoMsg = videoMsg.SerializeAsString();
        server->broadcastMessage(chanIdVideo, now, reinterpret_cast<const uint8_t*>(serializedVideoMsg.data()), serializedVideoMsg.size());
        server->broadcastMessage(chanId, now, reinterpret_cast<const uint8_t*>(serializedMsg.data()), serializedMsg.size());
    }
    return 0;
}
