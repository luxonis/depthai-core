// Direct device-to-device link nodes (PeerLinkOut, PeerLinkIn) in the RVC4 firmware, with one device: the host
// plays the other device, or the device links to itself. Needs an RVC4 reached over Ethernet.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <catch2/catch_all.hpp>
#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "depthai/pipeline/node/internal/PeerLinkIn.hpp"
#include "depthai/pipeline/node/internal/PeerLinkOut.hpp"
#include "peerlink/PeerLinkClient.hpp"
#include "peerlink/PeerLinkListener.hpp"
#include "peerlink/PeerLinkProtocol.hpp"

using namespace std::chrono_literals;

namespace {

constexpr std::uint32_t WIDTH = 640;
constexpr std::uint32_t HEIGHT = 400;
constexpr float FPS = 30.0f;

dai::peerlink::Token testToken() {
    dai::peerlink::Token token{};
    for(std::size_t i = 0; i < token.size(); i++) token[i] = static_cast<std::uint8_t>(0xA0 + i);
    return token;
}

// The device's IP when the host reaches it over TCP/IP; the test needs it to reach the device's listener
std::optional<std::string> deviceAddress(const std::shared_ptr<dai::Device>& device) {
    const auto info = device->getDeviceInfo();
    if(device->getPlatform() != dai::Platform::RVC4 || info.protocol != X_LINK_TCP_IP) return std::nullopt;
    return info.name;
}

// The host's address on the interface that reaches `deviceIp`
std::string hostAddressTowards(const std::string& deviceIp) {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(fd >= 0);
    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(9);
    REQUIRE(inet_pton(AF_INET, deviceIp.c_str(), &remote.sin_addr) == 1);
    REQUIRE(::connect(fd, reinterpret_cast<sockaddr*>(&remote), sizeof(remote)) == 0);
    sockaddr_in local{};
    socklen_t length = sizeof(local);
    REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr*>(&local), &length) == 0);
    ::close(fd);
    char buffer[INET_ADDRSTRLEN] = {};
    inet_ntop(AF_INET, &local.sin_addr, buffer, sizeof(buffer));
    return buffer;
}

std::shared_ptr<dai::ImgFrame> makeNv12Frame(std::int64_t sequenceNum) {
    auto frame = std::make_shared<dai::ImgFrame>();
    std::vector<std::uint8_t> data(WIDTH * HEIGHT * 3 / 2);
    for(std::size_t i = 0; i < data.size(); i++) data[i] = static_cast<std::uint8_t>((i + sequenceNum) & 0xFF);
    frame->setData(std::move(data));
    frame->setWidth(WIDTH);
    frame->setHeight(HEIGHT);
    frame->setStride(WIDTH);
    frame->setType(dai::ImgFrame::Type::NV12);
    frame->setSequenceNum(sequenceNum);
    frame->setTimestamp(std::chrono::steady_clock::now());
    return frame;
}

// Receives one message, or nullptr after `timeout` (then the receiver is stopped for good), so a broken link
// fails the test instead of hanging it
std::shared_ptr<dai::ADatatype> receiveWithin(dai::peerlink::PeerLinkReceiver& receiver, std::chrono::milliseconds timeout) {
    auto result = std::async(std::launch::async, [&receiver]() { return receiver.receive(); });
    if(result.wait_for(timeout) != std::future_status::ready) {
        receiver.stop();
    }
    return result.get();
}

bool waitForConsumer(const std::shared_ptr<dai::peerlink::PeerLinkStream>& stream, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline) {
        if(stream->hasConsumer()) return true;
        std::this_thread::sleep_for(10ms);
    }
    return false;
}

}  // namespace

TEST_CASE("Device PeerLinkOut streams camera frames to a peer-link consumer") {
    dai::Pipeline pipeline;
    const auto address = deviceAddress(pipeline.getDefaultDevice());
    if(!address) SKIP("Needs an RVC4 device connected over Ethernet");

    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto* cameraOut = camera->requestOutput({WIDTH, HEIGHT}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS);
    auto peerLinkOut = pipeline.create<dai::node::internal::PeerLinkOut>();
    peerLinkOut->setStreamName("__p_device_out");
    peerLinkOut->properties.token = testToken();
    cameraOut->link(peerLinkOut->input);
    pipeline.start();

    dai::peerlink::PeerLinkReceiver::Config config;
    config.client.endpoints = {dai::peerlink::formatEndpoint(*address, dai::peerlink::PEERLINK_DEFAULT_PORT)};
    config.client.streamName = "__p_device_out";
    config.client.token = testToken();
    dai::peerlink::PeerLinkReceiver receiver(config);

    // First frame, connection included
    auto first = std::dynamic_pointer_cast<dai::ImgFrame>(receiveWithin(receiver, 15s));
    REQUIRE(first != nullptr);

    constexpr int FRAMES = 60;
    const auto start = std::chrono::steady_clock::now();
    for(int i = 0; i < FRAMES; i++) {
        auto frame = std::dynamic_pointer_cast<dai::ImgFrame>(receiveWithin(receiver, 2s));
        REQUIRE(frame != nullptr);
        REQUIRE(frame->getWidth() == WIDTH);
        REQUIRE(frame->getHeight() == HEIGHT);
        REQUIRE(frame->getType() == dai::ImgFrame::Type::NV12);
        REQUIRE(frame->getData().size() >= WIDTH * HEIGHT * 3 / 2);
    }
    const double fps = FRAMES / std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    CAPTURE(fps);
    REQUIRE(fps > FPS * 0.8);

    // A token of another pipeline is refused
    auto wrongToken = config.client;
    wrongToken.token[0] ^= 0xFF;
    dai::peerlink::PeerLinkClient stranger(wrongToken);
    REQUIRE(stranger.connect() == dai::peerlink::ConnectStatus::BAD_TOKEN);

    // A restarted consumer takes over the stream
    receiver.stop();
    dai::peerlink::PeerLinkReceiver second(config);
    REQUIRE(std::dynamic_pointer_cast<dai::ImgFrame>(receiveWithin(second, 5s)) != nullptr);
    REQUIRE(second.getConnectionCount() == 1);

    // The listener answers probes with the device's listener id
    const auto listenerId = dai::peerlink::probe(*address, dai::peerlink::PEERLINK_DEFAULT_PORT, 1000ms);
    REQUIRE(listenerId.has_value());
    REQUIRE_FALSE(listenerId->empty());
}

TEST_CASE("Device PeerLinkIn feeds frames from a peer-link producer into ImageManip") {
    dai::Pipeline pipeline;
    const auto address = deviceAddress(pipeline.getDefaultDevice());
    if(!address) SKIP("Needs an RVC4 device connected over Ethernet");

    dai::peerlink::PeerLinkListener::Config listenerConfig;
    listenerConfig.port = 0;
    listenerConfig.listenerId = "host";
    auto listener = std::make_unique<dai::peerlink::PeerLinkListener>(listenerConfig);
    listener->start();
    const auto port = listener->getPort();
    auto stream = listener->registerStream("__p_device_in", testToken());

    auto peerLinkIn = pipeline.create<dai::node::internal::PeerLinkIn>();
    peerLinkIn->setStreamName("__p_device_in");
    peerLinkIn->properties.token = testToken();
    peerLinkIn->properties.reconnectIntervalMs = 200;
    peerLinkIn->setProducerEndpoints({dai::peerlink::formatEndpoint(hostAddressTowards(*address), port)});
    // NV12 into a hardware node: the frames have to be converted into device memory on receive
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig->setOutputSize(WIDTH / 2, HEIGHT / 2);
    manip->initialConfig->setFrameType(dai::ImgFrame::Type::NV12);
    peerLinkIn->out.link(manip->inputImage);
    auto output = manip->out.createOutputQueue(8, false);
    pipeline.start();

    REQUIRE(waitForConsumer(stream, 15s));
    auto sendAndReceive = [&](int count, std::int64_t firstSequenceNum) {
        int received = 0;
        for(int i = 0; i < count; i++) {
            stream->send(makeNv12Frame(firstSequenceNum + i));
            std::this_thread::sleep_for(33ms);
            while(auto frame = output->tryGet<dai::ImgFrame>()) {
                REQUIRE(frame->getWidth() == WIDTH / 2);
                REQUIRE(frame->getHeight() == HEIGHT / 2);
                REQUIRE(frame->getSequenceNum() >= firstSequenceNum);
                received++;
            }
        }
        bool timedOut = false;
        while(auto frame = output->get<dai::ImgFrame>(500ms, timedOut)) {
            if(timedOut) break;
            received++;
        }
        return received;
    };
    REQUIRE(sendAndReceive(60, 0) >= 50);
    REQUIRE(stream->getStats().dropped == 0);

    // The producer restarts on the same port; the device reconnects by itself
    listener.reset();
    std::this_thread::sleep_for(1s);
    listenerConfig.port = port;
    listener = std::make_unique<dai::peerlink::PeerLinkListener>(listenerConfig);
    listener->start();
    stream = listener->registerStream("__p_device_in", testToken());
    REQUIRE(waitForConsumer(stream, 15s));
    REQUIRE(sendAndReceive(30, 1000) >= 25);
}

TEST_CASE("Device PeerLinkIn skips an address where another device answers") {
    dai::Pipeline pipeline;
    const auto address = deviceAddress(pipeline.getDefaultDevice());
    if(!address) SKIP("Needs an RVC4 device connected over Ethernet");
    const auto hostAddress = hostAddressTowards(*address);

    // The producer's first address now belongs to another device, which runs another pipeline: it answers that it
    // does not know the stream, and without the listener id check the device would retry it for good
    dai::peerlink::PeerLinkListener::Config otherConfig;
    otherConfig.port = 0;
    otherConfig.listenerId = "other-device";
    dai::peerlink::PeerLinkListener other(otherConfig);
    other.start();
    dai::peerlink::PeerLinkListener::Config producerConfig;
    producerConfig.port = 0;
    producerConfig.listenerId = "host";
    dai::peerlink::PeerLinkListener producer(producerConfig);
    producer.start();
    auto stream = producer.registerStream("__p_device_in", testToken());

    auto peerLinkIn = pipeline.create<dai::node::internal::PeerLinkIn>();
    peerLinkIn->setStreamName("__p_device_in");
    peerLinkIn->properties.token = testToken();
    peerLinkIn->properties.producerListenerId = "host";
    peerLinkIn->setProducerEndpoints(
        {dai::peerlink::formatEndpoint(hostAddress, other.getPort()), dai::peerlink::formatEndpoint(hostAddress, producer.getPort())});
    auto output = peerLinkIn->out.createOutputQueue(8, false);
    pipeline.start();

    REQUIRE(waitForConsumer(stream, 15s));
    stream->send(makeNv12Frame(5));
    bool timedOut = false;
    auto frame = output->get<dai::ImgFrame>(5s, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(frame->getSequenceNum() == 5);
}

TEST_CASE("Device links to itself through its own peer-link listener") {
    dai::Pipeline pipeline;
    if(!deviceAddress(pipeline.getDefaultDevice())) SKIP("Needs an RVC4 device connected over Ethernet");

    auto camera = pipeline.create<dai::node::Camera>()->build();
    auto* cameraOut = camera->requestOutput({WIDTH, HEIGHT}, dai::ImgFrame::Type::NV12, dai::ImgResizeMode::CROP, FPS);
    auto peerLinkOut = pipeline.create<dai::node::internal::PeerLinkOut>();
    peerLinkOut->setStreamName("__p_loopback");
    peerLinkOut->properties.token = testToken();
    cameraOut->link(peerLinkOut->input);

    auto peerLinkIn = pipeline.create<dai::node::internal::PeerLinkIn>();
    peerLinkIn->setStreamName("__p_loopback");
    peerLinkIn->properties.token = testToken();
    peerLinkIn->setProducerEndpoints({"127.0.0.1:" + std::to_string(dai::peerlink::PEERLINK_DEFAULT_PORT)});
    auto manip = pipeline.create<dai::node::ImageManip>();
    manip->initialConfig->setOutputSize(WIDTH / 2, HEIGHT / 2);
    peerLinkIn->out.link(manip->inputImage);
    auto output = manip->out.createOutputQueue(8, false);
    pipeline.start();

    bool timedOut = false;
    auto first = output->get<dai::ImgFrame>(15s, timedOut);
    REQUIRE_FALSE(timedOut);
    REQUIRE(first != nullptr);
    constexpr int FRAMES = 60;
    const auto start = std::chrono::steady_clock::now();
    for(int i = 0; i < FRAMES; i++) {
        auto frame = output->get<dai::ImgFrame>(2s, timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(frame->getWidth() == WIDTH / 2);
        REQUIRE(frame->getHeight() == HEIGHT / 2);
    }
    const double fps = FRAMES / std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    CAPTURE(fps);
    REQUIRE(fps > FPS * 0.8);
}
