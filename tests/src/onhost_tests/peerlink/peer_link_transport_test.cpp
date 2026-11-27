// Peer-link transport (direct device-to-device links) over real loopback sockets: the producer listener and
// streams, the consumer client and the reconnecting receiver that the device firmware runs.

#include <poll.h>
#include <sys/socket.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <catch2/catch_all.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "depthai/pipeline/datatype/Buffer.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai/pipeline/datatype/MessageGroup.hpp"
#include "depthai/pipeline/datatype/NNData.hpp"
#include "peerlink/PeerLinkClient.hpp"
#include "peerlink/PeerLinkListener.hpp"
#include "peerlink/PeerLinkProtocol.hpp"
#include "peerlink/PeerLinkSocket.hpp"

using namespace std::chrono_literals;
using dai::peerlink::ConnectStatus;
using dai::peerlink::PeerLinkClient;
using dai::peerlink::PeerLinkListener;
using dai::peerlink::PeerLinkReceiver;
using dai::peerlink::Token;

namespace {

constexpr const char* LISTENER_ID = "producer-device";

Token makeToken(std::uint8_t seed) {
    Token token{};
    for(std::size_t i = 0; i < token.size(); i++) token[i] = static_cast<std::uint8_t>(seed + i);
    return token;
}

PeerLinkListener::Config listenerConfig(std::uint16_t port = 0) {
    PeerLinkListener::Config config;
    config.bindAddress = "127.0.0.1";
    config.port = port;
    config.listenerId = LISTENER_ID;
    config.heartbeatInterval = 200ms;
    config.sendTimeout = 500ms;
    config.handshakeTimeout = 500ms;
    return config;
}

PeerLinkClient::Config clientConfig(std::uint16_t port, const std::string& stream, const Token& token) {
    PeerLinkClient::Config config;
    config.endpoints = {dai::peerlink::formatEndpoint("127.0.0.1", port)};
    config.streamName = stream;
    config.token = token;
    config.connectTimeout = 500ms;
    config.livenessTimeout = 1000ms;
    return config;
}

std::shared_ptr<dai::Buffer> makeBuffer(std::size_t size, std::int64_t sequenceNum) {
    auto buffer = std::make_shared<dai::Buffer>();
    std::vector<std::uint8_t> data(size);
    for(std::size_t i = 0; i < size; i++) data[i] = static_cast<std::uint8_t>((i * 31 + sequenceNum) & 0xFF);
    buffer->setData(std::move(data));
    buffer->setSequenceNum(sequenceNum);
    return buffer;
}

bool sameBytes(dai::span<const std::uint8_t> a, dai::span<const std::uint8_t> b) {
    return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin());
}

// Waits until the stream has a consumer (the reply is sent before the consumer is attached on the producer side)
bool waitForConsumer(const std::shared_ptr<dai::peerlink::PeerLinkStream>& stream, std::chrono::milliseconds timeout = 2000ms) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline) {
        if(stream->hasConsumer()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return false;
}

}  // namespace

TEST_CASE("Peer-link wire encoding round trips") {
    dai::peerlink::Hello hello;
    hello.flags = dai::peerlink::PEERLINK_HELLO_FLAG_PROBE;
    hello.token = makeToken(7);
    hello.streamName = "__p_12";
    const auto helloBytes = dai::peerlink::encodeHello(hello);
    REQUIRE(helloBytes.size() == dai::peerlink::PEERLINK_HELLO_HEADER_SIZE + hello.streamName.size());
    const auto header = dai::peerlink::decodeHelloHeader(helloBytes.data());
    REQUIRE(header.has_value());
    REQUIRE(header->version == dai::peerlink::PEERLINK_PROTOCOL_VERSION);
    REQUIRE(header->flags == dai::peerlink::PEERLINK_HELLO_FLAG_PROBE);
    REQUIRE(header->token == hello.token);
    REQUIRE(header->nameLength == hello.streamName.size());

    auto corrupted = helloBytes;
    corrupted[0] = 'X';
    REQUIRE_FALSE(dai::peerlink::decodeHelloHeader(corrupted.data()).has_value());

    const auto replyBytes = dai::peerlink::encodeReply({dai::peerlink::ReplyStatus::BAD_TOKEN, 3, "abc"});
    const auto reply = dai::peerlink::decodeReplyHeader(replyBytes.data());
    REQUIRE(reply.status == static_cast<std::uint8_t>(dai::peerlink::ReplyStatus::BAD_TOKEN));
    REQUIRE(reply.version == 3);
    REQUIRE(reply.listenerIdLength == 3);

    const auto frame = dai::peerlink::encodeFrameHeader(dai::peerlink::FrameKind::MESSAGE, 0x0102030405060708ULL);
    const auto frameHeader = dai::peerlink::decodeFrameHeader(frame.data());
    REQUIRE(frameHeader.kind == static_cast<std::uint8_t>(dai::peerlink::FrameKind::MESSAGE));
    REQUIRE(frameHeader.payloadSize == 0x0102030405060708ULL);

    dai::peerlink::Hello longName;
    longName.streamName = std::string(dai::peerlink::PEERLINK_MAX_STREAM_NAME_LENGTH + 1, 'a');
    REQUIRE_THROWS_AS(dai::peerlink::encodeHello(longName), std::invalid_argument);
}

TEST_CASE("Peer-link handshake connects a consumer to a registered stream") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));

    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    std::string detail;
    REQUIRE(client.connect(&detail) == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    auto sent = makeBuffer(1000, 42);
    REQUIRE(stream->send(sent));
    auto received = std::dynamic_pointer_cast<dai::Buffer>(client.receive());
    REQUIRE(received != nullptr);
    REQUIRE(received->getSequenceNum() == 42);
    REQUIRE(sameBytes(received->getData(), sent->getData()));
    REQUIRE(stream->getStats().sent == 1);
    REQUIRE(stream->getStats().connections == 1);
}

TEST_CASE("Peer-link rejects a consumer with another pipeline's token") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));

    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(99)));
    std::string detail;
    REQUIRE(client.connect(&detail) == ConnectStatus::BAD_TOKEN);
    REQUIRE_FALSE(client.isConnected());
    std::this_thread::sleep_for(100ms);
    REQUIRE_FALSE(stream->hasConsumer());
}

TEST_CASE("Peer-link rejects another protocol version") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    listener.registerStream("__p_1", makeToken(1));

    auto socket = dai::peerlink::connectTcp("127.0.0.1", listener.getPort(), 500ms);
    REQUIRE(socket.valid());
    dai::peerlink::setReceiveTimeout(socket, 1000ms);
    dai::peerlink::Hello hello;
    hello.version = dai::peerlink::PEERLINK_PROTOCOL_VERSION + 1;
    hello.token = makeToken(1);
    hello.streamName = "__p_1";
    auto bytes = dai::peerlink::encodeHello(hello);
    iovec buffer{bytes.data(), bytes.size()};
    REQUIRE(dai::peerlink::sendAll(socket, &buffer, 1) == dai::peerlink::IoResult::OK);
    std::array<std::uint8_t, dai::peerlink::PEERLINK_REPLY_HEADER_SIZE> replyBytes{};
    REQUIRE(dai::peerlink::receiveAll(socket, replyBytes.data(), replyBytes.size()) == dai::peerlink::IoResult::OK);
    const auto reply = dai::peerlink::decodeReplyHeader(replyBytes.data());
    REQUIRE(reply.status == static_cast<std::uint8_t>(dai::peerlink::ReplyStatus::BAD_VERSION));
    REQUIRE(reply.version == dai::peerlink::PEERLINK_PROTOCOL_VERSION);
}

TEST_CASE("Peer-link listener ignores a peer that is not a consumer") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    {
        auto socket = dai::peerlink::connectTcp("127.0.0.1", listener.getPort(), 500ms);
        REQUIRE(socket.valid());
        std::vector<std::uint8_t> garbage(dai::peerlink::PEERLINK_HELLO_HEADER_SIZE, 0x55);
        iovec buffer{garbage.data(), garbage.size()};
        REQUIRE(dai::peerlink::sendAll(socket, &buffer, 1) == dai::peerlink::IoResult::OK);
    }
    // The listener still serves real consumers
    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
}

TEST_CASE("Peer-link probe returns the listener id") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    const auto listenerId = dai::peerlink::probe("127.0.0.1", listener.getPort(), 500ms);
    REQUIRE(listenerId.has_value());
    REQUIRE(*listenerId == LISTENER_ID);

    const auto port = listener.getPort();
    listener.stop();
    REQUIRE_FALSE(dai::peerlink::probe("127.0.0.1", port, 300ms).has_value());
}

TEST_CASE("Peer-link consumer retries an unknown stream until the producer registers it") {
    PeerLinkListener listener(listenerConfig());
    listener.start();

    PeerLinkClient probeClient(clientConfig(listener.getPort(), "__p_late", makeToken(3)));
    REQUIRE(probeClient.connect() == ConnectStatus::UNKNOWN_STREAM);

    PeerLinkReceiver::Config config;
    config.client = clientConfig(listener.getPort(), "__p_late", makeToken(3));
    config.minRetryInterval = 20ms;
    config.maxRetryInterval = 100ms;
    PeerLinkReceiver receiver(config);
    std::shared_ptr<dai::ADatatype> received;
    std::thread consumer([&]() { received = receiver.receive(); });

    std::this_thread::sleep_for(300ms);
    REQUIRE_FALSE(receiver.isConnected());
    auto stream = listener.registerStream("__p_late", makeToken(3));
    REQUIRE(waitForConsumer(stream));
    REQUIRE(stream->send(makeBuffer(10, 1)));
    consumer.join();
    REQUIRE(received != nullptr);
    REQUIRE(receiver.getConnectionCount() == 1);
}

TEST_CASE("Peer-link carries a large ImgFrame and NNData byte exact") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    // 4032x3056 NV12 is about 18.5 MB; use 20 MB of data
    constexpr std::size_t FRAME_SIZE = 20 * 1024 * 1024;
    auto frame = std::make_shared<dai::ImgFrame>();
    std::vector<std::uint8_t> pixels(FRAME_SIZE);
    std::iota(pixels.begin(), pixels.end(), static_cast<std::uint8_t>(0));
    frame->setData(pixels);
    frame->setWidth(4096);
    frame->setHeight(3413);
    frame->setType(dai::ImgFrame::Type::NV12);
    frame->setSequenceNum(7);
    const auto timestamp = std::chrono::steady_clock::now();
    frame->setTimestamp(timestamp);

    auto nnData = std::make_shared<dai::NNData>();
#ifdef DEPTHAI_XTENSOR_SUPPORT
    std::vector<float> values(1000);
    std::iota(values.begin(), values.end(), 0.5f);
    nnData->addTensor<float>("scores", values, dai::TensorInfo::DataType::FP32);
#endif
    nnData->setSequenceNum(8);

    std::thread sender([&]() {
        REQUIRE(stream->send(frame));
        REQUIRE(stream->send(nnData));
    });
    auto receivedFrame = std::dynamic_pointer_cast<dai::ImgFrame>(client.receive());
    auto receivedNNData = std::dynamic_pointer_cast<dai::NNData>(client.receive());
    sender.join();

    REQUIRE(receivedFrame != nullptr);
    REQUIRE(receivedFrame->getWidth() == 4096);
    REQUIRE(receivedFrame->getHeight() == 3413);
    REQUIRE(receivedFrame->getType() == dai::ImgFrame::Type::NV12);
    REQUIRE(receivedFrame->getSequenceNum() == 7);
    REQUIRE(receivedFrame->getTimestamp() == frame->getTimestamp());
    REQUIRE(sameBytes(receivedFrame->getData(), frame->getData()));

    REQUIRE(receivedNNData != nullptr);
    REQUIRE(receivedNNData->getSequenceNum() == 8);
    REQUIRE(receivedNNData->getAllLayerNames() == nnData->getAllLayerNames());
    REQUIRE(sameBytes(receivedNNData->getData(), nnData->getData()));
}

TEST_CASE("Peer-link carries a MessageGroup with its parts") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    auto group = std::make_shared<dai::MessageGroup>();
    group->add("a", makeBuffer(100, 1));
    group->add("b", makeBuffer(0, 2));
    group->add("c", makeBuffer(5000, 3));
    group->setSequenceNum(9);
    REQUIRE(stream->send(group));
    // A message after the group must not be mistaken for a part
    REQUIRE(stream->send(makeBuffer(10, 4)));

    auto received = std::dynamic_pointer_cast<dai::MessageGroup>(client.receive());
    REQUIRE(received != nullptr);
    REQUIRE(received->getSequenceNum() == 9);
    REQUIRE(received->getNumMessages() == 3);
    for(const auto& name : {"a", "b", "c"}) {
        auto part = received->get<dai::Buffer>(name);
        REQUIRE(part != nullptr);
        REQUIRE(sameBytes(part->getData(), group->get<dai::Buffer>(name)->getData()));
        REQUIRE(part->getSequenceNum() == group->get<dai::Buffer>(name)->getSequenceNum());
    }
    auto next = std::dynamic_pointer_cast<dai::Buffer>(client.receive());
    REQUIRE(next != nullptr);
    REQUIRE(next->getSequenceNum() == 4);
}

TEST_CASE("Peer-link producer drops messages without a consumer and never waits for one") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));

    const auto start = std::chrono::steady_clock::now();
    for(int i = 0; i < 100; i++) {
        REQUIRE_FALSE(stream->send(makeBuffer(100000, i)));
    }
    REQUIRE(std::chrono::steady_clock::now() - start < 1s);
    REQUIRE(stream->getStats().dropped == 100);
    REQUIRE(stream->getStats().sent == 0);
}

TEST_CASE("Peer-link producer disconnects a consumer that stops reading") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    // Connected, but never reads
    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    // Kernel buffers absorb a few MB, then sends stall; the send timeout (500 ms) disconnects the consumer
    const auto start = std::chrono::steady_clock::now();
    bool dropped = false;
    for(int i = 0; i < 200 && !dropped; i++) {
        dropped = !stream->send(makeBuffer(1024 * 1024, i));
    }
    const auto elapsed = std::chrono::steady_clock::now() - start;
    REQUIRE(dropped);
    REQUIRE_FALSE(stream->hasConsumer());
    REQUIRE(elapsed < 5s);
    // Later messages are dropped without waiting
    const auto afterDisconnect = std::chrono::steady_clock::now();
    REQUIRE_FALSE(stream->send(makeBuffer(1024 * 1024, 1000)));
    REQUIRE(std::chrono::steady_clock::now() - afterDisconnect < 100ms);
}

TEST_CASE("Peer-link heartbeats keep an idle link alive") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    std::thread sender([&]() {
        // Idle for 2.5x the consumer's liveness timeout; heartbeats (every 200 ms) keep it connected
        std::this_thread::sleep_for(2500ms);
        stream->send(makeBuffer(10, 5));
    });
    auto received = client.receive();
    sender.join();
    REQUIRE(received != nullptr);
    REQUIRE(stream->getStats().connections == 1);
}

TEST_CASE("Peer-link consumer detects a silent producer within the liveness timeout") {
    // A fake producer that accepts the consumer and then sends nothing, not even heartbeats
    auto listenSocket = dai::peerlink::listenTcp("127.0.0.1", 0);
    const auto port = dai::peerlink::localPort(listenSocket);
    std::atomic<bool> done{false};
    std::thread producer([&]() {
        dai::peerlink::Socket socket(::accept(listenSocket.get(), nullptr, nullptr));
        std::array<std::uint8_t, dai::peerlink::PEERLINK_HELLO_HEADER_SIZE> header{};
        dai::peerlink::receiveAll(socket, header.data(), header.size());
        std::string name(dai::peerlink::decodeHelloHeader(header.data())->nameLength, '\0');
        dai::peerlink::receiveAll(socket, name.data(), name.size());
        auto reply = dai::peerlink::encodeReply({dai::peerlink::ReplyStatus::OK, dai::peerlink::PEERLINK_PROTOCOL_VERSION, "silent"});
        iovec buffer{reply.data(), reply.size()};
        dai::peerlink::sendAll(socket, &buffer, 1);
        while(!done) std::this_thread::sleep_for(10ms);
    });

    auto config = clientConfig(port, "__p_1", makeToken(1));
    config.livenessTimeout = 1000ms;
    PeerLinkClient client(config);
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
    const auto start = std::chrono::steady_clock::now();
    REQUIRE(client.receive() == nullptr);
    const auto elapsed = std::chrono::steady_clock::now() - start;
    done = true;
    producer.join();
    REQUIRE(elapsed >= 900ms);
    REQUIRE(elapsed < 3s);
    REQUIRE_FALSE(client.isConnected());
}

TEST_CASE("Peer-link newest consumer replaces the previous one") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));

    PeerLinkClient first(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(first.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));
    PeerLinkClient second(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(second.connect() == ConnectStatus::CONNECTED);
    // The second consumer is attached once the stream counted it
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while(stream->getStats().connections < 2 && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(5ms);
    REQUIRE(stream->getStats().connections == 2);

    REQUIRE(stream->send(makeBuffer(10, 1)));
    auto received = second.receive();
    REQUIRE(received != nullptr);
    REQUIRE(first.receive() == nullptr);
}

TEST_CASE("Peer-link receiver reconnects after the producer restarts") {
    auto listener = std::make_unique<PeerLinkListener>(listenerConfig());
    listener->start();
    const auto port = listener->getPort();
    auto stream = listener->registerStream("__p_1", makeToken(1));

    PeerLinkReceiver::Config config;
    config.client = clientConfig(port, "__p_1", makeToken(1));
    config.minRetryInterval = 20ms;
    config.maxRetryInterval = 100ms;
    PeerLinkReceiver receiver(config);

    std::atomic<int> receivedCount{0};
    std::thread consumer([&]() {
        while(receiver.receive() != nullptr) receivedCount++;
    });

    REQUIRE(waitForConsumer(stream));
    REQUIRE(stream->send(makeBuffer(10, 1)));
    // Producer process restarts on the same port, with the same pipeline (token)
    listener.reset();
    listener = std::make_unique<PeerLinkListener>(listenerConfig(port));
    listener->start();
    stream = listener->registerStream("__p_1", makeToken(1));
    REQUIRE(waitForConsumer(stream, 5s));
    REQUIRE(stream->send(makeBuffer(10, 2)));

    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while(receivedCount < 2 && std::chrono::steady_clock::now() < deadline) std::this_thread::sleep_for(5ms);
    receiver.stop();
    consumer.join();
    REQUIRE(receivedCount == 2);
    REQUIRE(receiver.getConnectionCount() == 2);
}

TEST_CASE("Peer-link receiver stops promptly while waiting") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    PeerLinkReceiver::Config config;
    config.client = clientConfig(listener.getPort(), "__p_1", makeToken(1));
    PeerLinkReceiver receiver(config);
    std::shared_ptr<dai::ADatatype> result = std::make_shared<dai::Buffer>();
    std::thread consumer([&]() { result = receiver.receive(); });
    REQUIRE(waitForConsumer(stream));
    const auto start = std::chrono::steady_clock::now();
    receiver.stop();
    consumer.join();
    REQUIRE(result == nullptr);
    REQUIRE(std::chrono::steady_clock::now() - start < 500ms);
}

TEST_CASE("Peer-link serves two streams at once") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto streamA = listener.registerStream("__p_a", makeToken(1));
    auto streamB = listener.registerStream("__p_b", makeToken(1));
    REQUIRE_THROWS_AS(listener.registerStream("__p_a", makeToken(1)), std::runtime_error);

    PeerLinkClient clientA(clientConfig(listener.getPort(), "__p_a", makeToken(1)));
    PeerLinkClient clientB(clientConfig(listener.getPort(), "__p_b", makeToken(1)));
    REQUIRE(clientA.connect() == ConnectStatus::CONNECTED);
    REQUIRE(clientB.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(streamA));
    REQUIRE(waitForConsumer(streamB));

    constexpr int COUNT = 50;
    std::thread senderA([&]() {
        for(int i = 0; i < COUNT; i++) streamA->send(makeBuffer(64 * 1024, i));
    });
    std::thread senderB([&]() {
        for(int i = 0; i < COUNT; i++) streamB->send(makeBuffer(32 * 1024, 1000 + i));
    });
    for(int i = 0; i < COUNT; i++) {
        auto a = std::dynamic_pointer_cast<dai::Buffer>(clientA.receive());
        auto b = std::dynamic_pointer_cast<dai::Buffer>(clientB.receive());
        REQUIRE(a != nullptr);
        REQUIRE(b != nullptr);
        REQUIRE(a->getSequenceNum() == i);
        REQUIRE(b->getSequenceNum() == 1000 + i);
        REQUIRE(a->getData().size() == 64 * 1024);
        REQUIRE(b->getData().size() == 32 * 1024);
    }
    senderA.join();
    senderB.join();

    listener.unregisterStream(streamA);
    REQUIRE(clientA.receive() == nullptr);
    PeerLinkClient again(clientConfig(listener.getPort(), "__p_a", makeToken(1)));
    REQUIRE(again.connect() == ConnectStatus::UNKNOWN_STREAM);
}

TEST_CASE("Peer-link endpoints format and parse") {
    REQUIRE(dai::peerlink::formatEndpoint("10.0.0.5", 11494) == "10.0.0.5:11494");
    REQUIRE(dai::peerlink::formatEndpoint("fe80::1", 11494) == "[fe80::1]:11494");
    const auto v4 = dai::peerlink::parseEndpoint("10.0.0.5:11494");
    REQUIRE(v4.has_value());
    REQUIRE(v4->host == "10.0.0.5");
    REQUIRE(v4->port == 11494);
    const auto v6 = dai::peerlink::parseEndpoint("[fe80::1]:1");
    REQUIRE(v6.has_value());
    REQUIRE(v6->host == "fe80::1");
    REQUIRE(v6->port == 1);
    for(const auto* bad : {"10.0.0.5", "10.0.0.5:", ":11494", "10.0.0.5:0", "10.0.0.5:65536", "10.0.0.5:12a", "fe80::1:5", "[fe80::1]5"}) {
        CAPTURE(bad);
        REQUIRE_FALSE(dai::peerlink::parseEndpoint(bad).has_value());
    }
}

TEST_CASE("Peer-link listener serves probes while a peer stays silent") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    // Connects and never sends a hello; the handshake timeout is 500 ms
    auto silent = dai::peerlink::connectTcp("127.0.0.1", listener.getPort(), 500ms);
    REQUIRE(silent.valid());
    std::this_thread::sleep_for(50ms);
    const auto start = std::chrono::steady_clock::now();
    const auto listenerId = dai::peerlink::probe("127.0.0.1", listener.getPort(), 300ms);
    REQUIRE(listenerId.has_value());
    REQUIRE(std::chrono::steady_clock::now() - start < 250ms);
}

TEST_CASE("Peer-link stalled consumer does not delay heartbeats of another stream") {
    auto config = listenerConfig();
    config.sendTimeout = 3000ms;
    PeerLinkListener listener(config);
    listener.start();
    auto stalled = listener.registerStream("__p_stalled", makeToken(1));
    auto idle = listener.registerStream("__p_idle", makeToken(1));

    PeerLinkClient stalledClient(clientConfig(listener.getPort(), "__p_stalled", makeToken(1)));
    REQUIRE(stalledClient.connect() == ConnectStatus::CONNECTED);
    PeerLinkClient idleClient(clientConfig(listener.getPort(), "__p_idle", makeToken(1)));
    REQUIRE(idleClient.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stalled));
    REQUIRE(waitForConsumer(idle));

    // Fill the stalled consumer's socket: the sender then blocks for up to 3 s per attempt
    std::atomic<bool> stop{false};
    std::thread sender([&]() {
        while(!stop && stalled->send(makeBuffer(1024 * 1024, 0))) {
        }
    });
    // The idle consumer (liveness 1 s) must keep getting heartbeats meanwhile
    std::thread idleSender([&]() {
        std::this_thread::sleep_for(2500ms);
        idle->send(makeBuffer(10, 7));
    });
    auto received = std::dynamic_pointer_cast<dai::Buffer>(idleClient.receive());
    idleSender.join();
    stop = true;
    stalledClient.disconnect();
    sender.join();
    REQUIRE(received != nullptr);
    REQUIRE(received->getSequenceNum() == 7);
    REQUIRE(idle->getStats().connections == 1);
}

TEST_CASE("Peer-link new consumer takes over while a send to a stalled one is in progress") {
    auto config = listenerConfig();
    config.sendTimeout = 10000ms;
    PeerLinkListener listener(config);
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    PeerLinkClient stalledClient(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(stalledClient.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    std::atomic<bool> stop{false};
    std::atomic<int> sentToNew{0};
    std::thread sender([&]() {
        int i = 0;
        while(!stop) {
            if(stream->send(makeBuffer(256 * 1024, i++)) && stream->getStats().connections == 2) sentToNew++;
        }
    });
    // Let the sender block on the stalled consumer's full socket
    std::this_thread::sleep_for(500ms);
    const auto start = std::chrono::steady_clock::now();
    PeerLinkClient newClient(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(newClient.connect() == ConnectStatus::CONNECTED);
    auto received = newClient.receive();
    const auto elapsed = std::chrono::steady_clock::now() - start;
    stop = true;
    newClient.disconnect();
    sender.join();
    REQUIRE(received != nullptr);
    // Far below the 10 s send timeout that the stalled send would otherwise take
    REQUIRE(elapsed < 2s);
}

TEST_CASE("Peer-link consumer skips an address that another device took over") {
    // Another device's listener now answers at the producer's first address, even with a stream of the same
    // name and token (a stale session of the same pipeline)
    auto otherConfig = listenerConfig();
    otherConfig.listenerId = "other-device";
    PeerLinkListener other(otherConfig);
    other.start();
    auto otherStream = other.registerStream("__p_1", makeToken(1));
    // A server that accepts and closes without a reply (handshake broken)
    auto brokenSocket = dai::peerlink::listenTcp("127.0.0.1", 0);
    const auto brokenPort = dai::peerlink::localPort(brokenSocket);
    std::atomic<bool> done{false};
    std::thread broken([&]() {
        while(!done) {
            pollfd pfd{brokenSocket.get(), POLLIN, 0};
            if(::poll(&pfd, 1, 50) > 0) dai::peerlink::Socket accepted(::accept(brokenSocket.get(), nullptr, nullptr));
        }
    });
    PeerLinkListener producer(listenerConfig());
    producer.start();
    auto stream = producer.registerStream("__p_1", makeToken(1));

    auto config = clientConfig(producer.getPort(), "__p_1", makeToken(1));
    config.endpoints = {dai::peerlink::formatEndpoint("127.0.0.1", other.getPort()),
                        dai::peerlink::formatEndpoint("127.0.0.1", brokenPort),
                        dai::peerlink::formatEndpoint("127.0.0.1", producer.getPort())};
    SECTION("with the producer's listener id the consumer moves on to the producer") {
        config.listenerId = LISTENER_ID;
        PeerLinkClient client(config);
        std::string detail;
        REQUIRE(client.connect(&detail) == ConnectStatus::CONNECTED);
        REQUIRE(client.getEndpoint() == config.endpoints[2]);
    }
    SECTION("without it the first listener that answers decides") {
        PeerLinkClient client(config);
        REQUIRE(client.connect() == ConnectStatus::CONNECTED);
        REQUIRE(client.getEndpoint() == config.endpoints[0]);
    }
    done = true;
    broken.join();
}

TEST_CASE("Peer-link probe gives up within its timeout") {
    // An address that routes nowhere: the probe must not take longer than its timeout
    const auto start = std::chrono::steady_clock::now();
    REQUIRE_FALSE(dai::peerlink::probe("10.255.255.1", dai::peerlink::PEERLINK_DEFAULT_PORT, 300ms).has_value());
    REQUIRE(std::chrono::steady_clock::now() - start < 600ms);
    // A name is never looked up (a lookup has no time limit)
    REQUIRE_FALSE(dai::peerlink::probe("localhost", dai::peerlink::PEERLINK_DEFAULT_PORT, 300ms).has_value());
}

TEST_CASE("Peer-link producer drops a MessageGroup with an empty part and keeps the consumer") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    auto group = std::make_shared<dai::MessageGroup>();
    group->add("a", makeBuffer(10, 1));
    group->add("missing", nullptr);
    REQUIRE_FALSE(stream->send(group));
    REQUIRE(stream->hasConsumer());
    REQUIRE(stream->send(makeBuffer(10, 2)));
    auto next = std::dynamic_pointer_cast<dai::Buffer>(client.receive());
    REQUIRE(next != nullptr);
    REQUIRE(next->getSequenceNum() == 2);
}

TEST_CASE("Peer-link receiver follows a producer that comes back at another address") {
    PeerLinkListener listener(listenerConfig());
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));

    // Built for an address that is gone; long retry intervals, so only the endpoint update can make it quick
    PeerLinkReceiver::Config config;
    config.client = clientConfig(1, "__p_1", makeToken(1));
    config.client.endpoints = {"127.0.0.1:1"};
    config.minRetryInterval = 2000ms;
    config.maxRetryInterval = 5000ms;
    PeerLinkReceiver receiver(config);
    std::shared_ptr<dai::ADatatype> received;
    std::thread consumer([&]() { received = receiver.receive(); });
    std::this_thread::sleep_for(300ms);
    REQUIRE_FALSE(receiver.isConnected());

    const auto start = std::chrono::steady_clock::now();
    receiver.setEndpoints({dai::peerlink::formatEndpoint("127.0.0.1", listener.getPort())});
    REQUIRE(waitForConsumer(stream, 1500ms));
    REQUIRE(std::chrono::steady_clock::now() - start < 1s);
    REQUIRE(stream->send(makeBuffer(10, 1)));
    consumer.join();
    REQUIRE(received != nullptr);
}

TEST_CASE("Peer-link unregistering a stream unblocks a send to a stalled consumer") {
    auto config = listenerConfig();
    config.sendTimeout = 10000ms;
    PeerLinkListener listener(config);
    listener.start();
    auto stream = listener.registerStream("__p_1", makeToken(1));
    // Connected, never reads
    PeerLinkClient client(clientConfig(listener.getPort(), "__p_1", makeToken(1)));
    REQUIRE(client.connect() == ConnectStatus::CONNECTED);
    REQUIRE(waitForConsumer(stream));

    std::atomic<bool> blocked{false};
    std::atomic<bool> finished{false};
    std::thread sender([&]() {
        while(stream->send(makeBuffer(1024 * 1024, 0))) {
            blocked = true;
        }
        finished = true;
    });
    // The socket buffers fill within a few MB; then a send blocks for up to the 10 s send timeout
    std::this_thread::sleep_for(1s);
    REQUIRE_FALSE(finished);
    const auto start = std::chrono::steady_clock::now();
    listener.unregisterStream(stream);  // what PeerLinkOut does when the node stops
    sender.join();
    REQUIRE(std::chrono::steady_clock::now() - start < 1s);
    REQUIRE(blocked);
}
