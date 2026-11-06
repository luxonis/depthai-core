#include <XLink/XLinkPlatform.h>

#include <catch2/catch_all.hpp>
#include <cstring>
#include <depthai/utility/OffsetMemory.hpp>
#include <depthai/utility/VectorMemory.hpp>
#include <limits>
#include <utility>

// Include depthai library
#include <depthai/depthai.hpp>
#include <depthai/pipeline/datatype/StreamMessageParser.hpp>

// TODO(themarpe) - fuzz me instead

constexpr auto MARKER_SIZE = 16;

TEST_CASE("Correct message") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    auto des = dai::StreamMessageParser::parseMessage(&packet);
    auto ser2 = dai::StreamMessageParser::serializeMetadata(des);

    REQUIRE(ser == ser2);
}

TEST_CASE("Incorrect message bad size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 100;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Incorrect message negative size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 200;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Incorrect message too small size") {
    std::vector<uint8_t> ser = {0, 1, 2};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Incorrect message too small size 2") {
    std::vector<uint8_t> ser = {0, 1, 1};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Correct message") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    auto des = dai::StreamMessageParser::parseMessage(&packet);
    auto ser2 = dai::StreamMessageParser::serializeMetadata(des);

    REQUIRE(ser == ser2);
}

TEST_CASE("Raw - Incorrect message bad size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 100;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Incorrect message negative size") {
    dai::ImgFrame frm;
    auto ser = dai::StreamMessageParser::serializeMetadata(frm);

    // wreak havoc on serialized data
    ser[ser.size() - 1 - MARKER_SIZE] = 200;

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Incorrect message too small size") {
    std::vector<uint8_t> ser = {0, 1, 2};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

TEST_CASE("Raw - Incorrect message too small size 2") {
    std::vector<uint8_t> ser = {0, 1, 1};

    streamPacketDesc_t packet;
    packet.data = ser.data();
    packet.length = ser.size();

    REQUIRE_THROWS(dai::StreamMessageParser::parseMessage(&packet));
}

#ifndef DEPTHAI_HAVE_BETA
TEST_CASE("Beta parser Config has a clear error when beta support is disabled") {
    dai::Buffer buffer;
    auto serialized = dai::StreamMessageParser::serializeMetadata(buffer);

    constexpr std::size_t TRAILER_SIZE = 8 + MARKER_SIZE;
    REQUIRE(serialized.size() >= TRAILER_SIZE);
    const auto datatype = static_cast<std::uint32_t>(dai::DatatypeEnum::FastSAMParserConfig);
    const auto datatypeOffset = serialized.size() - TRAILER_SIZE;
    serialized[datatypeOffset] = static_cast<std::uint8_t>(datatype);
    serialized[datatypeOffset + 1] = static_cast<std::uint8_t>(datatype >> 8U);
    serialized[datatypeOffset + 2] = static_cast<std::uint8_t>(datatype >> 16U);
    serialized[datatypeOffset + 3] = static_cast<std::uint8_t>(datatype >> 24U);

    streamPacketDesc_t packet{};
    packet.data = serialized.data();
    packet.length = serialized.size();
    packet.fd = -1;

    REQUIRE_THROWS_WITH(dai::StreamMessageParser::parseMessage(&packet),
                        Catch::Matchers::ContainsSubstring("Cannot parse beta datatype: depthai-core was built without beta support"));
}
#endif

TEST_CASE("Owning parser retains the receive allocation without copying its payload") {
    dai::Buffer source;
    source.setData({0x12, 0x34, 0x56});
    source.setSequenceNum(42);
    auto bytes = source.getData();
    auto metadata = dai::StreamMessageParser::serializeMetadata(source);
    dai::StreamPacketDesc packet;
    packet.length = bytes.size() + metadata.size();
    packet.data = static_cast<uint8_t*>(XLinkPlatformAllocateData((packet.length + 63U) & ~63U, 64));
    REQUIRE(packet.data != nullptr);
    std::memcpy(packet.data, bytes.data(), bytes.size());
    std::memcpy(packet.data + bytes.size(), metadata.data(), metadata.size());
    auto* receivedAllocation = packet.data;

    auto parsed = std::dynamic_pointer_cast<dai::Buffer>(dai::StreamMessageParser::parseMessage(std::move(packet)));
    REQUIRE(parsed);
    REQUIRE(packet.data == nullptr);
    REQUIRE(parsed->getData().data() == receivedAllocation);
    REQUIRE(parsed->getData().size() == 3);
    REQUIRE(parsed->getSequenceNum() == 42);
    auto retained = parsed->data;
    std::weak_ptr<dai::Memory> lifetime = retained;
    parsed.reset();
    REQUIRE(std::vector<uint8_t>(retained->getData().begin(), retained->getData().end()) == std::vector<uint8_t>{0x12, 0x34, 0x56});
    retained.reset();
    REQUIRE(lifetime.expired());
}

TEST_CASE("Borrowed parser keeps an independent payload after the packet changes") {
    dai::Buffer source;
    source.setData({0x12, 0x34, 0x56});
    auto bytes = source.getData();
    std::vector<uint8_t> wire(bytes.begin(), bytes.end());
    auto metadata = dai::StreamMessageParser::serializeMetadata(source);
    wire.insert(wire.end(), metadata.begin(), metadata.end());
    streamPacketDesc_t packet{};
    packet.data = wire.data();
    packet.length = wire.size();
    packet.fd = -1;
    auto parsed = std::dynamic_pointer_cast<dai::Buffer>(dai::StreamMessageParser::parseMessage(&packet));
    REQUIRE(parsed);
    REQUIRE(parsed->getData().data() != wire.data());
    wire[0] = 0xFF;
    REQUIRE(std::vector<uint8_t>(parsed->getData().begin(), parsed->getData().end()) == std::vector<uint8_t>{0x12, 0x34, 0x56});
}

namespace {
class SeparateMemory final : public dai::Memory {
    std::vector<uint8_t> first;
    std::vector<uint8_t> second;

   public:
    SeparateMemory(std::vector<uint8_t> first, std::vector<uint8_t> second) : first(std::move(first)), second(std::move(second)) {}
    dai::span<uint8_t> getData() override {
        throw std::logic_error("Separate allocations have no flat payload");
    }
    dai::span<const uint8_t> getData() const override {
        throw std::logic_error("Separate allocations have no flat payload");
    }
    size_t getSize() const override {
        return first.size() + second.size();
    }
    size_t getMaxSize() const override {
        return getSize();
    }
    size_t getOffset() const override {
        return 0;
    }
    void setSize(size_t size) override {
        if(size != getSize()) throw std::logic_error("Published allocations cannot resize");
    }
    dai::span<const uint8_t> getDataRange(size_t offset, size_t length) const override {
        const auto& storage = offset < first.size() ? first : second;
        if(offset >= first.size()) offset -= first.size();
        if(offset > storage.size() || length > storage.size() - offset) throw std::out_of_range("Range crosses storage boundaries");
        return {storage.data() + offset, length};
    }
    std::vector<dai::span<const uint8_t>> getDataSegments() const override {
        return {{first.data(), first.size()}, {second.data(), second.size()}};
    }
};

}  // namespace

TEST_CASE("Flat Memory range access aliases storage and checks bounds") {
    dai::VectorMemory storage(std::vector<uint8_t>{10, 20, 30});
    auto range = storage.getDataRange(1, 2);
    range[0] = 42;
    REQUIRE(storage[1] == 42);
    REQUIRE(range.data() == storage.data() + 1);
    REQUIRE(std::as_const(storage).getDataRange(1, 2)[1] == 30);
    REQUIRE(storage.getDataRange(3, 0).empty());
    REQUIRE_THROWS_AS(storage.getDataRange(3, 1), std::out_of_range);
    REQUIRE_THROWS_AS(storage.getDataRange(std::numeric_limits<size_t>::max(), 2), std::out_of_range);
    const auto segments = std::as_const(storage).getDataSegments();
    REQUIRE(segments.size() == 1);
    REQUIRE(segments[0].data() == storage.data());
    REQUIRE(segments[0].size() == 3);
}

TEST_CASE("Offset memory adapters preserve separate allocation access") {
    auto memory = std::make_shared<SeparateMemory>(std::vector<uint8_t>{1, 2, 3}, std::vector<uint8_t>{9, 8});
    auto adapted = dai::ConvertedOffsetMemory::convert(memory);
    adapted->setOffset(2);
    REQUIRE(adapted->getOffset() == 2);
    REQUIRE(adapted->getSize() == 5);
    REQUIRE(adapted->getOffsetSize() == 3);
    REQUIRE_THROWS_AS(std::as_const(*adapted).getOffsetData(), std::out_of_range);
    adapted->setOffset(1);
    REQUIRE(std::as_const(*adapted).getOffsetData()[0] == 9);
    const auto range = std::as_const(*adapted).getDataRange(3, 2);
    REQUIRE(range[0] == 9);
    REQUIRE(range.data() == std::as_const(*memory).getDataRange(3, 2).data());
    const auto segments = std::as_const(*adapted).getDataSegments();
    REQUIRE(segments.size() == 2);
    REQUIRE(segments[0].data() == std::as_const(*memory).getDataRange(0, 3).data());
    REQUIRE(segments[1].data() == range.data());
    REQUIRE_THROWS_AS(adapted->getData(), std::logic_error);
}

TEST_CASE("Received payload replacement preserves the exact new byte count") {
    dai::Buffer source;
    source.setData({0x12, 0x34, 0x56});
    auto metadata = dai::StreamMessageParser::serializeMetadata(source);
    dai::StreamPacketDesc packet;
    packet.length = 3 + metadata.size();
    packet.data = static_cast<uint8_t*>(XLinkPlatformAllocateData((packet.length + 63U) & ~63U, 64));
    REQUIRE(packet.data != nullptr);
    std::memcpy(packet.data, source.getData().data(), 3);
    std::memcpy(packet.data + 3, metadata.data(), metadata.size());
    auto parsed = std::dynamic_pointer_cast<dai::Buffer>(dai::StreamMessageParser::parseMessage(std::move(packet)));
    REQUIRE(parsed);
    const std::vector<uint8_t> larger{1, 2, 3, 4, 5, 6, 7, 8};
    parsed->setData(larger);
    REQUIRE(std::vector<uint8_t>(parsed->getData().begin(), parsed->getData().end()) == larger);
    const std::vector<uint8_t> smaller{9, 8};
    parsed->setData(smaller);
    REQUIRE(std::vector<uint8_t>(parsed->getData().begin(), parsed->getData().end()) == smaller);
}
