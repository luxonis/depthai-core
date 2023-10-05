#include <catch2/catch_all.hpp>

// Include depthai library
#include <cstdint>
#include <depthai/pipeline/datatypes.hpp>
#include <iostream>

#include "depthai-shared/datatype/DatatypeEnum.hpp"
#include "depthai-shared/datatype/RawMessageGroup.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"

TEST_CASE("Serialization and deserialization - JSON") {
    dai::MessageGroup msg;
    auto buffer = dai::Buffer();
    buffer.setSequenceNum(255);
    auto imgFrame = dai::ImgFrame();
    imgFrame.setSize({4, 5});
    auto stereoConfig = dai::StereoDepthConfig();
    stereoConfig.setConfidenceThreshold(67);
    msg.add("buffer", buffer);
    msg.add("imgFrame", imgFrame);
    msg.add("stereoConfig", stereoConfig);

    // std::cout << (unsigned int)msg.getInternal().group["buffer"].buffer->data[2] << std::endl;

    auto serialized = dai::utility::serialize<dai::SerializationType::JSON>(msg.getInternal());
    dai::RawMessageGroup deserialized;
    dai::utility::deserialize<dai::SerializationType::JSON>(serialized, deserialized);

    // std::cout << std::string(serialized.begin(), serialized.end()) << std::endl;
    REQUIRE(deserialized.group.find("buffer") != deserialized.group.end());
    REQUIRE(deserialized.group.find("imgFrame") != deserialized.group.end());
    REQUIRE(deserialized.group.find("stereoConfig") != deserialized.group.end());

    auto group = deserialized.group;
    REQUIRE(group["buffer"].datatype == dai::DatatypeEnum::Buffer);
    REQUIRE(group["imgFrame"].datatype == dai::DatatypeEnum::ImgFrame);
    REQUIRE(group["stereoConfig"].datatype == dai::DatatypeEnum::StereoDepthConfig);

    auto dBuffer = dai::Buffer(group["buffer"].buffer);
    auto dImgFrame = dai::ImgFrame(std::dynamic_pointer_cast<dai::RawImgFrame>(group["imgFrame"].buffer));
    auto dStereoConfig = dai::StereoDepthConfig(std::dynamic_pointer_cast<dai::RawStereoDepthConfig>(group["stereoConfig"].buffer));

    REQUIRE(dBuffer.getSequenceNum() == buffer.getSequenceNum());
    REQUIRE((dImgFrame.getWidth() == imgFrame.getWidth() && dImgFrame.getHeight() == imgFrame.getHeight()));
    REQUIRE(dStereoConfig.getConfidenceThreshold() == stereoConfig.getConfidenceThreshold());
}

TEST_CASE("Serialization and deserialization - NOP") {
    dai::MessageGroup msg;
    auto buffer = dai::Buffer();
    buffer.setSequenceNum(255);
    auto imgFrame = dai::ImgFrame();
    imgFrame.setSize({4, 5});
    auto stereoConfig = dai::StereoDepthConfig();
    stereoConfig.setConfidenceThreshold(67);
    msg.add("buffer", buffer);
    msg.add("imgFrame", imgFrame);
    msg.add("stereoConfig", stereoConfig);

    // std::cout << (unsigned int)msg.getInternal().group["buffer"].buffer->data[2] << std::endl;

    auto serialized = dai::utility::serialize<dai::SerializationType::LIBNOP>(msg.getInternal());
    dai::RawMessageGroup deserialized;
    dai::utility::deserialize<dai::SerializationType::LIBNOP>(serialized, deserialized);

    // std::cout << std::string(serialized.begin(), serialized.end()) << std::endl;
    REQUIRE(deserialized.group.find("buffer") != deserialized.group.end());
    REQUIRE(deserialized.group.find("imgFrame") != deserialized.group.end());
    REQUIRE(deserialized.group.find("stereoConfig") != deserialized.group.end());

    auto group = deserialized.group;
    REQUIRE(group["buffer"].datatype == dai::DatatypeEnum::Buffer);
    REQUIRE(group["imgFrame"].datatype == dai::DatatypeEnum::ImgFrame);
    REQUIRE(group["stereoConfig"].datatype == dai::DatatypeEnum::StereoDepthConfig);

    auto dBuffer = dai::Buffer(group["buffer"].buffer);
    auto dImgFrame = dai::ImgFrame(std::dynamic_pointer_cast<dai::RawImgFrame>(group["imgFrame"].buffer));
    auto dStereoConfig = dai::StereoDepthConfig(std::dynamic_pointer_cast<dai::RawStereoDepthConfig>(group["stereoConfig"].buffer));

    REQUIRE(dBuffer.getSequenceNum() == buffer.getSequenceNum());
    REQUIRE((dImgFrame.getWidth() == imgFrame.getWidth() && dImgFrame.getHeight() == imgFrame.getHeight()));
    REQUIRE(dStereoConfig.getConfidenceThreshold() == stereoConfig.getConfidenceThreshold());
}
