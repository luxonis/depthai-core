#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>
#include <depthai/utility/AudioHelpers.hpp>
#include <depthai/pipeline/InputQueue.hpp>

TEST_CASE("AudioEncoder") {
    dai::Pipeline pipeline;
    std::shared_ptr<dai::AudioFrame> inputFrame = std::make_shared<dai::AudioFrame>(2, 24000, 2, SF_FORMAT_PCM_32);
    std::vector<uint8_t> frameData((32 / 8) * 2 * 2);
    inputFrame->setData(frameData);

    auto encoder = pipeline.create<dai::node::AudioEncoder>();
    encoder->setRunOnHost(true);
    encoder->setFormat(SF_FORMAT_PCM_16);
    encoder->setBitrate(48000);
    encoder->setChannels(2);

    auto inputQueue = encoder->input.createInputQueue();
    auto outputQueue = encoder->out.createOutputQueue();
    
    pipeline.start();

    inputQueue->send(inputFrame);

    std::shared_ptr<dai::AudioFrame> outputFrame = outputQueue->get<dai::AudioFrame>();
    REQUIRE(outputFrame->getFrames() == 4);
    REQUIRE(outputFrame->getBitrate() == 48000);
    REQUIRE(outputFrame->getChannels() == 2);
    REQUIRE(outputFrame->getFormat() == SF_FORMAT_PCM_16);
    
    pipeline.stop();
}
