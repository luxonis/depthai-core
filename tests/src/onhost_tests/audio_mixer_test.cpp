#include <catch2/catch_all.hpp>

// Include depthai library
#include <depthai/depthai.hpp>
#include <depthai/utility/AudioHelpers.hpp>
#include <depthai/pipeline/InputQueue.hpp>

TEST_CASE("AudioMixer") {
    dai::Pipeline pipeline;
    std::shared_ptr<dai::AudioFrame> inputFrame = std::make_shared<dai::AudioFrame>(2, 48000, 2, SF_FORMAT_PCM_32);
    std::vector<uint8_t> frameData((32 / 8) * 2 * 2);
    inputFrame->setData(frameData);

    auto mixer= pipeline.create<dai::node::AudioMixer>();
    mixer->setRunOnHost(true);
    mixer->registerSource("source1", 1);
    mixer->registerSink("sink1", 48000, 2, SF_FORMAT_PCM_32);
    mixer->linkSourceToSink("source1", "sink1");

    auto inputQueue = mixer->inputs["source1"].createInputQueue();
    auto outputQueue = mixer->outputs["sink1"].createOutputQueue();
    
    pipeline.start();

    inputQueue->send(inputFrame);

    std::shared_ptr<dai::AudioFrame> outputFrame = outputQueue->get<dai::AudioFrame>();
    REQUIRE(outputFrame->getFrames() == 2);
    REQUIRE(outputFrame->getBitrate() == 48000);
    REQUIRE(outputFrame->getChannels() == 2);
    REQUIRE(outputFrame->getFormat() == SF_FORMAT_PCM_32);

    pipeline.stop();
}

