// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "depthai/utility/AudioHelpers.hpp"
#include "depthai/pipeline/InputQueue.hpp"

#include <iostream>
#include <chrono>

int main() {
    // Create pipeline
    dai::Pipeline pipeline;
/*
    auto in = pipeline.create<dai::node::AudioIn>();
    in->setDeviceName("microphone");
    in->setDevicePath("hw:0");
    in->setBitrate(44100);
    in->setFps(10);
    in->setChannels(1);

    auto out = pipeline.create<dai::node::AudioOut>();
    out->setDeviceName("speaker");
    out->setDevicePath("hw:0");
    out->setBitrate(44100);
    out->setFps(10);
    out->setChannels(1);*/

    auto replay = pipeline.create<dai::node::AudioReplay>();
    replay->setSourceFile("/tmp/test.wav");
    replay->setLoop(true);
    replay->setFps(10); 

    auto inHost = pipeline.create<dai::node::AudioIn>();
    inHost->setRunOnHost(true);
    inHost->setDeviceName("microphone");
    inHost->setDevicePath("default");
    inHost->setBitrate(44100);
    inHost->setFps(10);
    inHost->setChannels(1);
    inHost->setFormat(SF_FORMAT_PCM_16);//replay->getFormat());

    auto outHost = pipeline.create<dai::node::AudioOut>();
    outHost->setRunOnHost(true);
    outHost->setDeviceName("speaker");
    outHost->setDevicePath("default");
    outHost->setBitrate(44100);
    outHost->setFps(10);
    outHost->setChannels(1);
    outHost->setFormat(SF_FORMAT_PCM_16);//replay->getFormat());

    auto mixer = pipeline.create<dai::node::AudioMixer>();
    mixer->setRunOnHost(true);
//    mixer->setBitrate(44100);
//    mixer->setFps(10);
//    mixer->setChannels(1);
//    mixer->setFormat(replay->getFormat());



//    inHost->out.link(outHost->input);
//    replay->out.link(outHost->input);
//    auto outputQueue = replay->out.createOutputQueue();

    mixer->registerSource("source1", 2);
    mixer->registerSource("source2", 4);
    mixer->registerSink("sink1", SF_FORMAT_PCM_16);

    mixer->linkSourceToSink("source1", "sink1");
    mixer->linkSourceToSink("source2", "sink1");

    replay->out.link(mixer->inputs["source1"]);
    inHost->out.link(mixer->inputs["source2"]);

    mixer->outputs["sink1"].link(outHost->input);

    /*
    auto devVec = dai::audio::ListAlsaDevices();
    for (auto dev : devVec) {
	std::cout << dev.name << std::endl << dev.desc << std::endl << dev.ioid << std::endl << std::endl;
    }*/
//    in->out.link(out->input);
//    replay->out.link(out->input);


//    std::cout << "Starting pipeline" << std::endl;

    pipeline.start();
//    auto t_start = std::chrono::high_resolution_clock::now();

    while(pipeline.isRunning()) {
/*	    auto buf = outputQueue->tryGet<dai::Buffer>();
	    if (buf) {
		    std::cout << "Got audio buffer of size " << buf->getData().size() << std::endl;
		    if( buf->getData().size() == 0) {
			    break;
		    }
	    }*/
    }
/*
    auto t_end = std::chrono::high_resolution_clock::now();
    double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end-t_start).count();

    std::cout << "Took " << elapsed_time_ms << "ms" << std::endl;*/
    return 0;
}
