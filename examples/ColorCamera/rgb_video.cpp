#include <iostream>
#include <fstream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Pass the argument `uvc` to run in UVC mode

int main(int argc, char** argv) {
    bool enableUVC = 0;
    bool enableToF = 0;
    bool enableMic = 0;
    if(argc > 1) {
        if (std::string(argv[1]) == "uvc") {
            enableUVC = 1;
        } else if (std::string(argv[1]) == "tof") {
            enableToF = 1;
        } else if (std::string(argv[1]) == "mic") {
            enableMic = 1;
        } else {
            printf("Unrecognized argument: %s\n", argv[1]);
        }
    }

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and output
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
//    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    camRgb->setIspScale(1, 2);
    camRgb->setVideoSize(1920, 1080);
    camRgb->initialControl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::MAINS_60_HZ);
    camRgb->setFps(30);

    xoutVideo->input.setBlocking(false);
    xoutVideo->input.setQueueSize(1);

    // Linking
    if (enableUVC) {
        auto uvc = pipeline.create<dai::node::UVC>();
        camRgb->video.link(uvc->input);
        //camRgb->video.link(xoutVideo->input); // Could actually keep this as well
    } else {
        camRgb->video.link(xoutVideo->input);
    }

    if (enableToF) {
        auto camToF = pipeline.create<dai::node::Camera>();
        auto tof = pipeline.create<dai::node::ToF>();
        auto xoutToF = pipeline.create<dai::node::XLinkOut>();
        camToF->setBoardSocket(dai::CameraBoardSocket::LEFT);
        xoutToF->setStreamName("tof");

        camToF->raw.link(tof->inputImage);
        tof->out.link(xoutToF->input);
    }

    std::ofstream fAudio;
    int audioSampleSize = 3; // 2, 3, 4
    if (enableMic) {
        auto uac = pipeline.create<dai::node::UAC>();
        auto xoutMic = pipeline.create<dai::node::XLinkOut>();
        uac->initialConfig.setMicGainDecibels(30);
        uac->setXlinkSampleSizeBytes(audioSampleSize);
        xoutMic->setStreamName("mic");
        uac->out.link(xoutMic->input);

        fAudio.open("audio.raw", std::ios::trunc | std::ios::binary);
    }

    // Connect to device and start pipeline
    auto config = dai::Device::Config();
    config.board.uvcEnable = enableUVC;
    printf("=== Creating device with board config...\n");
    dai::Device device(config);
    printf("=== Device created, connected cameras:\n");
    for (auto s : device.getCameraSensorNames()) {
        std::cout << "  > " << s.first << " : " << s.second << "\n";
    }
    printf("Starting pipeline...\n");
    device.startPipeline(pipeline);
    printf("=== Started!\n");
    if (enableUVC) printf(">>> Keep this running, and open a separate UVC viewer\n");

    int qsize = 1;
    bool blocking = false;
    auto video = device.getOutputQueue("video", qsize, blocking);

    auto depth = enableToF ? device.getOutputQueue("tof", qsize, blocking) : nullptr;
    auto audio = enableMic ? device.getOutputQueue("mic", qsize, blocking) : nullptr;

    using namespace std::chrono;
    auto tprev = steady_clock::now();
    int count = 0;

    while(true) {
        auto videoIn = video->get<dai::ImgFrame>();

        if (1) { // FPS calc
            auto tnow = steady_clock::now();
            count++;
            auto tdiff = duration<double>(tnow - tprev).count();
            if (tdiff >= 1) {
                double fps = count / tdiff;
                printf("FPS: %.3f\n", fps);
                count = 0;
                tprev = tnow;
            }
        }

        // Get BGR frame from NV12 encoded video frame to show with opencv
        // Visualizing the frame on slower hosts might have overhead
        cv::imshow("video", videoIn->getCvFrame());

        if (enableToF) {
            auto depthIn = depth->tryGet<dai::ImgFrame>();
            if (depthIn) {
                auto depthFrm = depthIn->getFrame();
                cv::Mat norm;
                cv::normalize(depthFrm, norm, 255, 0, cv::NORM_MINMAX, CV_8UC1);
                cv::applyColorMap(norm, norm, cv::COLORMAP_JET);
                cv::imshow("tof-depth", norm);
            }
        }

        if (enableMic) {
            auto audioIn = audio->tryGet<dai::ImgFrame>();
            if (audioIn) {
#if 0 // works without zero-copy
                auto audioData = audioIn->getData();
                printf("MIC seq %ld, data size %lu, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        audioData.size(),
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudio.write((char*)&audioData[0], audioData.size());
#else // with zero-copy
                uint8_t *audioData = audioIn->packet->data;
                // uint32_t size = audioIn->packet->length; // not ok, includes metadata
                uint32_t size = audioSampleSize * audioIn->getHeight() * audioIn->getWidth();
                printf("MIC seq %ld, data size %u, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        size,
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudio.write((char*)audioData, size);
#endif
            }
        }


        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
