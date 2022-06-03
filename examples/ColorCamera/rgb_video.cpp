#include <iostream>
#include <fstream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Pass the argument `uvc` to run in UVC mode

int main(int argc, char** argv) {
    bool enableUVC = 0;
    bool enableToF = 0;
    bool enableMic = 0;
    bool enableMicNc = 0;
    bool enableNN = 0; // Set 1 for resource allocation test
    int audioSampleSize = 3; // 2, 3, 4. Note: must be 2 for NC
    if(argc > 1) {
        if (std::string(argv[1]) == "uvc") {
            enableUVC = 1;
        } else if (std::string(argv[1]) == "tof") {
            enableToF = 1;
        } else if (std::string(argv[1]) == "mic") {
            enableMic = 1;
        } else if (std::string(argv[1]) == "micnc") {
            enableMic = 1;
            enableMicNc = 1;
            audioSampleSize = 2;
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
    std::ofstream fAudioBack;
    std::ofstream fAudioNc;
    if (enableMic) {
        auto uac = pipeline.create<dai::node::UAC>();
        auto mic = pipeline.create<dai::node::AudioMic>();
        auto xoutMic = pipeline.create<dai::node::XLinkOut>();
        auto xoutMicBack = pipeline.create<dai::node::XLinkOut>();

        // mic->initialConfig.setMicGainDecibels(30); // with NC we also have AGC
        mic->setXlinkSampleSizeBytes(audioSampleSize);

        xoutMic->setStreamName("mic");
        xoutMicBack->setStreamName("micBack");

        mic->out.link(xoutMic->input);
        mic->outBack.link(xoutMicBack->input);

        if (enableMicNc) {
            auto audioProc = pipeline.create<dai::node::AudioProc>();
            auto xoutMicNc = pipeline.create<dai::node::XLinkOut>();

            xoutMicNc->setStreamName("micNc");

            mic->out.link(audioProc->input);
            mic->outBack.link(audioProc->reference);
            audioProc->out.link(xoutMicNc->input);
            audioProc->out.link(uac->input);

            fAudioNc.open("audioNc.raw", std::ios::trunc | std::ios::binary);
        } else {
            mic->out.link(uac->input);
        }

        fAudio.open("audio.raw", std::ios::trunc | std::ios::binary);
        fAudioBack.open("audioBack.raw", std::ios::trunc | std::ios::binary);
    }

    if (enableNN) {
        // Default blob path provided by Hunter private data download
        // Applicable for easier example usage only
        std::string nnPath(BLOB_PATH);
        std::cout << "NN blob: " << nnPath << "\n";

        auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();
        auto nnOut = pipeline.create<dai::node::XLinkOut>();
        nnOut->setStreamName("nn");

        camRgb->setPreviewSize(300, 300);  // NN input
        camRgb->setInterleaved(false);

        nn->setConfidenceThreshold(0.5);
        nn->setBlobPath(nnPath);
        nn->setNumInferenceThreads(2); // TODO
        nn->input.setBlocking(false);

        camRgb->preview.link(nn->input);
        nn->out.link(nnOut->input);
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

    int qsize = 8;
    bool blocking = false;
    auto video = device.getOutputQueue("video", qsize, blocking);

    auto depth = enableToF ? device.getOutputQueue("tof", qsize, blocking) : nullptr;
    auto audio = enableMic ? device.getOutputQueue("mic", qsize, blocking) : nullptr;
    auto audioBack = enableMic ? device.getOutputQueue("micBack", qsize, blocking) : nullptr;
    auto audioNc = enableMicNc ? device.getOutputQueue("micNc", qsize, blocking) : nullptr;
    auto nn = enableNN ? device.getOutputQueue("nn", qsize, blocking) : nullptr;

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

        if (enableNN) {
            auto nnIn = nn->tryGet<dai::ImgDetections>();
            if (nnIn) {
                printf("Got NN results\n");
            }
        }

        if (enableMic) {
            // Main/front mics - 2x 48kHz
            auto audioIn = audio->tryGet<dai::ImgFrame>();
            if (audioIn) {
#if 0 // works without zero-copy
                auto audioData = audioIn->getData();
                printf("MIC      seq %ld, data size %lu, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        audioData.size(),
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudio.write((char*)&audioData[0], audioData.size());
#else // with zero-copy
                uint8_t *audioData = audioIn->packet->data;
                // uint32_t size = audioIn->packet->length; // not ok, includes metadata
                uint32_t size = audioSampleSize * audioIn->getHeight() * audioIn->getWidth();
                printf("MIC      seq %ld, data size %u, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        size,
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudio.write((char*)audioData, size);
#endif
            }

            // Back mic - 1x 48kHz
            audioIn = audioBack->tryGet<dai::ImgFrame>();
            if (audioIn) {
#if 0 // works without zero-copy
                auto audioData = audioIn->getData();
                printf("MIC-back seq %ld, data size %lu, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        audioData.size(),
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudio.write((char*)&audioData[0], audioData.size());
#else // with zero-copy
                uint8_t *audioData = audioIn->packet->data;
                // uint32_t size = audioIn->packet->length; // not ok, includes metadata
                uint32_t size = audioSampleSize * audioIn->getHeight() * audioIn->getWidth();
                printf("MIC-back seq %ld, data size %u, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        size,
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudioBack.write((char*)audioData, size);
#endif
            }

          if (enableMicNc) {
            // AudioProc output (with noise cancelation) - 2x 16kHz
            audioIn = audioNc->tryGet<dai::ImgFrame>();
            if (audioIn) {
#if 0 // works without zero-copy
                auto audioData = audioIn->getData();
                printf("MIC-NC   seq %ld, data size %lu, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        audioData.size(),
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudio.write((char*)&audioData[0], audioData.size());
#else // with zero-copy
                uint8_t *audioData = audioIn->packet->data;
                // uint32_t size = audioIn->packet->length; // not ok, includes metadata
                uint32_t size = audioSampleSize * audioIn->getHeight() * audioIn->getWidth();
                printf("MIC-NC   seq %ld, data size %u, samples %d, mics %d\n",
                        audioIn->getSequenceNum(),
                        size,
                        audioIn->getHeight(),
                        audioIn->getWidth() );
                fAudioNc.write((char*)audioData, size);
#endif
            }
          }
        }


        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
