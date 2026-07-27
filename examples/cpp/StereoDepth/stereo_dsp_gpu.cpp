#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <opencv2/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

constexpr int kMaxDisparity = 64;
// Roughly 5 s at 13 fps, enough for auto-exposure to settle.
constexpr int kFramesToSkip = 65;

std::atomic<bool> quitEvent(false);

void signalHandler(int) {
    quitEvent = true;
}

// Reference implementation the DSP+GPU backend is modelled on: OpenCV SGBM in
// MODE_SGBM_3WAY (two horizontal paths plus one vertical path).
//
// The parameters mirror the DSP kernel exactly:
//   blockSize 1      - the DSP computes a per-pixel BT cost, no block sum
//   preFilterCap 31  - the Sobel channel is clamped to +-31 and offset by 31
//   P1 8, P2 32      - the DSP penalties
//   uniquenessRatio 10, disp12MaxDiff 1 - the OpenCL select / LR-check kernels
cv::Mat referenceSgbm3Way(const cv::Mat& rectifiedLeft, const cv::Mat& rectifiedRight) {
    auto sgbm = cv::StereoSGBM::create(0, kMaxDisparity, 1);
    sgbm->setP1(8);
    sgbm->setP2(32);
    sgbm->setPreFilterCap(31);
    sgbm->setUniquenessRatio(10);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setSpeckleWindowSize(0);
    sgbm->setSpeckleRange(0);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disparity;
    sgbm->compute(rectifiedLeft, rectifiedRight, disparity);
    return disparity;  // CV_16S, fixed point with 4 fractional bits
}

// Writes the rectified pair and our disparity so the frame can be re-checked
// offline with the standalone StereoDepth3Path tooling.
void dumpFrames(const cv::Mat& rectifiedLeft, const cv::Mat& rectifiedRight, const cv::Mat& ours, int64_t sequenceNum) {
    const std::string prefix = "/tmp/stereo_dsp_gpu_" + std::to_string(sequenceNum);
    cv::imwrite(prefix + "_left.pgm", rectifiedLeft);
    cv::imwrite(prefix + "_right.pgm", rectifiedRight);
    std::ofstream stream(prefix + "_disparity.s16", std::ios::binary);
    for(int y = 0; y < ours.rows; ++y) {
        stream.write(ours.ptr<char>(y), static_cast<std::streamsize>(ours.cols) * 2);
    }
    std::printf("[dump] wrote %s_{left,right}.pgm and %s_disparity.s16\n", prefix.c_str(), prefix.c_str());
}

struct Comparison {
    long long compared = 0;
    long long exact = 0;
    long long within1 = 0;
    long long ourValid = 0;
    long long referenceValid = 0;
    long long total = 0;
    double meanAbsError = 0.0;
};

// Both images are Q12.4 fixed point. OpenCV marks invalid pixels with a negative
// disparity, the node marks them with zero, so only pixels valid in both are
// compared. A genuine zero disparity is degenerate and excluded either way.
Comparison compare(const cv::Mat& ours, const cv::Mat& reference) {
    Comparison result;
    double errorSum = 0.0;
    for(int y = 0; y < reference.rows; ++y) {
        const auto* referenceRow = reference.ptr<short>(y);
        const auto* ourRow = ours.ptr<short>(y);
        for(int x = kMaxDisparity; x < reference.cols; ++x) {
            ++result.total;
            result.ourValid += ourRow[x] > 0;
            result.referenceValid += referenceRow[x] > 0;
            if(referenceRow[x] <= 0 || ourRow[x] <= 0) {
                continue;
            }
            const int difference = std::abs(static_cast<int>(ourRow[x]) - static_cast<int>(referenceRow[x]));
            ++result.compared;
            result.exact += difference == 0;
            result.within1 += difference <= 16;
            errorSum += difference / 16.0;
        }
    }
    if(result.compared > 0) {
        result.meanAbsError = errorSum / static_cast<double>(result.compared);
    }
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    signal(SIGTERM, signalHandler);
    signal(SIGINT, signalHandler);

    const int framesToRun = argc > 1 ? std::atoi(argv[1]) : 200;
    const bool display = std::getenv("DISPLAY") != nullptr;

    dai::Pipeline pipeline;

    auto monoLeft = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_B);
    auto monoRight = pipeline.create<dai::node::Camera>()->build(dai::CameraBoardSocket::CAM_C);

    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto monoLeftOut = monoLeft->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8);
    auto monoRightOut = monoRight->requestOutput({1280, 800}, dai::ImgFrame::Type::GRAY8);

    monoLeftOut->link(stereo->left);
    monoRightOut->link(stereo->right);

    stereo->setRectification(true);
    stereo->setStereoBackend(dai::node::StereoDepth::Properties::StereoBackend::DSP_GPU);

    // On-device throughput measurement of the disparity stream.
    auto benchmark = pipeline.create<dai::node::BenchmarkIn>();
    benchmark->sendReportEveryNMessages(50);
    stereo->disparity.link(benchmark->input);

    auto disparityQueue = benchmark->passthrough.createOutputQueue();
    auto reportQueue = benchmark->report.createOutputQueue();
    auto rectifiedLeftQueue = stereo->rectifiedLeft.createOutputQueue();
    auto rectifiedRightQueue = stereo->rectifiedRight.createOutputQueue();

    pipeline.start();

    int framesReceived = 0;
    Comparison total;
    int comparedFrames = 0;

    // The rectified and disparity streams are separate queues, so a frame may be
    // dropped from one but not the other. Align them by sequence number before
    // comparing, otherwise the check compares unrelated frames.
    auto getWithSequenceNum = [](const std::shared_ptr<dai::MessageQueue>& queue, int64_t sequenceNum) {
        for(int attempt = 0; attempt < 60; ++attempt) {
            auto frame = queue->get<dai::ImgFrame>();
            if(frame == nullptr) {
                return std::shared_ptr<dai::ImgFrame>{};
            }
            if(frame->getSequenceNum() >= sequenceNum) {
                return frame->getSequenceNum() == sequenceNum ? frame : std::shared_ptr<dai::ImgFrame>{};
            }
        }
        return std::shared_ptr<dai::ImgFrame>{};
    };

    while(pipeline.isRunning() && !quitEvent && framesReceived < framesToRun) {
        auto disparity = disparityQueue->get<dai::ImgFrame>();
        if(disparity == nullptr) {
            continue;
        }
        ++framesReceived;

        if(auto report = reportQueue->tryGet<dai::BenchmarkReport>(); report != nullptr) {
            std::printf("[perf] %.2f fps over %.0f messages, average latency %.2f ms\n",
                        report->fps,
                        report->numMessagesReceived,
                        report->averageLatency * 1000.0);
            std::fflush(stdout);
        }

        // Validate a few frames against OpenCV SGBM_3WAY on the same rectified pair.
        // The first frames are skipped so auto-exposure has settled; on dark,
        // low-texture frames both implementations are dominated by cost ties and
        // the comparison says more about the scene than about the backend.
        if(comparedFrames < 3 && framesReceived > kFramesToSkip) {
            const auto sequenceNum = disparity->getSequenceNum();
            auto rectifiedLeft = getWithSequenceNum(rectifiedLeftQueue, sequenceNum);
            auto rectifiedRight = getWithSequenceNum(rectifiedRightQueue, sequenceNum);
            if(rectifiedLeft != nullptr && rectifiedRight != nullptr) {
                const cv::Mat ours = disparity->getCvFrame();
                const cv::Mat leftMat = rectifiedLeft->getCvFrame();
                const cv::Mat rightMat = rectifiedRight->getCvFrame();
                if(comparedFrames == 0) {
                    dumpFrames(leftMat, rightMat, ours, sequenceNum);
                }
                const cv::Mat reference = referenceSgbm3Way(leftMat, rightMat);
                const auto comparison = compare(ours, reference);
                std::printf("[check] seq %lld vs SGBM_3WAY: valid ours=%.1f%% opencv=%.1f%% | exact=%.2f%% within1px=%.2f%% MAE=%.3f px\n",
                            static_cast<long long>(sequenceNum),
                            100.0 * comparison.ourValid / std::max<long long>(comparison.total, 1),
                            100.0 * comparison.referenceValid / std::max<long long>(comparison.total, 1),
                            100.0 * comparison.exact / std::max<long long>(comparison.compared, 1),
                            100.0 * comparison.within1 / std::max<long long>(comparison.compared, 1),
                            comparison.meanAbsError);
                std::fflush(stdout);
                total.compared += comparison.compared;
                total.exact += comparison.exact;
                total.within1 += comparison.within1;
                total.meanAbsError += comparison.meanAbsError;
                ++comparedFrames;
            }
        }

        if(display) {
            cv::Mat normalized;
            disparity->getFrame().convertTo(normalized, CV_8UC1, 255.0 / ((kMaxDisparity - 1) * 16));
            cv::Mat colorized;
            cv::applyColorMap(normalized, colorized, cv::COLORMAP_JET);
            colorized.setTo(cv::Scalar(0, 0, 0), normalized == 0);
            cv::imshow("disparity (DSP+GPU)", colorized);
            if(cv::waitKey(1) == 'q') {
                break;
            }
        }
    }

    pipeline.stop();
    pipeline.wait();

    if(comparedFrames > 0) {
        std::printf("[check] average over %d frames: exact=%.2f%% within1px=%.2f%% MAE=%.3f px\n",
                    comparedFrames,
                    100.0 * total.exact / std::max<long long>(total.compared, 1),
                    100.0 * total.within1 / std::max<long long>(total.compared, 1),
                    total.meanAbsError / comparedFrames);
    }

    return 0;
}
