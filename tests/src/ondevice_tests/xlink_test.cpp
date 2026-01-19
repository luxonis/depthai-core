#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <depthai/depthai.hpp>
#include <thread>

using namespace std::chrono_literals;

TEST_CASE("XLinkIn lazy allocation test") {
    dai::Pipeline p;
    auto manip = p.create<dai::node::ImageManip>()->build();

    auto xLinkInImage = p.create<dai::node::internal::XLinkIn>();
    auto xLinkInConfig = p.create<dai::node::internal::XLinkIn>();

    xLinkInImage->setMaxDataSize(1024 * 1024 * 1024);  // 1GB per frame
    xLinkInImage->setNumFrames(64);

    xLinkInConfig->setMaxDataSize(1024 * 1024 * 1024);  // 1GB per frame
    xLinkInConfig->setNumFrames(64);

    xLinkInImage->out.link(manip->inputImage);
    xLinkInConfig->out.link(manip->inputConfig);

    auto run = [&]() {
        p.start();
        std::this_thread::sleep_for(1s);
    };

    // Without lazy allocation, this will throw as all the above XLinkIn nodes
    // would allocate all the frames at once.
    REQUIRE_NOTHROW(p.build());
    REQUIRE_NOTHROW(run());
}

TEST_CASE("XLinkBridge fps limit test") {
    constexpr float CAMERA_FPS = 30.0;
    constexpr float XLINK_FPS_LIMIT = 3.5;
    constexpr std::chrono::duration<double> TEST_DURATION = 2s;

    dai::Pipeline p;
    auto cam = p.create<dai::node::Camera>()->build();

    auto videoOutput = cam->requestOutput(std::make_pair(640, 400), std::nullopt, dai::ImgResizeMode::CROP, CAMERA_FPS);
    auto videoQueue = videoOutput->createOutputQueue();

    p.build();
    auto xlinkBridge = videoOutput->getXLinkBridge();
    REQUIRE(xlinkBridge != nullptr);
    REQUIRE(xlinkBridge->xLinkOut != nullptr);

    xlinkBridge->xLinkOut->setFpsLimit(XLINK_FPS_LIMIT);
    REQUIRE(xlinkBridge->xLinkOut->getFpsLimit() == XLINK_FPS_LIMIT);

    p.start();
    auto start = std::chrono::steady_clock::now();
    size_t numReceived = 0;
    while(p.isRunning()) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) continue;
        numReceived++;

        if(std::chrono::steady_clock::now() - start > TEST_DURATION) {
            break;
        }
    }
    REQUIRE(numReceived == Catch::Approx(XLINK_FPS_LIMIT * TEST_DURATION.count()).margin(1.01));  // +- 1 frame
}

TEST_CASE("Sync node packet transfer timing and data integrity with varying delays", "[sync][xlink][timing][generate]") {
    // Use GENERATE to run the entire test case once for each value
    const int sendingDistanceMs = GENERATE(1, 10, 50, 100);

    // --- Core Test Parameters ---
    constexpr int width = 1280;
    constexpr int height = 800;
    const int totalSize = width * height;

    constexpr int numPackets = 20;
    // Calculate packet size to ensure the whole image fits perfectly
    const int packetSize = static_cast<int>((totalSize + numPackets - 1) / numPackets);

    // --- 1. Pipeline and Node Setup ---
    dai::Pipeline pipeline;
    auto syncNode = pipeline.create<dai::node::Sync>();

    // Create input and output queues
    auto input = syncNode->inputs["input"];
    auto inputQueue = input.createInputQueue();
    auto output = syncNode->out.createOutputQueue();

    // --- 2. Build, Configure XLink, and Start Pipeline ---
    pipeline.build();
    auto xlinkBridge = syncNode->out.getXLinkBridge();
    xlinkBridge->xLinkOut->setPacketSize(packetSize);
    xlinkBridge->xLinkOut->setBytesPerSecondLimit(packetSize * (1000 / sendingDistanceMs));
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // --- 3. Create Test Frame and Timestamps ---
    auto desired_duration = std::chrono::seconds(5);
    std::chrono::time_point<std::chrono::steady_clock> custom_timestamp(desired_duration);
    cv::Mat mat = cv::Mat::zeros(height, width, CV_8UC1);
    auto imgFrame = std::make_shared<dai::ImgFrame>();
    imgFrame->setCvFrame(mat, dai::ImgFrame::Type::GRAY8);
    imgFrame->setTimestamp(custom_timestamp);
    imgFrame->setTimestampDevice(custom_timestamp);

    // --- 4. Send, Receive, and Measure Time ---
    auto start = std::chrono::high_resolution_clock::now();
    inputQueue->send(imgFrame);
    auto returnedMessageGroup = std::static_pointer_cast<dai::MessageGroup>(output->get());
    auto returnedFrame = returnedMessageGroup->get<dai::ImgFrame>("input");
    auto end = std::chrono::high_resolution_clock::now();
    auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    // Validate data integrity
    cv::Mat returnedMat = returnedFrame->getCvFrame();
    bool same = std::equal(mat.begin<uchar>(), mat.end<uchar>(), returnedMat.begin<uchar>());

    REQUIRE(same);

    // Assert minimum forced delay (numPackets * sendingDistanceMs)
    REQUIRE(durationMs >= numPackets * sendingDistanceMs);

    // Assert maximum bound allowing for a reasonable margin of 200ms overhead
    REQUIRE(durationMs < numPackets * sendingDistanceMs + 200);
}

TEST_CASE("Sync node packet transfer data integrity with more frames in MessageGroup", "[sync][xlink]") {
    // Use GENERATE to run the entire test case once for each value
    const int sendingDistanceMs = 10;

    // --- Core Test Parameters ---
    constexpr int width = 1280;
    constexpr int height = 800;
    const int totalSize = width * height;

    constexpr int numPackets = 20;
    // Calculate packet size to ensure the whole image fits perfectly
    const int packetSize = static_cast<int>((totalSize + numPackets - 1) / numPackets);

    // --- 1. Pipeline and Node Setup ---
    dai::Pipeline pipeline;
    auto syncNode = pipeline.create<dai::node::Sync>();

    // Create input and output queues
    auto input1 = syncNode->inputs["input1"];
    auto input2 = syncNode->inputs["input2"];
    auto inputQueue1 = input1.createInputQueue();
    auto inputQueue2 = input2.createInputQueue();
    auto output = syncNode->out.createOutputQueue();

    // --- 2. Build, Configure XLink, and Start Pipeline ---
    pipeline.build();
    auto xlinkBridge = syncNode->out.getXLinkBridge();
    xlinkBridge->xLinkOut->setPacketSize(packetSize);
    xlinkBridge->xLinkOut->setBytesPerSecondLimit(packetSize * (1000 / sendingDistanceMs));
    pipeline.start();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // --- 3. Create Test Frame and Timestamps ---
    auto desired_duration = std::chrono::seconds(5);
    std::chrono::time_point<std::chrono::steady_clock> custom_timestamp(desired_duration);
    cv::Mat mat1 = cv::Mat::zeros(height, width, CV_8UC1);
    {
        uchar* dataPtr = mat1.data;
        for(int i = 1; i <= totalSize; ++i) {
            *dataPtr = static_cast<uchar>(i % 256);
            dataPtr++;
        }
    }
    cv::Mat mat2 = cv::Mat::zeros(height, width, CV_8UC1);
    {
        uchar* dataPtr = mat2.data;
        for(int i = 1; i <= totalSize; ++i) {
            *dataPtr = static_cast<uchar>(i % 128);
            dataPtr++;
        }
    }
    auto imgFrame1 = std::make_shared<dai::ImgFrame>();
    imgFrame1->setCvFrame(mat1, dai::ImgFrame::Type::GRAY8);
    imgFrame1->setTimestamp(custom_timestamp);
    imgFrame1->setTimestampDevice(custom_timestamp);

    auto imgFrame2 = std::make_shared<dai::ImgFrame>();
    imgFrame2->setCvFrame(mat2, dai::ImgFrame::Type::GRAY8);
    imgFrame2->setTimestamp(custom_timestamp);
    imgFrame2->setTimestampDevice(custom_timestamp);

    // --- 4. Send, Receive, and Measure Time ---
    inputQueue1->send(imgFrame1);
    inputQueue2->send(imgFrame2);
    auto returnedMessageGroup = output->get<dai::MessageGroup>();

    auto returnedFrame1 = returnedMessageGroup->get<dai::ImgFrame>("input1");
    auto returnedFrame2 = returnedMessageGroup->get<dai::ImgFrame>("input2");

    // Validate data integrity
    cv::Mat returnedMat1 = returnedFrame1->getCvFrame();
    cv::Mat returnedMat2 = returnedFrame2->getCvFrame();
    bool same1 = std::equal(mat1.begin<uchar>(), mat1.end<uchar>(), returnedMat1.begin<uchar>());
    bool same2 = std::equal(mat2.begin<uchar>(), mat2.end<uchar>(), returnedMat2.begin<uchar>());

    REQUIRE(same1);
    REQUIRE(same2);
}
