#include <catch2/catch_all.hpp>
#include <depthai/pipeline/Pipeline.hpp>
#include <depthai/pipeline/datatype/MessageGroup.hpp>
#include <depthai/pipeline/node/GPUStereo.hpp>
#include <depthai/pipeline/node/Rectification.hpp>
#include <depthai/pipeline/node/StereoDepth.hpp>
#include <depthai/properties/GPUStereoProperties.hpp>
#include <depthai/properties/RectificationProperties.hpp>
#include <limits>

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
    #include <opencv2/calib3d.hpp>
    #include <opencv2/imgproc.hpp>
#endif

TEST_CASE("Rectification backend selection survives host-device serialization", "[Rectification]") {
    auto node = dai::node::Rectification::create();
    REQUIRE(node->properties.backend == dai::Backend::AUTO);
    for(auto backend : {dai::Backend::CPU, dai::Backend::HW, dai::Backend::GPU, dai::Backend::AUTO}) {
        REQUIRE(&node->setBackend(backend) == node.get());
        for(auto format : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
            std::vector<uint8_t> data;
            node->getProperties().serialize(data, format);
            dai::RectificationProperties decoded;
            REQUIRE(dai::utility::deserialize(data, decoded, format));
            REQUIRE(decoded.backend == backend);
        }
    }
}

TEST_CASE("Rectification alpha scaling is optional, validated and serialized", "[Rectification]") {
    auto node = dai::node::Rectification::create();
    REQUIRE_FALSE(node->properties.alphaScaling);
    const auto alpha = GENERATE(std::optional<float>{}, std::optional<float>{-0.5f}, std::optional<float>{0.f}, std::optional<float>{1.f});
    if(alpha) REQUIRE(&node->setAlphaScaling(*alpha) == node.get());
    for(auto format : {dai::SerializationType::LIBNOP, dai::SerializationType::JSON, dai::SerializationType::JSON_MSGPACK}) {
        std::vector<uint8_t> data;
        node->getProperties().serialize(data, format);
        dai::RectificationProperties decoded;
        REQUIRE(dai::utility::deserialize(data, decoded, format));
        REQUIRE(decoded.alphaScaling == alpha);
    }
    REQUIRE_THROWS_AS(node->setAlphaScaling(std::numeric_limits<float>::quiet_NaN()), std::invalid_argument);
    REQUIRE_THROWS_AS(node->setAlphaScaling(std::numeric_limits<float>::infinity()), std::invalid_argument);
    REQUIRE(node->properties.alphaScaling == alpha);
    auto stereo = dai::node::StereoDepth::create();
    stereo->setAlphaScaling(0.5f);
    REQUIRE(stereo->rectification->properties.alphaScaling == 0.5f);
}

TEST_CASE("GPUStereo rectification toggle updates Rectification subnode properties", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();

    node->setRectification(false);
    {
        auto& rectProps = (*node->rectification)->properties;
        REQUIRE(rectProps.enableRectification == false);
    }

    node->setRectification(true);
    {
        auto& rectProps = (*node->rectification)->properties;
        REQUIRE(rectProps.enableRectification == true);
    }
}

TEST_CASE("GPUStereo confidence threshold is clamped to [0, 255]", "[GPUStereo]") {
    auto node = dai::node::GPUStereo::create();

    node->initialConfig->setConfidenceThreshold(-1);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(0));

    node->initialConfig->setConfidenceThreshold(0);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(0));

    node->initialConfig->setConfidenceThreshold(255);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(255));

    node->initialConfig->setConfidenceThreshold(256);
    REQUIRE(node->initialConfig->confidenceThreshold == static_cast<std::uint8_t>(255));
}

TEST_CASE("StereoDepth follows Rectification then Sync, with a true bypass when disabled", "[StereoDepth][Rectification]") {
    const bool enabled = GENERATE(true, false);
    dai::Pipeline pipeline(false);
    struct HostStereoDepth : dai::node::StereoDepth {
        bool runOnHost() const override {
            return true;
        }
    };
    auto stereo = std::make_shared<HostStereoDepth>();
    pipeline.add(stereo);
    stereo->buildInternal();
    stereo->rectification->setRunOnHost(true);
    for(auto& child : stereo->getNodeMap()) {
        if(auto sync = std::dynamic_pointer_cast<dai::node::Sync>(child)) sync->setRunOnHost(true);
    }
    const bool configureBeforeLinking = GENERATE(true, false);
    if(configureBeforeLinking) stereo->setRectification(enabled);
    auto left = stereo->left.createInputQueue();
    auto right = stereo->right.createInputQueue();
    auto syncedLeft = stereo->syncedLeft.createOutputQueue();
    auto syncedRight = stereo->syncedRight.createOutputQueue();
    auto sourceOf = [&](dai::Node::Input& input) -> dai::Node::Output* {
        for(auto& node : pipeline.getAllNodes()) {
            for(auto* output : node->getOutputRefs()) {
                for(auto& connection : output->getConnections()) {
                    if(connection.in == &input) return output;
                }
            }
        }
        return nullptr;
    };
    auto* leftSource = sourceOf(stereo->left);
    auto* rightSource = sourceOf(stereo->right);
    REQUIRE(leftSource != nullptr);
    REQUIRE(rightSource != nullptr);
    // Changing the route after producer links exist must preserve those links.
    if(!configureBeforeLinking) {
        stereo->setRectification(!enabled);
        stereo->setRectification(enabled);
    }
    REQUIRE(stereo->rectification->properties.enableRectification == enabled);
    // StereoDepth owns the rectification toggle, including when the subnode was configured directly.
    stereo->rectification->enableRectification(false);
    stereo->left.setMaxSize(7);
    stereo->left.setBlocking(false);

    pipeline.build();
    std::shared_ptr<dai::node::Sync> sync;
    for(const auto& child : stereo->getNodeMap()) {
        if(auto candidate = std::dynamic_pointer_cast<dai::node::Sync>(child)) sync = candidate;
        REQUIRE(std::string(child->getName()) != "MessageDemux");
    }
    REQUIRE(sync != nullptr);
    REQUIRE(sync->getSyncThreshold() == std::chrono::nanoseconds(16'666'666));
    REQUIRE(stereo->getNodeMap().size() == (enabled ? 2 : 1));
    REQUIRE(sync->out.getConnections().at(0).in == stereo->getInputRef("inputSynced"));
    REQUIRE_FALSE(stereo->properties.enableRectification);
    REQUIRE(stereo->syncedRight.getQueueConnections().at(0).queue == syncedRight);
    auto& leftInput = enabled ? stereo->rectification->input1 : sync->inputs["left"];
    auto& rightInput = enabled ? stereo->rectification->input2 : sync->inputs["right"];
    REQUIRE(leftSource->getConnections().at(0).in == &leftInput);
    REQUIRE(rightSource->getConnections().at(0).in == &rightInput);
    REQUIRE(leftInput.getMaxSize() == 7);
    REQUIRE_FALSE(leftInput.getBlocking());
    auto leftFrame = std::make_shared<dai::ImgFrame>();
    auto rightFrame = std::make_shared<dai::ImgFrame>();
    leftSource->send(leftFrame);
    rightSource->send(rightFrame);
    REQUIRE(leftInput.tryGet<dai::ImgFrame>() == leftFrame);
    REQUIRE(rightInput.tryGet<dai::ImgFrame>() == rightFrame);
    if(enabled) {
        REQUIRE(stereo->rectification->properties.enableRectification);
        REQUIRE(sync->getSyncAttempts() == -1);
        for(const auto& input : sync->inputs) {
            REQUIRE_FALSE(input.second.getBlocking());
            REQUIRE(input.second.getMaxSize() == 1);
        }
        REQUIRE(stereo->rectification->output1.getConnections().at(0).in == &sync->inputs["left"]);
        REQUIRE(stereo->rectification->output2.getConnections().at(0).in == &sync->inputs["right"]);
        REQUIRE(stereo->rectification->passthrough1.getConnections().at(0).in == &sync->inputs["originalLeft"]);
        REQUIRE(stereo->rectification->passthrough2.getConnections().at(0).in == &sync->inputs["originalRight"]);
    } else {
        REQUIRE(sync->inputs.size() == 2);
        REQUIRE_FALSE(stereo->rectification->input1.isConnected());
        REQUIRE_FALSE(stereo->rectification->input2.isConnected());
    }
}

#if defined(DEPTHAI_HAVE_OPENCV_SUPPORT)
TEST_CASE("Rectification preserves RGB and BGR formats through bypass and remapping", "[Rectification]") {
    const auto type = GENERATE(dai::ImgFrame::Type::RGB888i, dai::ImgFrame::Type::RGB888p, dai::ImgFrame::Type::BGR888i, dai::ImgFrame::Type::BGR888p);
    const auto alpha =
        GENERATE(std::optional<float>{}, std::optional<float>{-0.5f}, std::optional<float>{0.f}, std::optional<float>{0.5f}, std::optional<float>{1.f});
    const bool resize = GENERATE(false, true);
    dai::Pipeline pipeline(false);
    auto rectify = pipeline.create<dai::node::Rectification>();
    rectify->setRunOnHost(true);
    if(alpha) rectify->setAlphaScaling(*alpha);
    if(resize) rectify->setOutputSize(32, 24);
    auto left = rectify->input1.createInputQueue();
    auto right = rectify->input2.createInputQueue();
    auto outputLeft = rectify->output1.createOutputQueue();
    auto outputRight = rectify->output2.createOutputQueue();
    cv::Mat pixels(48, 64, CV_8UC3);
    for(int y = 0; y < pixels.rows; ++y)
        for(int x = 0; x < pixels.cols; ++x) pixels.at<cv::Vec3b>(y, x) = cv::Vec3b(x * 3, y * 4, 220 - x);
    pipeline.start();
    for(const bool distorted : {false, true, false}) {
        CAPTURE(type, distorted, alpha, resize);
        const std::vector<float> distortion = {distorted ? -0.1f : 0.f, 0, 0, 0, 0};
        for(int side = 0; side < 2; ++side) {
            const std::array<std::array<float, 3>, 3> intrinsic = {{{80, 0, 32}, {0, 80, 24}, {0, 0, 1}}};
            dai::ImgTransformation transformation(64, 48, intrinsic);
            transformation.setDistortionCoefficients(distortion);
            transformation.setExtrinsics(dai::Extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {side ? 7.5f : 0.f, 0, 0}, dai::CameraBoardSocket::CAM_A));
            auto frame = std::make_shared<dai::ImgFrame>();
            frame->setCvFrame(pixels, type);
            frame->setTransformation(transformation);
            (side == 0 ? left : right)->send(frame);
        }
        const cv::Mat intrinsic = (cv::Mat_<float>(3, 3) << 80, 0, 32, 0, 80, 24, 0, 0, 1);
        cv::Mat target = alpha ? cv::getOptimalNewCameraMatrix(intrinsic, distortion, pixels.size(), *alpha) : intrinsic.clone();
        target.convertTo(target, CV_32F);
        if(resize) {
            target.row(0) *= 0.5f;
            target.row(1) *= 0.5f;
        }
        const cv::Size outputSize = resize ? cv::Size(32, 24) : pixels.size();
        cv::Mat mapX, mapY, expected;
        cv::initUndistortRectifyMap(intrinsic, distortion, cv::Mat::eye(3, 3, CV_32F), target, outputSize, CV_32FC1, mapX, mapY);
        cv::remap(pixels, expected, mapX, mapY, cv::INTER_LINEAR);
        for(auto& queue : {outputLeft, outputRight}) {
            bool timedOut = false;
            auto output = queue->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(output);
            REQUIRE(output->getType() == type);
            REQUIRE(output->getData().size() == static_cast<size_t>(outputSize.area() * 3));
            const auto actualIntrinsic = output->transformation.getIntrinsicMatrix();
            for(int row = 0; row < 3; ++row)
                for(int col = 0; col < 3; ++col) REQUIRE(actualIntrinsic[row][col] == Catch::Approx(target.at<float>(row, col)));
            REQUIRE(cv::norm(output->getCvFrame(), expected, cv::NORM_INF) == 0);
        }
    }
    pipeline.stop();
}

TEST_CASE("StereoDepth rectification recovers from input skew larger than its queues", "[StereoDepth][Rectification]") {
    const bool leftAhead = GENERATE(false, true);
    dai::Pipeline pipeline(false);
    struct HostStereoDepth : dai::node::StereoDepth {
        Output matched{*this, {"matched", DEFAULT_GROUP, {{{dai::DatatypeEnum::MessageGroup, false}}}}};
        bool runOnHost() const override {
            return true;
        }
        void run() override {
            while(mainLoop()) matched.send(getInputRef("inputSynced")->get<dai::MessageGroup>());
        }
    };
    auto stereo = std::make_shared<HostStereoDepth>();
    pipeline.add(stereo);
    stereo->buildInternal();
    stereo->rectification->setRunOnHost(true);
    stereo->rectification->setBackend(dai::Backend::CPU);
    for(auto& child : stereo->getNodeMap()) {
        if(auto sync = std::dynamic_pointer_cast<dai::node::Sync>(child)) sync->setRunOnHost(true);
    }
    stereo->left.setMaxSize(64);
    stereo->right.setMaxSize(64);
    auto left = stereo->left.createInputQueue(64, true);
    auto right = stereo->right.createInputQueue(64, true);
    auto output = stereo->matched.createOutputQueue(64, true);
    pipeline.start();
    for(int index = 0; index <= 40; ++index) {
        for(int side = 0; side < 2; ++side) {
            const int sequence = index < 30 ? index + ((side == 0) == leftAhead ? 10 : 0) : index + 10;
            const std::array<std::array<float, 3>, 3> intrinsic = {{{80, 0, 32}, {0, 80, 24}, {0, 0, 1}}};
            dai::ImgTransformation transformation(64, 48, intrinsic);
            transformation.setDistortionCoefficients({-0.1f, 0, 0, 0, 0});
            transformation.setExtrinsics(dai::Extrinsics({{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, {side ? 7.5f : 0.f, 0, 0}, dai::CameraBoardSocket::CAM_A));
            auto frame = std::make_shared<dai::ImgFrame>();
            frame->setType(dai::ImgFrame::Type::RAW8);
            frame->setSize(64, 48);
            frame->setData(std::vector<uint8_t>(64 * 48, 127));
            frame->setSequenceNum(sequence);
            frame->setTimestampDevice(std::chrono::steady_clock::time_point(std::chrono::milliseconds(33 * sequence)));
            frame->setTransformation(transformation);
            (side == 0 ? left : right)->send(frame);
        }
    }
    int previousSequence = -1;
    while(previousSequence < 50) {
        bool timedOut = false;
        auto group = output->get<dai::MessageGroup>(std::chrono::seconds(5), timedOut);
        REQUIRE_FALSE(timedOut);
        REQUIRE(group);
        const auto sequence = group->get<dai::ImgFrame>("left")->getSequenceNum();
        REQUIRE(sequence > previousSequence);
        previousSequence = sequence;
        for(const auto* name : {"left", "right", "originalLeft", "originalRight"}) {
            auto frame = group->get<dai::ImgFrame>(name);
            REQUIRE(frame);
            REQUIRE(frame->getSequenceNum() == sequence);
            REQUIRE(frame->getTimestampDevice() == std::chrono::steady_clock::time_point(std::chrono::milliseconds(33 * sequence)));
        }
    }
    pipeline.stop();
}

TEST_CASE("Rectification forwards already rectified frames and resumes remapping when geometry changes", "[Rectification]") {
    const auto change = GENERATE(1, 2, 3);
    const auto resize = GENERATE(false, true);
    dai::Pipeline pipeline(false);
    auto rectify = pipeline.create<dai::node::Rectification>();
    rectify->setRunOnHost(true);
    if(resize) rectify->setOutputSize(32, 24);
    auto left = rectify->input1.createInputQueue();
    auto right = rectify->input2.createInputQueue();
    auto outputLeft = rectify->output1.createOutputQueue();
    auto outputRight = rectify->output2.createOutputQueue();
    auto passthroughLeft = rectify->passthrough1.createOutputQueue();
    auto passthroughRight = rectify->passthrough2.createOutputQueue();
    pipeline.start();
    for(const auto stage : {0, 0, change, change, 0, 0}) {
        CAPTURE(stage, resize);
        auto frame = [&](bool rightSide) {
            std::array<std::array<float, 3>, 3> intrinsic = {{{80, 0, 32}, {0, 80, 24}, {0, 0, 1}}};
            if(rightSide && stage == 3) intrinsic[0][0] = 90;
            dai::ImgTransformation transformation(64, 48, intrinsic);
            if(stage == 1) transformation.setDistortionCoefficients({-0.1f, 0, 0, 0, 0});
            const std::vector<std::vector<float>> identity = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
            dai::Extrinsics pose(identity, {rightSide ? 7.5f : 0.f, rightSide && stage == 2 ? 1.f : 0.f, 0}, dai::CameraBoardSocket::CAM_A);
            transformation.setExtrinsics(pose);
            auto image = std::make_shared<dai::ImgFrame>();
            image->setType(dai::ImgFrame::Type::RAW8);
            image->setSize(64, 48);
            image->setData(std::vector<uint8_t>(64 * 48, 127));
            image->setTransformation(transformation);
            return image;
        };
        auto inputLeft = frame(false);
        auto inputRight = frame(true);
        left->send(inputLeft);
        right->send(inputRight);
        auto receive = [](const std::shared_ptr<dai::MessageQueue>& queue) {
            bool timedOut = false;
            auto image = queue->get<dai::ImgFrame>(std::chrono::seconds(5), timedOut);
            REQUIRE_FALSE(timedOut);
            REQUIRE(image);
            return image;
        };
        auto actualLeft = receive(outputLeft);
        auto actualRight = receive(outputRight);
        const bool bypass = stage == 0 && !resize;
        REQUIRE((actualLeft == inputLeft) == bypass);
        REQUIRE((actualRight == inputRight) == bypass);
        REQUIRE(actualLeft->getWidth() == (resize ? 32 : 64));
        REQUIRE(actualLeft->getHeight() == (resize ? 24 : 48));
        REQUIRE(receive(passthroughLeft) == inputLeft);
        REQUIRE(receive(passthroughRight) == inputRight);
    }
    pipeline.stop();
}
#endif
