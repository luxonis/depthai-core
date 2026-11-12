#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <vector>
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/AprilTag.hpp"
#include "depthai/pipeline/datatype/AprilTags.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
#include <common/image_u8.h>
}

int main() {
    try {
        auto* family = tag36h11_create();
        auto* tag = apriltag_to_image(family, 0);
        constexpr int scale = 20;
        constexpr int border = 40;
        const int width = tag->width * scale + border * 2;
        const int height = tag->height * scale + border * 2;
        std::vector<uint8_t> pixels(width * height, 255);
        for(int y = 0; y < tag->height; ++y)
            for(int x = 0; x < tag->width; ++x)
                for(int dy = 0; dy < scale; ++dy)
                    for(int dx = 0; dx < scale; ++dx)
                        pixels[(border + y * scale + dy) * width + border + x * scale + dx] = tag->buf[y * tag->stride + x];
        image_u8_destroy(tag);
        tag36h11_destroy(family);
        for(int threads : {1, 2}) {
            dai::Pipeline pipeline(false);
            auto node = pipeline.create<dai::node::AprilTag>();
            node->setNumThreads(threads);
            node->initialConfig->setFamily(dai::AprilTagConfig::Family::TAG_36H11);
            auto input = node->inputImage.createInputQueue();
            auto output = node->out.createOutputQueue();
            pipeline.start();
            if(!node->runOnHost()) throw std::runtime_error("Expected host execution");
            for(int iteration = 0; iteration < 3; ++iteration) {
                auto frame = std::make_shared<dai::ImgFrame>();
                frame->setType(dai::ImgFrame::Type::GRAY8);
                frame->setWidth(width);
                frame->setHeight(height);
                frame->setData(pixels);
                frame->setSequenceNum(iteration + 42);
                frame->setTimestamp(std::chrono::steady_clock::now());
                input->send(frame);
                bool timedOut = false;
                auto tags = output->get<dai::AprilTags>(std::chrono::seconds(10), timedOut);
                if(timedOut || !tags) throw std::runtime_error("No detection output before timeout");
                if(tags->aprilTags.size() != 1 || tags->aprilTags[0].id != 0 || tags->getSequenceNum() != iteration + 42)
                    throw std::runtime_error("Incorrect tag ID, detection count, or sequence number");
                std::printf("PASS actual DepthAI node: threads=%d iteration=%d tag_id=%d sequence=%lld\n", threads, iteration, tags->aprilTags[0].id, static_cast<long long>(tags->getSequenceNum()));
            }
            pipeline.stop();
            pipeline.wait();
        }
        return 0;
    } catch(const std::exception& error) {
        std::fprintf(stderr, "FAIL actual DepthAI node: %s\n", error.what());
        return 1;
    }
}
