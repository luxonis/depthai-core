#include "Warp.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <sstream>

// #include "WarpHelper.hpp"
#include "pipeline/NodeTiming.hpp"
#include "utilities/AcceleratorMemory.hpp"

template <typename T>
T ALIGN_UP(T value, uintptr_t align) {
    uintptr_t val = static_cast<uintptr_t>(value);
    uintptr_t SECTION_ALIGNMENT_SIZE = align;
    return ((((val) + (SECTION_ALIGNMENT_SIZE)-1)) & ~((SECTION_ALIGNMENT_SIZE)-1));
};

namespace dai {
namespace gate {

void printImgFrame(const dai::ImgFrame& img);

void Warp::run() {
    using namespace std::chrono;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");
    std::shared_ptr<ImgFrame> outImage;

    // Initialize the wrap module after first image received
    bool initialized = false;

    warpDispatcher = std::make_unique<WarpDispatcher>();

    std::shared_ptr<ImgFrame> inImage = nullptr;
    std::vector<double> rawGridX, rawGridY;

    if(!properties.meshUri.empty()) {
        auto meshResource = loadResource(properties.meshUri);
        logger->info("Mesh data read {}", meshResource.size());
        WarpDispatcher::ParseMeshResourceData(meshResource, properties.meshWidth, properties.meshHeight, rawGridX, rawGridY, true);
    } else {
        logger->error("No mesh data. Exit");
        return;
    }

    while(mainLoop()) {
        auto tAbsoluteBeginning = steady_clock::now();
        microseconds wastedTime = 0ms;

        logger->info("Waiting for image.");

        inImage = inputImage.get<ImgFrame>();
        auto tGotInput = steady_clock::now();
        {
            auto t1 = steady_clock::now();
            // Converts or just returns the same pointer if already AcceleratorMemory
            inImage->data = AcceleratorMemory::convert(inImage->data);
            auto t2 = steady_clock::now();
            wastedTime += duration_cast<microseconds>(t2 - t1);
            logger->info("Input image convert time: {}us", duration_cast<microseconds>(t2 - t1).count());
        }

        if(!initialized) {
            initialized = true;

            auto t1 = steady_clock::now();

            // Initialize the warp engine of the hardware accelerator
            warpDispatcher->Open();
            warpDispatcher->Init(properties, inImage->fb.width, inImage->fb.height, inImage->fb.type, inImage->fb.type);
            warpDispatcher->SetWarpMesh(rawGridX, rawGridY);

            auto t2 = steady_clock::now();
            logger->info("Initialize wrap time: {}us", duration_cast<microseconds>(t2 - t1).count());
        }

        logger->info("Running wrap on received image.");

        outImage = std::make_shared<ImgFrame>();
        outImage->data = dai::AcceleratorMemory::create(warpDispatcher->getMinOutImgSize());
        outImage->setBufferMetadataFrom(inImage);
        outImage->setSourceSize(inImage->fb.width, inImage->fb.height);
        outImage->setSize(properties.outputWidth, properties.outputHeight);
        outImage->setType(inImage->fb.type);

        {
            auto t1 = steady_clock::now();
            warpDispatcher->Run(inImage, outImage);
            auto t2 = steady_clock::now();
            logger->info("Input wrap time: {}us", duration_cast<microseconds>(t2 - t1).count());
        }

        // printf("IN IMAGE -----> ");
        // printImgFrame(*inImage);
        // printf("OUT IMAGE -----> ");
        // printImgFrame(*outImage);
        // printf("CONFIG -----> ");
        // printConfig(currentConfig);

        // Send image out
        auto tProcessed = steady_clock::now();
        out.send(outImage);
        logger->info("Image sent out.");
        auto tAbsoluteEnd = steady_clock::now();
        dai::gate::logTiming(logger, this->getName(), tAbsoluteBeginning, tGotInput, tProcessed, tAbsoluteEnd);
    }  // while running

    warpDispatcher->Close();
}

void printImgFrame(const dai::ImgFrame& img) {
    printf("fb: p1: 0x%08lX, w: %d, h: %d, stride: %d, planeStride: %d, Bpp: %d, type: %d\n",
           (uintptr_t)img.data.get()->getData().data(),
           img.fb.width,
           img.fb.height,
           img.fb.stride,
           0 /*img.fb.planeStride*/,
           img.fb.bytesPP,
           (int)img.fb.type);
}

}  // namespace gate
}  // namespace dai
