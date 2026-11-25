#pragma once
#include <cstdint>


struct TemporalFilterParams {
    uint8_t* currentFrame = nullptr;
    uint8_t* accumulatorFrame = nullptr;
    uint8_t* history = nullptr;
    uint8_t* persistenceMap = nullptr;

    float alpha = 0.4f; // The normalized weight of the current pixel
    uint8_t delta = 20; // A threshold when a filter is invoked
    uint8_t currFrameIdx = 0;

    std::size_t frameHeight = 0;
    std::size_t frameWidth = 0;
};
