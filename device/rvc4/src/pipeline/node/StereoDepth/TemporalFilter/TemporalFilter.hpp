#pragma once
#include <cstdint>

#include <cstdint>
#include <cstdlib>
#include <vector>
#include <array>
#include <cmath>
#include "TemporalFilterParams.h"

namespace dai
{

    constexpr const size_t PERSISTENCY_LUT_SIZE = 256;

    template<typename T>
    class TemporalFilter
    {
    public:
        TemporalFilter();
        ~TemporalFilter();
        int Init(size_t frameSize, float alpha, int delta, int persistenceMode);

        void process(uint8_t* currentFrame, size_t frameHeight, size_t frameWidth);

    private:
        struct MemSections
        {
            uint8_t *mem = nullptr;
            size_t size = 0;
        };

        int allocateBuffers(int frameSize);
        void buildPersistanceMap();
        uint8_t persistenceMode = 255; //default value: "invalid"
        uint8_t currFrameIdx = 0;

        ::TemporalFilterParams params = {};

        MemSections rawAccumulatorFrame = {}; // Hold the last frame received for the current profile
        MemSections rawHistoryFrame = {};     // represents the history over the last 8 frames, 1 bit per frame
        MemSections rawPersistenceMapLUT = {};  // Lookup table if persitency is enabled

        bool measureTime = false;

    };

}
