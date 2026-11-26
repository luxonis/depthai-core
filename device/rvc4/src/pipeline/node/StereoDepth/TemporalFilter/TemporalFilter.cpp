#include "TemporalFilter.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dai {

template <typename T>
void processImpl(::TemporalFilterParams* params) {
    auto& currentFrame = params->currentFrame;
    auto frameHeight = params->frameHeight;
    auto frameWidth = params->frameWidth;
    auto& accumulatorFrame = params->accumulatorFrame;
    uint8_t* history = params->history;
    float alpha = params->alpha;
    uint8_t delta = params->delta;
    uint8_t currFrameIdx = params->currFrameIdx;
    uint8_t* persistenceMap = params->persistenceMap;
    static_assert((std::is_arithmetic<T>::value), "temporal filter assumes numeric types");

    const bool fp = (std::is_floating_point<T>::value);

    T delta_z = static_cast<T>(delta);

    auto frame = reinterpret_cast<T*>(currentFrame);
    auto _last_frame = reinterpret_cast<T*>(accumulatorFrame);

    unsigned char mask = 1 << currFrameIdx;

    size_t frameSize = frameWidth * frameHeight;

    decltype(alpha) oneMinusAlpha = 1 - alpha;

    // pass one -- go through image and update all
    for(size_t i = 0; i < frameSize; i++) {
        T cur_val = frame[i];
        T prev_val = _last_frame[i];

        if(cur_val) {
            if(!prev_val) {
                _last_frame[i] = cur_val;
                history[i] = mask;
            } else {  // old and new val
                T diff = static_cast<T>(fabs(cur_val - prev_val));

                if(diff < delta_z) {  // old and new val agree
                    history[i] |= mask;
                    float filtered = alpha * cur_val + oneMinusAlpha * prev_val;
                    T result = static_cast<T>(filtered);
                    frame[i] = result;
                    _last_frame[i] = result;
                } else {
                    _last_frame[i] = cur_val;
                    history[i] = mask;
                }
            }
        } else {            // no cur_val
            if(prev_val) {  // only case we can help
                unsigned char hist = history[i];
                unsigned char classification = persistenceMap[hist];
                if(classification & mask) {  // we have had enough samples lately
                    frame[i] = prev_val;
                }
            }
            history[i] &= ~mask;
        }
    }
}

template<typename T>
TemporalFilter<T>::TemporalFilter() {}

template<typename T>
int TemporalFilter<T>::Init(size_t frameSize, float alpha, int delta, int _persistenceMode) {
    bool resetBuffers = false;
    // could be lazy loading too, avoided by choice
    int err = allocateBuffers(frameSize);
    if(err) {
        return err;
    }

    if(this->persistenceMode != _persistenceMode) {
        this->persistenceMode = _persistenceMode;
        buildPersistanceMap();
        resetBuffers = true;
    }
    if(alpha != params.alpha) {
        params.alpha = alpha;
        resetBuffers = true;
    }
    if(delta != params.delta) {
        params.delta = delta;
        resetBuffers = true;
    }

    if(resetBuffers) {
        currFrameIdx = 0;
        memset(rawAccumulatorFrame.mem, 0, rawAccumulatorFrame.size);
        memset(rawHistoryFrame.mem, 0, rawHistoryFrame.size);
    }

    params.history = rawHistoryFrame.mem;
    params.persistenceMap = rawPersistenceMapLUT.mem;

    return 0;
}

template<typename T>
TemporalFilter<T>::~TemporalFilter() {
    if(rawAccumulatorFrame.mem) {
        free(rawAccumulatorFrame.mem);
    }

    if(rawHistoryFrame.mem) {
        free(rawHistoryFrame.mem);
    }

    if(rawPersistenceMapLUT.mem) {
        free(rawPersistenceMapLUT.mem);
    }
}

template<typename T>
void TemporalFilter<T>::process(uint8_t* currentFrame, size_t frameHeight, size_t frameWidth) {
    params.currentFrame = currentFrame;
    params.frameHeight = frameHeight;
    params.frameWidth = frameWidth;
    params.currFrameIdx = currFrameIdx;
    params.accumulatorFrame = rawAccumulatorFrame.mem;

    processImpl<T>(&params);

    currFrameIdx = (currFrameIdx + 1) % 8;  // at end of cycle
}

template<typename T>
int TemporalFilter<T>::allocateBuffers(int frameSize) {
    if(rawAccumulatorFrame.mem) {
        if(rawAccumulatorFrame.size < frameSize) {
            free(rawAccumulatorFrame.mem);
            rawAccumulatorFrame.mem = nullptr;
        }
    }
    if(rawAccumulatorFrame.mem == nullptr) {
        rawAccumulatorFrame.size = frameSize;
        rawAccumulatorFrame.mem = (uint8_t*)aligned_alloc(64, rawAccumulatorFrame.size);
        if(rawAccumulatorFrame.mem == nullptr) {
            return __LINE__;
        }
        memset(rawAccumulatorFrame.mem, 0, rawAccumulatorFrame.size);
    }

    if(rawHistoryFrame.mem) {
        if(rawHistoryFrame.size < frameSize) {
            free(rawHistoryFrame.mem);
            rawHistoryFrame.mem = nullptr;
        }
    }
    if(rawHistoryFrame.mem == nullptr) {
        rawHistoryFrame.size = frameSize;
        rawHistoryFrame.mem = (uint8_t*)aligned_alloc(64, rawHistoryFrame.size);
        if(rawHistoryFrame.mem == nullptr) {
            return __LINE__;
        }
        memset(rawHistoryFrame.mem, 0, rawHistoryFrame.size);
    }

    if(rawPersistenceMapLUT.mem == nullptr) {
        rawPersistenceMapLUT.size = PERSISTENCY_LUT_SIZE;
        rawPersistenceMapLUT.mem = (uint8_t*)aligned_alloc(64, rawPersistenceMapLUT.size);
        if(rawPersistenceMapLUT.mem == nullptr) {
            return __LINE__;
        }
        memset(rawPersistenceMapLUT.mem, 0, rawPersistenceMapLUT.size);
    }

    return 0;
}

template<typename T>
void TemporalFilter<T>::buildPersistanceMap() {
    uint8_t* persistenceMapLUT = rawPersistenceMapLUT.mem;

    for(size_t i = 0; i < PERSISTENCY_LUT_SIZE; i++) {
        persistenceMapLUT[i] = 0;
        unsigned char last_7 = !!(i & 1);  // old
        unsigned char last_6 = !!(i & 2);
        unsigned char last_5 = !!(i & 4);
        unsigned char last_4 = !!(i & 8);
        unsigned char last_3 = !!(i & 16);
        unsigned char last_2 = !!(i & 32);
        unsigned char last_1 = !!(i & 64);
        unsigned char accumulatorFrame = !!(i & 128);  // new

        if(persistenceMode == 1) {
            int sum = accumulatorFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
            if(sum >= 8)  // valid in eight of the last eight frames
                persistenceMapLUT[i] = 1;
        } else if(persistenceMode == 2)  // <--- default choice in current libRS implementation
        {
            int sum = accumulatorFrame + last_1 + last_2;
            if(sum >= 2)  // valid in two of the last three frames
                persistenceMapLUT[i] = 1;
        } else if(persistenceMode == 3)  // <--- default choice recommended
        {
            int sum = accumulatorFrame + last_1 + last_2 + last_3;
            if(sum >= 2)  // valid in two of the last four frames
                persistenceMapLUT[i] = 1;
        } else if(persistenceMode == 4) {
            int sum = accumulatorFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
            if(sum >= 2)  // valid in two of the last eight frames
                persistenceMapLUT[i] = 1;
        } else if(persistenceMode == 5) {
            int sum = accumulatorFrame + last_1;
            if(sum >= 1)  // valid in one of the last two frames
                persistenceMapLUT[i] = 1;
        } else if(persistenceMode == 6) {
            int sum = accumulatorFrame + last_1 + last_2 + last_3 + last_4;
            if(sum >= 1)  // valid in one of the last five frames
                persistenceMapLUT[i] = 1;
        } else if(persistenceMode == 7)  //  <--- most filling
        {
            int sum = accumulatorFrame + last_1 + last_2 + last_3 + last_4 + last_5 + last_6 + last_7;
            if(sum >= 1)  // valid in one of the last eight frames
                persistenceMapLUT[i] = 1;
        } else if(persistenceMode == 8)  //  <--- all 1's
        {
            persistenceMapLUT[i] = 1;
        } else  // all others, including 0, no persistance
        {
        }
    }

    // Convert to credible enough
    std::array<uint8_t, PERSISTENCY_LUT_SIZE> credible_threshold;
    credible_threshold.fill(0);

    for(auto phase = 0; phase < 8; phase++) {
        // evaluating last phase
        // int ephase = (phase + 7) % 8;
        unsigned char mask = 1 << phase;
        int i;

        for(i = 0; i < 256; i++) {
            unsigned char pos = (unsigned char)((i << (8 - phase)) | (i >> phase));
            if(persistenceMapLUT[pos]) credible_threshold[i] |= mask;
        }
    }
    // Store results
    assert(rawPersistenceMapLUT.size == PERSISTENCY_LUT_SIZE);
    memcpy(persistenceMapLUT, credible_threshold.data(), PERSISTENCY_LUT_SIZE);
}

// Explicit template instantiations for common types
template class TemporalFilter<uint8_t>;
template class TemporalFilter<uint16_t>;
template class TemporalFilter<float>;

}  // namespace dai
