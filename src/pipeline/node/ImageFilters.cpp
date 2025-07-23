#include "depthai/pipeline/node/ImageFilters.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <utility/ErrorMacros.hpp>

#include "depthai/depthai.hpp"
#include "pipeline/ThreadedNodeImpl.hpp"
#include "pipeline/datatype/ImageFiltersConfig.hpp"

namespace dai {
namespace node {

namespace {

class Filter {
   public:
    virtual void process(std::shared_ptr<dai::ImgFrame>& frame) = 0;
    virtual void setParams(const FilterParams& params) = 0;
    virtual std::string getName() const = 0;
    virtual ~Filter() = default;
};

namespace impl {

class MedianFilter {
   public:
    MedianFilter();
    ~MedianFilter();
    int Init();

    void process(std::shared_ptr<dai::ImgFrame>& frame, int medianSize);
};

struct SpatialFilterParamsImpl {
    std::shared_ptr<dai::ImgFrame> currentFrame = {};

    float alpha = 0.5f;
    uint8_t delta = 20;
    int iterationNr = 2;
    int holesFillingRadius = 0;
};

class SpatialFilter {
   public:
    SpatialFilter();
    ~SpatialFilter();
    int Init(float alpha, int delta, int iterationNr, int holesFillingRadius);

    void process(std::shared_ptr<dai::ImgFrame>& frame);

   private:
    SpatialFilterParamsImpl params = {};
};

class SpeckleFilter {
   public:
    SpeckleFilter();
    ~SpeckleFilter();
    int Init();

    void process(std::shared_ptr<dai::ImgFrame>& frame, int speckleRange, int maxDiff);
};

constexpr const size_t PERSISTENCY_LUT_SIZE = 256;

struct TemporalFilterParamsImpl {
    std::shared_ptr<dai::ImgFrame> currentFrame = {};
    std::shared_ptr<std::vector<uint8_t>> accumulatorFrame = nullptr;
    std::shared_ptr<std::vector<uint8_t>> history = nullptr;
    std::shared_ptr<std::vector<uint8_t>> persistenceMap = nullptr;

    float alpha = 0.4f;  // The normalized weight of the current pixel
    uint8_t delta = 20;  // A threshold when a filter is invoked
    uint8_t currFrameIdx = 0;
};

class TemporalFilter {
   public:
    TemporalFilter();
    ~TemporalFilter();
    int Init(size_t frameSize, float alpha, int delta, int persistenceMode);

    void process(std::shared_ptr<dai::ImgFrame>& frame);

   private:
    struct MemSections {
        std::shared_ptr<std::vector<uint8_t>> mem;
        size_t size = 0;
    };

    int allocateBuffers(int frameSize);
    void buildPersistenceMap();
    uint8_t persistenceMode = 255;  // default value: "invalid"
    uint8_t currFrameIdx = 0;

    TemporalFilterParamsImpl params = {};

    MemSections rawAccumulatorFrame = {};  // Hold the last frame received for the current profile
    MemSections rawHistoryFrame = {};      // represents the history over the last 8 frames, 1 bit per frame
    // encodes whether a particular 8 bit history is good enough for all 8 phases of storage
    MemSections rawPersistenceMapLUT = {};  // Lookup table if persistency is enabled
};

/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
MedianFilter::MedianFilter() {}

int MedianFilter::Init() {
    return 0;
}

MedianFilter::~MedianFilter() {}

void MedianFilter::process(std::shared_ptr<dai::ImgFrame>& frame, int medianSize) {
    // Median filter only supports RAW16 and RAW8
    int opencvType;
    ImgFrame::Type frameType = frame->getType();
    if(frameType == ImgFrame::Type::RAW8) {
        opencvType = CV_8UC1;
    } else if(frameType == ImgFrame::Type::RAW16) {
        opencvType = CV_16UC1;
    } else {
        DAI_CHECK_V(false, "Unsupported frame type. Supported types are RAW16 and RAW8.");
    }

    // Parameter check
    DAI_CHECK_V(medianSize % 2 == 1, "Median filter size must be odd, got %d", medianSize);
    DAI_CHECK_V(medianSize <= 5, "Median filter size must be <= 5, got %d", medianSize);

    // Convenience wrapper for the frame data - not a copy
    cv::Mat cvFrame = cv::Mat(frame->getHeight(), frame->getWidth(), opencvType, frame->data->getData().data());

    // Apply filter
    cv::Mat cvFrameFiltered = cv::Mat(frame->getHeight(), frame->getWidth(), opencvType);
    cv::medianBlur(cvFrame, cvFrameFiltered, medianSize);

    // Copy data back to frame
    std::memcpy(frame->data->getData().data(), cvFrameFiltered.data, cvFrameFiltered.total() * cvFrameFiltered.elemSize());
}

/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/

template <typename T>
void recursiveFilterHorizontal(SpatialFilterParamsImpl* params) {
    void* image_data = (void*)params->currentFrame->data->getData().data();
    float alpha = params->alpha;
    int _width = params->currentFrame->getWidth();
    int _height = params->currentFrame->getHeight();
    size_t _holesFillingRadius = params->holesFillingRadius;

    // Handle conversions for invalid input data
    bool fp = (std::is_floating_point<T>::value);

    // Filtering integer values requires round-up to the nearest discrete value
    const float round = fp ? 0.f : 0.5f;
    // define invalid inputs
    const T valid_threshold = fp ? static_cast<T>(std::numeric_limits<T>::epsilon()) : static_cast<T>(1);
    const T deltaZ = static_cast<T>(params->delta);

    auto image = reinterpret_cast<T*>(image_data);
    size_t currentFill = 0;

    for(int v = 0; v < _height; v++) {
        // left to right
        T* im = image + v * _width;
        T val0 = im[0];
        currentFill = 0;

        for(int u = 1; u < _width - 1; u++) {
            T val1 = im[1];

            if(fabs(val0) >= valid_threshold) {
                if(fabs(val1) >= valid_threshold) {
                    currentFill = 0;
                    T diff = static_cast<T>(fabs(val1 - val0));

                    if(diff >= valid_threshold && diff <= deltaZ) {
                        float filtered = val1 * alpha + val0 * (1.0f - alpha);
                        val1 = static_cast<T>(filtered + round);
                        im[1] = val1;
                    }
                } else  // Only the old value is valid - appy holes filling
                {
                    if(_holesFillingRadius) {
                        if(++currentFill < _holesFillingRadius) im[1] = val1 = val0;
                    }
                }
            }

            val0 = val1;
            im += 1;
        }

        // right to left
        im = image + (v + 1) * _width - 2;  // end of row - two pixels
        T val1 = im[1];
        currentFill = 0;

        for(int u = _width - 1; u > 0; u--) {
            T val0 = im[0];

            if(val1 >= valid_threshold) {
                if(val0 > valid_threshold) {
                    currentFill = 0;
                    T diff = static_cast<T>(fabs(val1 - val0));

                    if(diff <= deltaZ) {
                        float filtered = val0 * alpha + val1 * (1.0f - alpha);
                        val0 = static_cast<T>(filtered + round);
                        im[0] = val0;
                    }
                } else  // 'inertial' hole filling
                {
                    if(_holesFillingRadius) {
                        if(++currentFill < _holesFillingRadius) im[0] = val0 = val1;
                    }
                }
            }

            val1 = val0;
            im -= 1;
        }
    }
}

template <typename T>
void recursiveFilterVertical(SpatialFilterParamsImpl* params) {
    void* image_data = (void*)params->currentFrame->data->getData().data();
    float alpha = params->alpha;
    int _width = params->currentFrame->getWidth();
    int _height = params->currentFrame->getHeight();

    // Handle conversions for invalid input data
    bool fp = (std::is_floating_point<T>::value);

    // Filtering integer values requires round-up to the nearest discrete value
    const float round = fp ? 0.f : 0.5f;
    // define invalid range
    const T valid_threshold = fp ? static_cast<T>(std::numeric_limits<T>::epsilon()) : static_cast<T>(1);
    const T deltaZ = static_cast<T>(params->delta);

    auto image = reinterpret_cast<T*>(image_data);

    // we'll do one row at a time, top to bottom, then bottom to top

    // top to bottom

    T* im = image;
    T im0{};
    T imw{};
    for(int v = 1; v < _height; v++) {
        for(int u = 0; u < _width; u++) {
            im0 = im[0];
            imw = im[_width];

            // if ((fabs(im0) >= valid_threshold) && (fabs(imw) >= valid_threshold))
            {
                T diff = static_cast<T>(fabs(im0 - imw));
                if(diff < deltaZ) {
                    float filtered = imw * alpha + im0 * (1.f - alpha);
                    im[_width] = static_cast<T>(filtered + round);
                }
            }
            im += 1;
        }
    }

    // bottom to top
    im = image + (_height - 2) * _width;
    for(int v = 1; v < _height; v++, im -= (_width * 2)) {
        for(int u = 0; u < _width; u++) {
            im0 = im[0];
            imw = im[_width];

            if((fabs(im0) >= valid_threshold) && (fabs(imw) >= valid_threshold)) {
                T diff = static_cast<T>(fabs(im0 - imw));
                if(diff < deltaZ) {
                    float filtered = im0 * alpha + imw * (1.f - alpha);
                    im[0] = static_cast<T>(filtered + round);
                }
            }
            im += 1;
        }
    }
}

SpatialFilter::SpatialFilter() {}

int SpatialFilter::Init(float alpha, int delta, int iterationNr, int holesFillingRadius) {
    if(alpha != params.alpha) {
        params.alpha = alpha;
    }
    if(delta != params.delta) {
        params.delta = delta;
    }
    if(iterationNr != params.iterationNr) {
        params.iterationNr = iterationNr;
    }
    if(holesFillingRadius != params.holesFillingRadius) {
        params.holesFillingRadius = holesFillingRadius;
    }

    return 0;
}

SpatialFilter::~SpatialFilter() {}

void SpatialFilter::process(std::shared_ptr<dai::ImgFrame>& frame) {
    params.currentFrame = frame;
    ImgFrame::Type frameType = frame->getType();
    if(frameType == ImgFrame::Type::RAW16) {
        for(int i = 0; i < params.iterationNr; i++) {
            recursiveFilterHorizontal<uint16_t>(&params);
            recursiveFilterVertical<uint16_t>(&params);
        }
    } else if(frameType == ImgFrame::Type::RAW8) {
        for(int i = 0; i < params.iterationNr; i++) {
            recursiveFilterHorizontal<uint8_t>(&params);
            recursiveFilterVertical<uint8_t>(&params);
        }
    } else {
        DAI_CHECK_V(false, "Unsupported frame type. Supported types are RAW16 and RAW8.");
    }
}

/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/

SpeckleFilter::SpeckleFilter() {}

int SpeckleFilter::Init() {
    return 0;
}

SpeckleFilter::~SpeckleFilter() {}

void SpeckleFilter::process(std::shared_ptr<dai::ImgFrame>& frame, int speckleRange, int maxDiff) {
    int opencvType;
    ImgFrame::Type frameType = frame->getType();
    if(frameType == ImgFrame::Type::RAW8) {
        opencvType = CV_8UC1;
    } else if(frameType == ImgFrame::Type::RAW16) {
        opencvType = CV_16SC1;
    } else {
        DAI_CHECK_V(false, "Unsupported frame type. Supported types are RAW16 and RAW8.");
    }
    cv::Mat cvFrame = cv::Mat(frame->getHeight(), frame->getWidth(), opencvType, frame->data->getData().data());
    cv::filterSpeckles(cvFrame, 0, speckleRange, maxDiff);
}

/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/

template <typename T>
void processImpl(TemporalFilterParamsImpl* params) {
    auto& currentFrame = params->currentFrame;
    auto& accumulatorFrame = params->accumulatorFrame;
    auto& history = params->history;
    float alpha = params->alpha;
    uint8_t delta = params->delta;
    uint8_t currFrameIdx = params->currFrameIdx;
    auto& persistenceMap = params->persistenceMap;
    static_assert((std::is_arithmetic<T>::value), "temporal filter assumes numeric types");

    T deltaZ = static_cast<T>(delta);

    auto frame = reinterpret_cast<T*>(currentFrame->data->getData().data());
    auto _lastFrame = reinterpret_cast<T*>(accumulatorFrame->data());
    unsigned char mask = 1 << currFrameIdx;

    size_t frameSize = currentFrame->getWidth() * currentFrame->getHeight();

    decltype(alpha) oneMinusAlpha = 1 - alpha;

    // pass one -- go through image and update all
    for(size_t i = 0; i < frameSize; i++) {
        T currentVal = frame[i];
        T previousVal = _lastFrame[i];

        if(currentVal) {
            if(!previousVal) {
                _lastFrame[i] = currentVal;
                (*history)[i] = mask;
            } else {  // old and new val
                T diff = static_cast<T>(fabs(currentVal - previousVal));

                if(diff < deltaZ) {  // old and new val agree
                    (*history)[i] |= mask;
                    float filtered = alpha * currentVal + oneMinusAlpha * previousVal;
                    T result = static_cast<T>(filtered);
                    frame[i] = result;
                    _lastFrame[i] = result;
                } else {
                    _lastFrame[i] = currentVal;
                    (*history)[i] = mask;
                }
            }
        } else {               // no currentVal
            if(previousVal) {  // only case we can help
                unsigned char hist = (*history)[i];
                unsigned char classification = (*persistenceMap)[hist];
                if(classification & mask) {  // we have had enough samples lately
                    frame[i] = previousVal;
                }
            }
            (*history)[i] &= ~mask;
        }
    }
}

TemporalFilter::TemporalFilter() {}

int TemporalFilter::Init(size_t frameSize, float alpha, int delta, int _persistenceMode) {
    bool resetBuffers = false;
    // could be lazy loading too, avoided by choice
    int err = allocateBuffers(frameSize);
    if(err) {
        return err;
    }

    if(this->persistenceMode != _persistenceMode) {
        this->persistenceMode = _persistenceMode;
        buildPersistenceMap();
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
        std::fill(rawAccumulatorFrame.mem->begin(), rawAccumulatorFrame.mem->end(), 0);
        std::fill(rawHistoryFrame.mem->begin(), rawHistoryFrame.mem->end(), 0);
    }

    params.history = rawHistoryFrame.mem;
    params.persistenceMap = rawPersistenceMapLUT.mem;

    return 0;
}

TemporalFilter::~TemporalFilter() {}

void TemporalFilter::process(std::shared_ptr<dai::ImgFrame>& frame) {
    params.currentFrame = frame;
    params.currFrameIdx = currFrameIdx;
    params.accumulatorFrame = rawAccumulatorFrame.mem;

    ImgFrame::Type frameType = frame->getType();
    if(frameType == ImgFrame::Type::RAW16) {
        processImpl<uint16_t>(&params);
    } else if(frameType == ImgFrame::Type::RAW8) {
        processImpl<uint8_t>(&params);
    } else {
        DAI_CHECK_V(false, "Unsupported frame type. Supported types are RAW16 and RAW8.");
    }

    currFrameIdx = (currFrameIdx + 1) % 8;  // at end of cycle
}

int TemporalFilter::allocateBuffers(int frameSize) {
    if(rawAccumulatorFrame.mem) {
        if(rawAccumulatorFrame.size < static_cast<size_t>(frameSize)) {
            rawAccumulatorFrame.mem.reset();
        }
    }
    if(!rawAccumulatorFrame.mem) {
        rawAccumulatorFrame.size = frameSize;
        rawAccumulatorFrame.mem = std::make_shared<std::vector<uint8_t>>(rawAccumulatorFrame.size);
        if(!rawAccumulatorFrame.mem) {
            return __LINE__;
        }
        std::fill(rawAccumulatorFrame.mem->begin(), rawAccumulatorFrame.mem->end(), 0);
    }

    if(rawHistoryFrame.mem) {
        if(rawHistoryFrame.size < static_cast<size_t>(frameSize)) {
            rawHistoryFrame.mem.reset();
        }
    }
    if(!rawHistoryFrame.mem) {
        rawHistoryFrame.size = frameSize;
        rawHistoryFrame.mem = std::make_shared<std::vector<uint8_t>>(rawHistoryFrame.size);
        if(!rawHistoryFrame.mem) {
            return __LINE__;
        }
        std::fill(rawHistoryFrame.mem->begin(), rawHistoryFrame.mem->end(), 0);
    }

    if(rawPersistenceMapLUT.mem) {
        if(rawPersistenceMapLUT.size < static_cast<size_t>(PERSISTENCY_LUT_SIZE)) {
            rawPersistenceMapLUT.mem.reset();
        }
    }

    if(!rawPersistenceMapLUT.mem) {
        rawPersistenceMapLUT.size = PERSISTENCY_LUT_SIZE;
        rawPersistenceMapLUT.mem = std::make_shared<std::vector<uint8_t>>(rawPersistenceMapLUT.size);
        if(!rawPersistenceMapLUT.mem) {
            return __LINE__;
        }
        std::fill(rawPersistenceMapLUT.mem->begin(), rawPersistenceMapLUT.mem->end(), 0);
    }

    return 0;
}

void TemporalFilter::buildPersistenceMap() {
    auto& persistenceMapLUT = *rawPersistenceMapLUT.mem;

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
    std::copy(credible_threshold.begin(), credible_threshold.end(), rawPersistenceMapLUT.mem->begin());
}

}  // namespace impl

class MedianFilterWrapper : public Filter {
   public:
    MedianFilterWrapper(const MedianFilterParams& params) : params(params), medianFilter() {
        medianFilter.Init();
    }

    void process(std::shared_ptr<dai::ImgFrame>& frame) override {
        if(params != MedianFilterParams::MEDIAN_OFF) {
            const int medianSize = static_cast<int>(params);
            medianFilter.process(frame, medianSize);
        }
    }

    void setParams(const FilterParams& params) override {
        if(std::holds_alternative<MedianFilterParams>(params)) {
            this->params = std::get<MedianFilterParams>(params);
        } else {
            DAI_CHECK_V(false, "Invalid filter params. Expected MedianFilterParams, got {}", params.index());
        }
    }

    std::string getName() const override {
        return "MedianFilter(" + nlohmann::json(params).dump() + ")";
    }

   private:
    MedianFilterParams params;
    impl::MedianFilter medianFilter;
};

class SpatialFilterWrapper : public Filter {
   public:
    SpatialFilterWrapper(const SpatialFilterParams& params) : params(params), spatialFilter() {
        const float alpha = params.alpha;
        const int delta = params.delta;
        const int iterationNr = params.numIterations;
        const int holesFillingRadius = params.holeFillingRadius;
        spatialFilter.Init(alpha, delta, iterationNr, holesFillingRadius);
    }

    void process(std::shared_ptr<dai::ImgFrame>& frame) override {
        if(params.enable) {
            spatialFilter.process(frame);
        }
    }

    void setParams(const FilterParams& params) override {
        if(std::holds_alternative<SpatialFilterParams>(params)) {
            this->params = std::get<SpatialFilterParams>(params);
        } else {
            DAI_CHECK_V(false, "Invalid filter params. Expected SpatialFilterParams, got {}", params.index());
        }
    }

    std::string getName() const override {
        return "SpatialFilter(" + nlohmann::json(params).dump() + ")";
    }

   private:
    SpatialFilterParams params;
    impl::SpatialFilter spatialFilter;
};

class SpeckleFilterWrapper : public Filter {
   public:
    SpeckleFilterWrapper(const SpeckleFilterParams& params) : params(params), speckleFilter() {
        speckleFilter.Init();
    }

    void process(std::shared_ptr<dai::ImgFrame>& frame) override {
        if(params.enable) {
            const int speckleRange = params.speckleRange;
            const int maxDiff = params.differenceThreshold;
            speckleFilter.process(frame, speckleRange, maxDiff);
        }
    }

    void setParams(const FilterParams& params) override {
        if(std::holds_alternative<SpeckleFilterParams>(params)) {
            this->params = std::get<SpeckleFilterParams>(params);
        } else {
            DAI_CHECK_V(false, "Invalid filter params. Expected SpeckleFilterParams, got {}", params.index());
        }
    }

    std::string getName() const override {
        return "SpeckleFilter(" + nlohmann::json(params).dump() + ")";
    }

   private:
    SpeckleFilterParams params;
    impl::SpeckleFilter speckleFilter;
};

class TemporalFilterWrapper : public Filter {
   private:
    bool isInitialized = false;
    TemporalFilterParams params;
    impl::TemporalFilter temporalFilter;

   public:
    TemporalFilterWrapper(const TemporalFilterParams& p) {
        params = p;
    }

    void process(std::shared_ptr<dai::ImgFrame>& frame) override {
        if(params.enable) {
            if(!isInitialized) {
                size_t bytesPerPixel;
                auto frameType = frame->getType();
                if(frameType == ImgFrame::Type::RAW16) {
                    bytesPerPixel = sizeof(uint16_t);
                } else if(frameType == ImgFrame::Type::RAW8) {
                    bytesPerPixel = sizeof(uint8_t);
                } else {
                    DAI_CHECK_V(false, "Unsupported frame type. Supported types are RAW8 and RAW16.");
                }
                const size_t frameSize = frame->getHeight() * frame->getWidth() * bytesPerPixel;
                const float alpha = params.alpha;
                const int delta = params.delta;
                const int persistencyMode = static_cast<int>(params.persistencyMode);
                int ret = temporalFilter.Init(frameSize, alpha, delta, persistencyMode);
                DAI_CHECK_V(ret == 0, "Failed to initialize");
                isInitialized = true;
            }

            temporalFilter.process(frame);
        }
    }

    void setParams(const FilterParams& params) override {
        if(std::holds_alternative<TemporalFilterParams>(params)) {
            this->params = std::get<TemporalFilterParams>(params);
            isInitialized = false;
        } else {
            DAI_CHECK_V(false, "Invalid filter params. Expected TemporalFilterParams, got {}", params.index());
        }
    }

    std::string getName() const override {
        return "TemporalFilter(" + nlohmann::json(params).dump() + ")";
    }
};

std::unique_ptr<Filter> createFilter(const MedianFilterParams& params) {
    return std::make_unique<MedianFilterWrapper>(params);
}

std::unique_ptr<Filter> createFilter(const SpatialFilterParams& params) {
    return std::make_unique<SpatialFilterWrapper>(params);
}

std::unique_ptr<Filter> createFilter(const SpeckleFilterParams& params) {
    return std::make_unique<SpeckleFilterWrapper>(params);
}

std::unique_ptr<Filter> createFilter(const TemporalFilterParams& params) {
    return std::make_unique<TemporalFilterWrapper>(params);
}

std::unique_ptr<Filter> createFilter(const FilterParams& params) {
    return std::visit([](auto&& arg) -> std::unique_ptr<Filter> { return createFilter(arg); }, params);
}

}  // namespace

std::shared_ptr<ImageFilters> ImageFilters::build(Node::Output& input, ImageFiltersPresetMode presetMode) {
    input.link(this->input);
    setDefaultProfilePreset(presetMode);
    return std::static_pointer_cast<ImageFilters>(shared_from_this());
}

std::shared_ptr<ImageFilters> ImageFilters::build(ImageFiltersPresetMode presetMode) {
    setDefaultProfilePreset(presetMode);
    return std::static_pointer_cast<ImageFilters>(shared_from_this());
}

void ImageFilters::run() {
    // A vector of filters
    std::vector<std::unique_ptr<Filter>> filters;

    // A helper function to create a new pipeline
    auto createNewFilterPipeline = [&filters](const ImageFiltersConfig& config) {
        filters.clear();
        for(const auto& params : config.filterParams) {
            filters.push_back(createFilter(params));
        }
    };

    // A helper function to update an existing pipeline
    auto& logger = pimpl->logger;
    auto updateExistingFilterPipeline = [&filters, &logger](const ImageFiltersConfig& config) {
        for(size_t i = 0; i < config.filterIndices.size(); i++) {
            const auto& index = config.filterIndices[i];
            const auto& params = config.filterParams[i];

            if(index >= 0 && index < static_cast<int>(filters.size())) {
                filters[index]->setParams(params);
            } else {
                logger->warn("ImageFilters: Invalid filter index: {}. Should be in range [0, {})", index, filters.size());
            }
        }
    };

    // A helper function to get the string representation of the filter pipeline
    auto getFilterPipelineString = [&filters]() {
        std::stringstream ss;
        for(const auto& filter : filters) {
            ss << filter->getName() << " | ";
        }
        return ss.str();
    };

    // Create new filter pipeline
    auto& properties = getProperties();
    auto& initialConfig = properties.initialConfig;
    DAI_CHECK_V(initialConfig.filterIndices.size() == 0, "Initial config must describe a new filter pipeline, not an update to it.");
    createNewFilterPipeline(initialConfig);

    logger->debug("ImageFilters: Created a new filter pipeline with {} filters. Is passthrough: {}. Current filter pipeline: {}",
                  filters.size(),
                  filters.size() == 0,
                  getFilterPipelineString());

    while(isRunning()) {
        // Set config
        while(inputConfig.has()) {
            auto configMsg = inputConfig.get<ImageFiltersConfig>();
            bool isUpdate = configMsg->filterIndices.size() > 0;
            if(isUpdate) {
                updateExistingFilterPipeline(*configMsg);
                logger->debug("ImageFilters: Updating existing filter pipeline. New pipeline is {}", getFilterPipelineString());
            } else {
                createNewFilterPipeline(*configMsg);
                logger->debug("ImageFilters: Creating a new filter pipeline. New pipeline is {}", getFilterPipelineString());
            }
        }

        // Get frame from input queue
        std::shared_ptr<dai::ImgFrame> frame = input.get<dai::ImgFrame>();
        if(frame == nullptr) {
            logger->error("ImageFilters: Input frame is nullptr");
            break;
        }

        // If there are no filters, serve as a passthrough
        // Otherwise, create a copy and run filters inplace on the copy
        std::shared_ptr<dai::ImgFrame> filteredFrame = filters.size() == 0 ? frame : frame->clone();

        auto t1 = std::chrono::high_resolution_clock::now();
        for(const auto& filter : filters) {
            filter->process(filteredFrame);  // inplace filter
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        static auto tlast = t2;
        if(t2 - tlast > std::chrono::milliseconds(5000)) {
            pimpl->logger->debug("ImageFilters: Time taken: {}ms", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000);
            tlast = t2;
        }

        // Send filtered frame to the output queue
        output.send(filteredFrame);
    }
}

bool ImageFilters::runOnHost() const {
    return runOnHostVar;
}

void ImageFilters::setRunOnHost(bool runOnHost) {
    if(device && device->getPlatform() == Platform::RVC2 && !runOnHost) {
        DAI_CHECK_V(false, "ImageFilters: Running on device is not supported on RVC2");
    }
    runOnHostVar = runOnHost;
}

void ImageFilters::setDefaultProfilePreset(ImageFiltersPresetMode mode) {
    initialConfig->setProfilePreset(mode);
}

std::shared_ptr<ToFDepthConfidenceFilter> ToFDepthConfidenceFilter::build(Node::Output& depth, Node::Output& amplitude, ImageFiltersPresetMode presetMode) {
    depth.link(this->depth);
    amplitude.link(this->amplitude);
    setDefaultProfilePreset(presetMode);
    return std::static_pointer_cast<ToFDepthConfidenceFilter>(shared_from_this());
}

std::shared_ptr<ToFDepthConfidenceFilter> ToFDepthConfidenceFilter::build(ImageFiltersPresetMode presetMode) {
    setDefaultProfilePreset(presetMode);
    return std::static_pointer_cast<ToFDepthConfidenceFilter>(shared_from_this());
}

void ToFDepthConfidenceFilter::applyDepthConfidenceFilter(std::shared_ptr<ImgFrame> depthFrame,
                                                          std::shared_ptr<ImgFrame> amplitudeFrame,
                                                          std::shared_ptr<ImgFrame> filteredDepthFrame,
                                                          std::shared_ptr<ImgFrame> confidenceFrame,
                                                          float threshold) {
    auto getCvType = [](ImgFrame::Type type) -> int {
        if(type == ImgFrame::Type::RAW8) {
            return CV_8UC1;
        }
        if(type == ImgFrame::Type::RAW16) {
            return CV_16UC1;
        }
        DAI_CHECK_V(false, "DepthConfidenceFilter: Unsupported frame type. Supported types are RAW8 and RAW16.");
    };

    const int height = depthFrame->getHeight();
    const int width = depthFrame->getWidth();

    // OpenCV wrappers for easier frame manipulation
    cv::Mat depth(height, width, getCvType(depthFrame->getType()), depthFrame->getData().data());
    cv::Mat amplitude(height, width, getCvType(amplitudeFrame->getType()), amplitudeFrame->getData().data());
    cv::Mat filteredDepth(height, width, getCvType(filteredDepthFrame->getType()), filteredDepthFrame->getData().data());
    cv::Mat confidence(height, width, getCvType(confidenceFrame->getType()), confidenceFrame->getData().data());

    // Convert depth and amplitude values to float32 = common type
    cv::Mat depthFloat, amplitudeFloat;
    depth.convertTo(depthFloat, CV_32F);
    amplitude.convertTo(amplitudeFloat, CV_32F);

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            const float a = amplitudeFloat.at<float>(i, j);
            const float d = depthFloat.at<float>(i, j);

            // Avoid division by zero or very small depth values
            float conf;
            if(a < 1e-12f || d < 1e-12f) {
                conf = static_cast<float>(std::numeric_limits<std::uint16_t>::max());
            } else {
                // Corrected formula: amplitude / sqrt(depth / 2.0)
                // Higher amplitude --> higher confidence
                // Higher depth --> slightly lower confidence
                conf = a / std::sqrt(d / 2.0f);
            }
            conf = conf * 100;

            confidence.at<std::uint16_t>(i, j) = static_cast<std::uint16_t>(conf);

            // Invalidate pixel if confidence is below threshold
            if(conf < threshold) {
                filteredDepth.at<std::uint16_t>(i, j) = 0;
            } else {
                filteredDepth.at<std::uint16_t>(i, j) = static_cast<std::uint16_t>(d);
            }
        }
    }
}

void ToFDepthConfidenceFilter::run() {
    auto confidenceThreshold = getProperties().initialConfig.confidenceThreshold;
    while(isRunning()) {
        // Update threshold dynamically
        while(inputConfig.has()) {
            auto configMsg = inputConfig.get<ToFDepthConfidenceFilterConfig>();
            confidenceThreshold = configMsg->confidenceThreshold;
        }

        // Get frames from input queue
        std::shared_ptr<dai::ImgFrame> depthFrame = depth.get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> amplitudeFrame = amplitude.get<dai::ImgFrame>();
        if(depthFrame == nullptr || amplitudeFrame == nullptr) {
            pimpl->logger->error("DepthConfidenceFilter: Input frame is nullptr");
            break;
        }

        // In case the confidence threshold is 0, serve as a passthrough
        if(confidenceThreshold == 0.0f) {
            filteredDepth.send(depthFrame);
            confidence.send(amplitudeFrame);
            continue;
        }

        // Create empty output frames
        auto filteredDepthFrame = std::make_shared<dai::ImgFrame>();
        auto confidenceFrame = std::make_shared<dai::ImgFrame>();

        // Copy metadata from input frames
        filteredDepthFrame->setMetadata(depthFrame);
        confidenceFrame->setMetadata(depthFrame);

        filteredDepthFrame->setType(ImgFrame::Type::RAW16);
        confidenceFrame->setType(ImgFrame::Type::RAW16);

        filteredDepthFrame->data->setSize(depthFrame->getData().size() * depthFrame->getBytesPerPixel());
        confidenceFrame->data->setSize(depthFrame->getData().size() * depthFrame->getBytesPerPixel());

        // Apply filter
        auto t1 = std::chrono::high_resolution_clock::now();
        applyDepthConfidenceFilter(depthFrame, amplitudeFrame, filteredDepthFrame, confidenceFrame, confidenceThreshold);
        auto t2 = std::chrono::high_resolution_clock::now();

        static auto tlast = t2;
        if(t2 - tlast > std::chrono::milliseconds(5000)) {
            pimpl->logger->debug("DepthConfidenceFilter: Time taken: {}ms", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() / 1000);
            tlast = t2;
        }

        // Send results
        filteredDepth.send(filteredDepthFrame);
        confidence.send(confidenceFrame);
    }
}

void ToFDepthConfidenceFilter::setRunOnHost(bool runOnHost) {
    if(device && device->getPlatform() == Platform::RVC2 && !runOnHost) {
        DAI_CHECK_V(false, "DepthConfidenceFilter: Running on device is not supported on RVC2");
    }
    runOnHostVar = runOnHost;
}

bool ToFDepthConfidenceFilter::runOnHost() const {
    return runOnHostVar;
}

void ToFDepthConfidenceFilter::setDefaultProfilePreset(ImageFiltersPresetMode mode) {
    initialConfig->setProfilePreset(mode);
}

}  // namespace node
}  // namespace dai
