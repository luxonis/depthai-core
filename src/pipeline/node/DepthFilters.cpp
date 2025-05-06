#include "depthai/pipeline/node/DepthFilters.hpp"

#include <cmath>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"
#include "pipeline/datatype/DepthFiltersConfig.hpp"

namespace dai {
namespace node {

namespace {

// NOTE: This is a copy-pasted implementation from the FW codebase
// with only minor changes to account for all possible edge cases
namespace impl {

class MedianFilter {
   public:
    MedianFilter();
    ~MedianFilter();
    int Init();

    void process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int medianSize);
};

struct SpatialFilterParams {
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

    void process(std::shared_ptr<dai::ImgFrame>& disparityFrame);

   private:
    SpatialFilterParams params = {};
};

class SpeckleFilter {
   public:
    SpeckleFilter();
    ~SpeckleFilter();
    int Init();

    void process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int speckleRange, int maxDiff);
};

constexpr const size_t PERSISTENCY_LUT_SIZE = 256;

struct TemporalFilterParams {
    std::shared_ptr<dai::ImgFrame> currentFrame = {};
    uint8_t* accumulatorFrame = nullptr;

    uint8_t* history = nullptr;
    uint8_t* persistenceMap = nullptr;

    float alpha = 0.4f;  // The normalized weight of the current pixel
    uint8_t delta = 20;  // A threshold when a filter is invoked
    uint8_t currFrameIdx = 0;
};

class TemporalFilter {
   public:
    TemporalFilter();
    ~TemporalFilter();
    int Init(size_t frameSize, float alpha, int delta, int persistenceMode);

    void process(std::shared_ptr<dai::ImgFrame>& disparityFrame);

   private:
    struct MemSections {
        uint8_t* mem = nullptr;
        size_t size = 0;
    };

    int allocateBuffers(int frameSize);
    void buildPersistanceMap();
    uint8_t persistenceMode = 255;  // default value: "invalid"
    uint8_t currFrameIdx = 0;

    TemporalFilterParams params = {};

    MemSections rawAccumulatorFrame = {};  // Hold the last frame received for the current profile
    MemSections rawHistoryFrame = {};      // represents the history over the last 8 frames, 1 bit per frame
    // encodes whether a particular 8 bit history is good enough for all 8 phases of storage
    MemSections rawPersistenceMapLUT = {};  // Lookup table if persitency is enabled

    bool measureTime = false;
};

/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/
MedianFilter::MedianFilter() {}

int MedianFilter::Init() {
    return 0;
}

MedianFilter::~MedianFilter() {}

void MedianFilter::process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int medianSize) {
    int cvtype;
    if(disparityFrame->getType() == dai::ImgFrame::Type::RAW16) {
        cvtype = CV_16UC1;
    } else if(disparityFrame->getType() == dai::ImgFrame::Type::RAW8) {
        cvtype = CV_8UC1;
    } else {
        throw std::runtime_error("Unsupported disparity frame type. Supported types are RAW16 and RAW8.");
    }

    cv::Mat cvDisparity = cv::Mat(disparityFrame->getHeight(), disparityFrame->getWidth(), cvtype, disparityFrame->data->getData().data());

    if(medianSize % 2 == 0) {
        throw std::runtime_error("Median filter size must be odd");
    }
    if(medianSize > 5) {
        throw std::runtime_error("Median filter size must be <= 5");
    }

    cv::Mat cvDisparityFiltered = cv::Mat(disparityFrame->getHeight(), disparityFrame->getWidth(), cvtype);
    cv::medianBlur(cvDisparity, cvDisparityFiltered, medianSize);

    std::memcpy(disparityFrame->data->getData().data(), cvDisparityFiltered.data, cvDisparityFiltered.total() * cvDisparityFiltered.elemSize());
}

/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/

template <typename T>
void recursive_filter_horizontal(SpatialFilterParams* params) {
    void* image_data = (void*)params->currentFrame->data->getData().data();
    float alpha = params->alpha;
    float deltaZ = params->delta;
    int _width = params->currentFrame->getWidth();
    int _height = params->currentFrame->getHeight();
    size_t _holes_filling_radius = params->holesFillingRadius;

    // Handle conversions for invalid input data
    bool fp = (std::is_floating_point<T>::value);

    // Filtering integer values requires round-up to the nearest discrete value
    const float round = fp ? 0.f : 0.5f;
    // define invalid inputs
    const T valid_threshold = fp ? static_cast<T>(std::numeric_limits<T>::epsilon()) : static_cast<T>(1);
    const T delta_z = static_cast<T>(deltaZ);

    auto image = reinterpret_cast<T*>(image_data);
    size_t cur_fill = 0;

    for(int v = 0; v < _height; v++) {
        // left to right
        T* im = image + v * _width;
        T val0 = im[0];
        cur_fill = 0;

        for(int u = 1; u < _width - 1; u++) {
            T val1 = im[1];

            if(fabs(val0) >= valid_threshold) {
                if(fabs(val1) >= valid_threshold) {
                    cur_fill = 0;
                    T diff = static_cast<T>(fabs(val1 - val0));

                    if(diff >= valid_threshold && diff <= delta_z) {
                        float filtered = val1 * alpha + val0 * (1.0f - alpha);
                        val1 = static_cast<T>(filtered + round);
                        im[1] = val1;
                    }
                } else  // Only the old value is valid - appy holes filling
                {
                    if(_holes_filling_radius) {
                        if(++cur_fill < _holes_filling_radius) im[1] = val1 = val0;
                    }
                }
            }

            val0 = val1;
            im += 1;
        }

        // right to left
        im = image + (v + 1) * _width - 2;  // end of row - two pixels
        T val1 = im[1];
        cur_fill = 0;

        for(int u = _width - 1; u > 0; u--) {
            T val0 = im[0];

            if(val1 >= valid_threshold) {
                if(val0 > valid_threshold) {
                    cur_fill = 0;
                    T diff = static_cast<T>(fabs(val1 - val0));

                    if(diff <= delta_z) {
                        float filtered = val0 * alpha + val1 * (1.0f - alpha);
                        val0 = static_cast<T>(filtered + round);
                        im[0] = val0;
                    }
                } else  // 'inertial' hole filling
                {
                    if(_holes_filling_radius) {
                        if(++cur_fill < _holes_filling_radius) im[0] = val0 = val1;
                    }
                }
            }

            val1 = val0;
            im -= 1;
        }
    }
}

template <typename T>
void recursive_filter_vertical(SpatialFilterParams* params) {
    void* image_data = (void*)params->currentFrame->data->getData().data();
    float alpha = params->alpha;
    float deltaZ = params->delta;
    int _width = params->currentFrame->getWidth();
    int _height = params->currentFrame->getHeight();
    size_t _holes_filling_radius = params->holesFillingRadius;

    // Handle conversions for invalid input data
    bool fp = (std::is_floating_point<T>::value);

    // Filtering integer values requires round-up to the nearest discrete value
    const float round = fp ? 0.f : 0.5f;
    // define invalid range
    const T valid_threshold = fp ? static_cast<T>(std::numeric_limits<T>::epsilon()) : static_cast<T>(1);
    const T delta_z = static_cast<T>(deltaZ);

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
                if(diff < delta_z) {
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
                if(diff < delta_z) {
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

void SpatialFilter::process(std::shared_ptr<dai::ImgFrame>& disparityFrame) {
    params.currentFrame = disparityFrame;

    for(int i = 0; i < params.iterationNr; i++) {
        if(disparityFrame->getType() == dai::ImgFrame::Type::RAW16) {
            recursive_filter_horizontal<uint16_t>(&params);
            recursive_filter_vertical<uint16_t>(&params);
        } else if(disparityFrame->getType() == dai::ImgFrame::Type::RAW8) {
            recursive_filter_horizontal<uint8_t>(&params);
            recursive_filter_vertical<uint8_t>(&params);
        } else {
            throw std::runtime_error("Unsupported disparity frame type");
        }
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

void SpeckleFilter::process(std::shared_ptr<dai::ImgFrame>& disparityFrame, int speckleRange, int maxDiff) {
    int cvtype;
    if(disparityFrame->getType() == dai::ImgFrame::Type::RAW16) {
        cvtype = CV_16SC1;
    } else if(disparityFrame->getType() == dai::ImgFrame::Type::RAW8) {
        cvtype = CV_8UC1;
    } else {
        throw std::runtime_error("Unsupported disparity frame type. Supported types are RAW16 and RAW8.");
    }
    cv::Mat cvDisparity = cv::Mat(disparityFrame->getHeight(), disparityFrame->getWidth(), cvtype, disparityFrame->data->getData().data());
    cv::filterSpeckles(cvDisparity, 0, speckleRange, maxDiff);
}

/***********************************************************************************************************/
/***********************************************************************************************************/
/***********************************************************************************************************/

template <typename T>
void processImpl(TemporalFilterParams* params) {
    auto& currentFrame = params->currentFrame;
    auto& accumulatorFrame = params->accumulatorFrame;
    uint8_t* history = params->history;
    float alpha = params->alpha;
    uint8_t delta = params->delta;
    uint8_t currFrameIdx = params->currFrameIdx;
    uint8_t* persistenceMap = params->persistenceMap;
    static_assert((std::is_arithmetic<T>::value), "temporal filter assumes numeric types");

    const bool fp = (std::is_floating_point<T>::value);

    T delta_z = static_cast<T>(delta);

    auto frame = reinterpret_cast<T*>(currentFrame->data->getData().data());
    auto _last_frame = reinterpret_cast<T*>(accumulatorFrame);
    unsigned char mask = 1 << currFrameIdx;

    size_t frameSize = currentFrame->getWidth() * currentFrame->getHeight();

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

TemporalFilter::~TemporalFilter() {
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

void TemporalFilter::process(std::shared_ptr<dai::ImgFrame>& disparityFrame) {
    params.currentFrame = disparityFrame;
    params.currFrameIdx = currFrameIdx;
    params.accumulatorFrame = rawAccumulatorFrame.mem;

    processImpl<uint16_t>(&params);

    currFrameIdx = (currFrameIdx + 1) % 8;  // at end of cycle
}

int TemporalFilter::allocateBuffers(int frameSize) {
    if(rawAccumulatorFrame.mem) {
        if(rawAccumulatorFrame.size < static_cast<size_t>(frameSize)) {
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
        if(rawHistoryFrame.size < static_cast<size_t>(frameSize)) {
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

void TemporalFilter::buildPersistanceMap() {
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
    std::memcpy(persistenceMapLUT, credible_threshold.data(), PERSISTENCY_LUT_SIZE);
}

}  // namespace impl

class MedianFilterWrapper : public SequentialDepthFilters::Filter {
   public:
    MedianFilterWrapper(const MedianFilterParams& params) : params(params), medianFilter() {
        medianFilter.Init();
    }

    void process(std::shared_ptr<dai::ImgFrame>& frame) override {
        if(params.enable) {
            const int medianSize = static_cast<int>(params.median);
            medianFilter.process(frame, medianSize);
        }
    }

    void setParams(const FilterParams& params) override {
        if(std::holds_alternative<MedianFilterParams>(params)) {
            this->params = std::get<MedianFilterParams>(params);
        } else {
            throw std::runtime_error("Invalid filter params");
        }
    }

   private:
    MedianFilterParams params;
    impl::MedianFilter medianFilter;
};

class SpatialFilterWrapper : public SequentialDepthFilters::Filter {
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
            throw std::runtime_error("Invalid filter params");
        }
    }

   private:
    SpatialFilterParams params;
    impl::SpatialFilter spatialFilter;
};

class SpeckleFilterWrapper : public SequentialDepthFilters::Filter {
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
            throw std::runtime_error("Invalid filter params");
        }
    }

   private:
    SpeckleFilterParams params;
    impl::SpeckleFilter speckleFilter;
};

class TemporalFilterWrapper : public SequentialDepthFilters::Filter {
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
                const size_t frameSize = frame->getHeight() * frame->getWidth() * sizeof(uint16_t);
                const float alpha = params.alpha;
                const int delta = params.delta;
                const int persistencyMode = static_cast<int>(params.persistencyMode);
                temporalFilter.Init(frameSize, alpha, delta, persistencyMode);
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
            throw std::runtime_error("Invalid filter params");
        }
    }
};

}  // namespace

std::unique_ptr<SequentialDepthFilters::Filter> createFilter(const MedianFilterParams& params) {
    return std::make_unique<MedianFilterWrapper>(params);
}

std::unique_ptr<SequentialDepthFilters::Filter> createFilter(const SpatialFilterParams& params) {
    return std::make_unique<SpatialFilterWrapper>(params);
}

std::unique_ptr<SequentialDepthFilters::Filter> createFilter(const SpeckleFilterParams& params) {
    return std::make_unique<SpeckleFilterWrapper>(params);
}

std::unique_ptr<SequentialDepthFilters::Filter> createFilter(const TemporalFilterParams& params) {
    return std::make_unique<TemporalFilterWrapper>(params);
}

std::unique_ptr<SequentialDepthFilters::Filter> createFilter(const FilterParams& params) {
    return std::visit([](auto&& arg) -> std::unique_ptr<SequentialDepthFilters::Filter> { return createFilter(arg); }, params);
}

void SequentialDepthFilters::addFilter(const FilterParams& filter) {
    properties.filters.push_back(filter);
}

void SequentialDepthFilters::run() {
    // Create filters
    logger->debug("SequentialDepthFilters: Creating filters");
    std::vector<std::unique_ptr<Filter>> filters;
    for(const auto& filter : properties.filters) {
        filters.push_back(createFilter(filter));
    }

    logger->debug("SequentialDepthFilters: Starting");
    while(isRunning()) {
        // Set config
        while(config.has()) {
            auto configMsg = config.get<SequentialDepthFiltersConfig>();
            auto index = configMsg->filterIndex;
            if(index >= static_cast<int>(filters.size())) {
                logger->error("SequentialDepthFilters: Invalid filter index: {}", index);
                break;
            }
            filters[index]->setParams(configMsg->filterParams);
        }

        // Get frame from input queue
        std::shared_ptr<dai::ImgFrame> frame = input.get<dai::ImgFrame>();
        if(frame == nullptr) {
            logger->error("SequentialDepthFilters: Input frame is nullptr");
            break;
        }

        // Create a clone and run filters inplace on the clone
        std::shared_ptr<dai::ImgFrame> filteredFrame = frame->clone();
        for(const auto& filter : filters) {
            filter->process(filteredFrame);  // inplace filter
        }

        // Send filtered frame to the output queue
        output.send(filteredFrame);
    }
}

bool SequentialDepthFilters::runOnHost() const {
    return runOnHostVar;
}

void SequentialDepthFilters::setRunOnHost(bool runOnHost) {
    if(device && device->getPlatform() == Platform::RVC2 && !runOnHost) {
        throw std::runtime_error("SequentialDepthFilters: Running on device is not supported on RVC2");
    }
    runOnHostVar = runOnHost;
}

void DepthConfidenceFilter::applyDepthConfidenceFilter(std::shared_ptr<ImgFrame> depthFrame,
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
        if(type == ImgFrame::Type::RAW32) {
            return CV_32FC1;
        }
        throw std::runtime_error("DepthConfidenceFilter: Unsupported frame type");
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
                conf = 0.0f;
            } else {
                // Corrected formula: amplitude / sqrt(depth / 2.0)
                // Higher amplitude --> higher confidence
                // Higher depth --> slightly lower confidence
                conf = a / std::sqrt(d / 2.0f);
            }

            confidence.at<std::uint16_t>(i, j) = static_cast<std::uint16_t>(conf);

            // Invalidate pixel if confidence is below threshold
            if(conf < threshold) {
                filteredDepth.at<std::uint16_t>(i, j) = std::numeric_limits<std::uint16_t>::max();
            } else {
                filteredDepth.at<std::uint16_t>(i, j) = static_cast<std::uint16_t>(d);
            }
        }
    }
}

void DepthConfidenceFilter::run() {
    while(isRunning()) {
        // Update threshold dynamically
        while(config.has()) {
            auto configMsg = config.get<DepthConfidenceFilterConfig>();
            properties.confidenceThreshold = configMsg->confidenceThreshold;
        }

        // Get frames from input queue
        std::shared_ptr<dai::ImgFrame> depthFrame = depth.get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> amplitudeFrame = amplitude.get<dai::ImgFrame>();
        if(depthFrame == nullptr || amplitudeFrame == nullptr) {
            logger->error("DepthConfidenceFilter: Input frame is nullptr");
            break;
        }

        // Create empty output frames
        auto filteredDepthFrame = std::make_shared<dai::ImgFrame>();
        auto confidenceFrame = std::make_shared<dai::ImgFrame>();

        // Copy metadata from input frames
        filteredDepthFrame->setMetadata(depthFrame);
        confidenceFrame->setMetadata(depthFrame);

        // Allocate memory for output frames
        filteredDepthFrame->data->setSize(depthFrame->getData().size() * sizeof(std::uint16_t));
        confidenceFrame->data->setSize(depthFrame->getData().size() * sizeof(std::uint16_t));

        filteredDepthFrame->setType(ImgFrame::Type::RAW16);
        confidenceFrame->setType(ImgFrame::Type::RAW16);

        // Apply filter
        applyDepthConfidenceFilter(depthFrame, amplitudeFrame, filteredDepthFrame, confidenceFrame, properties.confidenceThreshold);

        // Send results
        filtered_depth.send(filteredDepthFrame);
        confidence.send(confidenceFrame);
    }
}

float DepthConfidenceFilter::getConfidenceThreshold() const {
    return properties.confidenceThreshold;
}

void DepthConfidenceFilter::setConfidenceThreshold(float threshold) {
    properties.confidenceThreshold = threshold;
}

void DepthConfidenceFilter::setRunOnHost(bool runOnHost) {
    if(device && device->getPlatform() == Platform::RVC2 && !runOnHost) {
        throw std::runtime_error("DepthConfidenceFilter: Running on device is not supported on RVC2");
    }
    runOnHostVar = runOnHost;
}

bool DepthConfidenceFilter::runOnHost() const {
    return runOnHostVar;
}

}  // namespace node
}  // namespace dai