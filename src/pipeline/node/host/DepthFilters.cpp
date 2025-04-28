#include "depthai/pipeline/node/host/DepthFilters.hpp"

#include <cmath>
#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"

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
    } else if (disparityFrame->getType() == dai::ImgFrame::Type::RAW8) {
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
        } else if (disparityFrame->getType() == dai::ImgFrame::Type::RAW8) {
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
    } else if (disparityFrame->getType() == dai::ImgFrame::Type::RAW8) {
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
        const int medianSize = static_cast<int>(params);
        medianFilter.process(frame, medianSize);
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
        spatialFilter.process(frame);
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
        const int speckleRange = params.speckleRange;
        const int maxDiff = params.differenceThreshold;
        speckleFilter.process(frame, speckleRange, maxDiff);
    }

   private:
    SpeckleFilterParams params;
    impl::SpeckleFilter speckleFilter;
};

class TemporalFilterWrapper : public SequentialDepthFilters::Filter {
   public:
    TemporalFilterWrapper(const TemporalFilterParams& params) : params(params), temporalFilter() {}

    void process(std::shared_ptr<dai::ImgFrame>& frame) override {
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

   private:
    bool isInitialized = false;
    TemporalFilterParams params;
    impl::TemporalFilter temporalFilter;
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

void DepthConfidenceFilter::apply_depth_confidence_filter(std::shared_ptr<ImgFrame> depthFrame,
                                                          std::shared_ptr<ImgFrame> amplitudeFrame,
                                                          std::shared_ptr<ImgFrame> filteredDepthFrame,
                                                          std::shared_ptr<ImgFrame> confidenceFrame,
                                                          float threshold) {
    const int height = depthFrame->getHeight();
    const int width = depthFrame->getWidth();

    // Create an OpenCV wrapper for frames for easier processing
    cv::Mat depth(height, width, CV_16UC1, depthFrame->getData().data());
    cv::Mat amplitude(height, width, CV_16UC1, amplitudeFrame->getData().data());
    cv::Mat filtered_depth(height, width, CV_16UC1, filteredDepthFrame->getData().data());
    cv::Mat confidence(height, width, CV_32FC1, confidenceFrame->getData().data());

    cv::Mat depth_float, amplitude_float;
    depth.convertTo(depth_float, CV_32F);
    amplitude.convertTo(amplitude_float, CV_32F);

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {
            float a = amplitude_float.at<float>(i, j);
            float d = depth_float.at<float>(i, j);

            // Avoid division by zero or very small depth values
            float conf;
            if(a < 1e-12f || d < 1e-12f) {
                conf = 0.0f;
            } else {
                // Corrected formula: amplitude / sqrt(depth / 2.0)
                // Higher amplitude → higher confidence
                // Higher depth → slightly lower confidence
                conf = a / std::sqrt(d / 2.0f);
            }

            confidence.at<float>(i, j) = conf;

            // Invalidate pixel if confidence is below the threshold
            if(conf < threshold) {
                filtered_depth.at<uint16_t>(i, j) = 0;  // Using 0 to represent invalid/NaN values
            } else {
                filtered_depth.at<uint16_t>(i, j) = depth.at<uint16_t>(i, j);
            }
        }
    }
}

void DepthConfidenceFilter::run() {
    while(isRunning()) {
        std::shared_ptr<dai::ImgFrame> depthFrame = depth.get<dai::ImgFrame>();
        std::shared_ptr<dai::ImgFrame> amplitudeFrame = amplitude.get<dai::ImgFrame>();

        if(depthFrame == nullptr || amplitudeFrame == nullptr) {
            logger->error("DepthConfidenceFilter: Input frame is nullptr");
            break;
        }

        // Create empty output frames
        std::shared_ptr<dai::ImgFrame> filteredDepthFrame = depthFrame->clone();
        std::shared_ptr<dai::ImgFrame> confidenceFrame = depthFrame->clone();

        // Initialize output frames
        filteredDepthFrame->setWidth(depthFrame->getWidth());
        filteredDepthFrame->setHeight(depthFrame->getHeight());
        filteredDepthFrame->setType(depthFrame->getType());
        filteredDepthFrame->data->setSize(depthFrame->getData().size());

        confidenceFrame->setWidth(depthFrame->getWidth());
        confidenceFrame->setHeight(depthFrame->getHeight());
        confidenceFrame->setType(ImgFrame::Type::GRAYF16);  // Using float16 for confidence values
        confidenceFrame->data->setSize(depthFrame->getWidth() * depthFrame->getHeight() * sizeof(float));

        // Apply the filter
        apply_depth_confidence_filter(depthFrame, amplitudeFrame, filteredDepthFrame, confidenceFrame, properties.confidenceThreshold);

        // Send the results
        filtered_depth.send(filteredDepthFrame);
        confidence.send(confidenceFrame);
    }
}

}  // namespace node
}  // namespace dai