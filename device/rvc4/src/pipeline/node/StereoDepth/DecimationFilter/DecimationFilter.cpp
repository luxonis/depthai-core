#include "DecimationFilter.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dai {
#define PIX_SORT(a, b)                    \
    {                                     \
        if((a) > (b)) PIX_SWAP((a), (b)); \
    }
#define PIX_SWAP(a, b)         \
    {                          \
        pixelvalue temp = (a); \
        (a) = (b);             \
        (b) = temp;            \
    }
#define PIX_MIN(a, b) ((a) > (b)) ? (b) : (a)
#define PIX_MAX(a, b) ((a) > (b)) ? (a) : (b)

template <class pixelvalue>  // From sci.image.processing
inline pixelvalue opt_med3(pixelvalue* p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[0], p[1]);
    return p[1];
}

template <class pixelvalue>
inline pixelvalue opt_med4(pixelvalue* p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[2], p[3]);
    PIX_SORT(p[0], p[2]);
    PIX_SORT(p[1], p[3]);
    return PIX_MIN(p[1], p[2]);
}

template <class pixelvalue>  // From sci.image.processing
inline pixelvalue opt_med5(pixelvalue* p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[4]);
    p[3] = PIX_MAX(p[0], p[3]);
    p[1] = PIX_MIN(p[1], p[4]);
    PIX_SORT(p[1], p[2]);
    p[2] = PIX_MIN(p[2], p[3]);
    return PIX_MAX(p[1], p[2]);
}

template <class pixelvalue>  // Selection network from Havlicek, Sakady, Katz: Fast, efficient median filters with even length windows
inline pixelvalue opt_med6(pixelvalue* p) {
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[2], p[3]);
    PIX_SORT(p[4], p[5]);
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[2], p[3]);
    p[4] = PIX_MIN(p[4], p[5]);
    p[2] = PIX_MAX(p[1], p[2]);
    p[3] = PIX_MIN(p[3], p[4]);
    return PIX_MIN(p[2], p[3]);
}

template <class pixelvalue>  // From sci.image.processing
inline pixelvalue opt_med7(pixelvalue* p) {
    PIX_SORT(p[0], p[5]);
    PIX_SORT(p[0], p[3]);
    PIX_SORT(p[1], p[6]);
    PIX_SORT(p[2], p[4]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[5]);
    PIX_SORT(p[2], p[6]);
    p[3] = PIX_MAX(p[2], p[3]);
    p[3] = PIX_MIN(p[3], p[6]);
    p[4] = PIX_MIN(p[4], p[5]);
    PIX_SORT(p[1], p[4]);
    p[3] = PIX_MAX(p[1], p[3]);
    return PIX_MIN(p[3], p[4]);
}

template <class pixelvalue>
inline pixelvalue opt_med8(pixelvalue* p) {
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[6], p[7]);
    PIX_SORT(p[2], p[3]);
    PIX_SORT(p[5], p[6]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[6], p[7]);
    p[4] = PIX_MIN(p[4], p[7]);
    PIX_SORT(p[3], p[6]);
    p[5] = PIX_MAX(p[2], p[5]);
    p[3] = PIX_MAX(p[0], p[3]);
    p[1] = PIX_MIN(p[1], p[4]);
    p[3] = PIX_MIN(p[3], p[6]);
    PIX_SORT(p[3], p[1]);
    p[3] = PIX_MAX(p[5], p[3]);
    return PIX_MIN(p[3], p[1]);
}

template <class pixelvalue>  // Formula from XILINX XCELL vol. 23 (John L. Smith); modifies the input array
inline pixelvalue opt_med9(pixelvalue* p) {
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[4], p[5]);
    PIX_SORT(p[7], p[8]);
    PIX_SORT(p[0], p[1]);
    PIX_SORT(p[3], p[4]);
    PIX_SORT(p[6], p[7]);
    PIX_SORT(p[1], p[2]);
    PIX_SORT(p[4], p[5]);
    PIX_SORT(p[7], p[8]);
    p[3] = PIX_MAX(p[0], p[3]);
    p[5] = PIX_MIN(p[5], p[8]);
    PIX_SORT(p[4], p[7]);
    p[6] = PIX_MAX(p[3], p[6]);
    p[4] = PIX_MAX(p[1], p[4]);
    p[2] = PIX_MIN(p[2], p[5]);
    p[4] = PIX_MIN(p[4], p[7]);
    PIX_SORT(p[4], p[2]);
    p[4] = PIX_MAX(p[6], p[4]);
    return PIX_MIN(p[4], p[2]);
}

template <typename T>
void decimate(DecimationFilterParams* params) {
    int _width = params->currentFrame->getWidth();
    int _height = params->currentFrame->getHeight();

    T* image = reinterpret_cast<T*>(params->currentFrame->data->getData().data());
    T* imageOut = reinterpret_cast<T*>(params->destFrame->data->getData().data());

    const int scale = params->decimationFactor;

    T* pixel_raws[scale];

    int _filter_width = _width / scale;
    int _filter_height = _height / scale;
    int _dest_width = params->destFrame->getWidth();
    int _dest_height = params->destFrame->getHeight();

    if(params->decimationMode == PIXEL_SKIPPING) {
        for(int j = 0; j < _dest_height; j++) {
            T* outLine = imageOut + j * _dest_width;
            T* inLine = image + j * scale * _width;

            for(int i = 0; i < _filter_width; i++) {
                outLine[i] = inLine[i * scale];
            }

            for(int i = _filter_width; i < _dest_width; i++) {
                outLine[i] = 0;
            }
        }

        return;
    }

    bool median = params->decimationMode == NON_ZERO_MEDIAN && (scale == 2 || scale == 3);

    const int _kernel_size = scale * scale;
    T working_kernel[_kernel_size];
    T* wk_begin = working_kernel;
    T* wk_itr = wk_begin;

    for(int j = 0; j < _filter_height; j++) {
        T* outLine = imageOut + j * _dest_width;

        T* p{};
        for(int i = 0; i < scale; i++) {
            pixel_raws[i] = image + (j * scale + i) * _width;
        }

        if(median) {
            for(int i = 0; i < _filter_width; i++) {
                wk_itr = wk_begin;
                for(int n = 0; n < scale; ++n) {
                    p = pixel_raws[n] + i * scale;
                    ;
                    for(int m = 0; m < scale; ++m) {
                        if(*(p + m)) *wk_itr++ = *(p + m);
                    }
                }

                auto ks = (int)(wk_itr - wk_begin);  // Even-size kernels take the member one below the middle
                if(ks == 0)
                    outLine[i] = 0;
                else {
                    switch(ks) {
                        case 1:
                            outLine[i] = working_kernel[0];
                            break;
                        case 2:
                            outLine[i] = PIX_MIN(working_kernel[0], working_kernel[1]);
                            break;
                        case 3:
                            outLine[i] = opt_med3<T>(working_kernel);
                            break;
                        case 4:
                            outLine[i] = opt_med4<T>(working_kernel);
                            break;
                        case 5:
                            outLine[i] = opt_med5<T>(working_kernel);
                            break;
                        case 6:
                            outLine[i] = opt_med6<T>(working_kernel);
                            break;
                        case 7:
                            outLine[i] = opt_med7<T>(working_kernel);
                            break;
                        case 8:
                            outLine[i] = opt_med8<T>(working_kernel);
                            break;
                        case 9:
                            outLine[i] = opt_med9<T>(working_kernel);
                            break;
                    }
                }
            }
        } else {
#pragma clang loop unroll_count(8)
            for(int i = 0; i < _filter_width; i++) {
                int sum = 0;
                int counter = 0;

                for(int n = 0; n < scale; ++n) {
                    p = pixel_raws[n] + i * scale;
                    for(int m = 0; m < scale; ++m) {
                        if(*(p + m)) {
                            sum += p[m];
                            ++counter;
                        }
                    }
                }

                outLine[i] = (counter == 0 ? 0 : sum / counter);
            }
        }

        for(int i = _filter_width; i < _dest_width; i++) outLine[i] = 0;
    }

    for(auto v = _filter_height; v < _dest_height; ++v) {
        T* outLine = imageOut + v * _dest_width;
        for(auto u = 0; u < _dest_width; ++u) outLine[u] = 0;
    }
}

DecimationFilter::DecimationFilter() {}

int DecimationFilter::Init(int decimationFactor, DecimationMode mode) {
    params.decimationFactor = decimationFactor;
    params.decimationMode = mode;

    return 0;
}

DecimationFilter::~DecimationFilter() {}

void DecimationFilter::process(std::shared_ptr<dai::ImgFrame>& fb, std::shared_ptr<dai::ImgFrame>& destFb) {
    params.currentFrame = fb;
    params.destFrame = destFb;

    decimate<uint16_t>(&params);
}

}  // namespace dai
