#include "SpatialFilter.hpp"

#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dai {

template <typename T>
void recursive_filter_horizontal(SpatialFilterParams* params) {
    void* image_data = (void*)params->currentFrame->data->getData().data();
    float alpha = params->alpha;
    float deltaZ = params->delta;
    int _width = params->currentFrame->getWidth();
    int _height = params->currentFrame->getHeight();
    size_t _holes_filling_radius = params->holesFillingRadius;

    bool fp = (std::is_floating_point<T>::value);

    const float round = fp ? 0.f : 0.5f;
    const T valid_threshold = fp ? static_cast<T>(std::numeric_limits<T>::epsilon()) : static_cast<T>(1);
    const T delta_z = static_cast<T>(deltaZ);

    auto image = reinterpret_cast<T*>(image_data);
    size_t cur_fill = 0;

    for(int v = 0; v < _height; v++) {
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

    bool fp = (std::is_floating_point<T>::value);

    const float round = fp ? 0.f : 0.5f;
    const T valid_threshold = fp ? static_cast<T>(std::numeric_limits<T>::epsilon()) : static_cast<T>(1);
    const T delta_z = static_cast<T>(deltaZ);

    auto image = reinterpret_cast<T*>(image_data);

    T* im = image;
    T im0{};
    T imw{};
    for(int v = 1; v < _height; v++) {
        for(int u = 0; u < _width; u++) {
            im0 = im[0];
            imw = im[_width];

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
        recursive_filter_horizontal<uint16_t>(&params);
        recursive_filter_vertical<uint16_t>(&params);
    }
}

}  // namespace dai
