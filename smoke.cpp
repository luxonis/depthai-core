#include <cstdint>
#include <cstdio>
#include <vector>
extern "C" {
#include <apriltag.h>
#include <tag36h11.h>
#include <common/image_u8.h>
}

int main() {
    auto* family = tag36h11_create();
    image_u8_t* tag = apriltag_to_image(family, 0);
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
    image_u8_t input{width, height, width, pixels.data()};
    auto* detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, family);
    for(int threads : {1, 2}) {
        detector->nthreads = threads;
        for(int iteration = 0; iteration < 3; ++iteration) {
            auto* detections = apriltag_detector_detect(detector, &input);
            if(zarray_size(detections) != 1) {
                std::printf("FAIL: threads=%d detections=%d\n", threads, zarray_size(detections));
                return 1;
            }
            apriltag_detection_t* detection;
            zarray_get(detections, 0, &detection);
            if(detection->id != 0 || detection->hamming != 0) return 2;
            std::printf("PASS: threads=%d iteration=%d tag_id=%d hamming=%d\n", threads, iteration, detection->id, detection->hamming);
            apriltag_detections_destroy(detections);
        }
    }
    apriltag_detector_destroy(detector);
    image_u8_destroy(tag);
    tag36h11_destroy(family);
    return 0;
}
