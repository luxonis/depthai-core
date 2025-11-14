#pragma once

#include <cstdint>

enum class YoloDecodingFamily : std::int32_t {
    TLBR,  // top left bottom right anchor free: yolo v6r2, v8 v10 v11
    R1AF,  // anchor free: yolo v6r1
    v5AB,  // anchor based yolo v5, v7, P
    v3AB,  // anchor based yolo v3 v3-Tiny
};
