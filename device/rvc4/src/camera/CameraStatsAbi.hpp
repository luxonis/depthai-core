#pragma once

#include <cstddef>
#include <cstdint>

#include "luxonis_sidechannel_data.h"

// Camera HAL statistics v1 is an immutable, optional C ABI. Older build sysroots lack its declarations.
#ifndef LUX_SC_CAMERA_STATS_ABI_VERSION
    #define LUX_SC_CAMERA_STATS_ABI_VERSION 1
    #define LUX_SC_CAMERA_STATS_CAUSE_COUNT 15
struct lux_sc_camera_stats_v1_t {
    std::uint32_t abi_version = LUX_SC_CAMERA_STATS_ABI_VERSION;
    std::uint32_t counter_bits = 64;
    std::uint32_t active = 0;
    std::uint64_t generation = 0;
    std::uint64_t timestamp_ns = 0;
    std::uint64_t error_events = 0;
    std::uint64_t error_causes[LUX_SC_CAMERA_STATS_CAUSE_COUNT] = {};
};
using lux_sc_camera_stats_get_v1_cb_t = int (*)(std::int32_t, lux_sc_camera_stats_v1_t*);
#endif

static_assert(sizeof(lux_sc_camera_stats_v1_t) == 160, "Camera HAL statistics v1 ABI size changed");
static_assert(offsetof(lux_sc_camera_stats_v1_t, error_causes) == 40, "Camera HAL statistics v1 ABI layout changed");
