#pragma once

#include <cstdint>
#include <string>
#include <utility>

namespace dai {

/// VD55H1 ToF sensor capture mode (RVC4 only).
enum class ToFSensorMode : uint32_t {
    F1_FULL,
    F2_FULL,
    F3_FULL,
    F2_BINNING_2X2,
    F3_BINNING_2X2,
};

constexpr uint32_t TOF_VD55H1_STATUS_LINE_NB = 8;
constexpr uint32_t TOF_VD55H1_SUBFRAME_PER_CONTEXT = 3;

inline uint32_t getToFSensorModeContextFrames(ToFSensorMode mode) {
    switch(mode) {
        case ToFSensorMode::F1_FULL:
            return 1;
        case ToFSensorMode::F2_FULL:
            return 2;
        case ToFSensorMode::F3_FULL:
            return 3;
        case ToFSensorMode::F2_BINNING_2X2:
            return 2;
        case ToFSensorMode::F3_BINNING_2X2:
            return 3;
    }
    return 3;
}

/// Raw superframe size captured by Camera.raw (input to IPP).
inline std::pair<uint32_t, uint32_t> getToFSensorModeRawResolution(ToFSensorMode mode) {
    switch(mode) {
        case ToFSensorMode::F1_FULL:
            return {1344, 2420};
        case ToFSensorMode::F2_FULL:
            return {1344, 4832};
        case ToFSensorMode::F3_FULL:
            return {1344, 7244};
        case ToFSensorMode::F2_BINNING_2X2:
            return {672, 2420};
        case ToFSensorMode::F3_BINNING_2X2:
            return {672, 3626};
    }
    return {1344, 7244};
}

/// Depth / amplitude / confidence output size after IPP and landscape transpose.
inline std::pair<uint32_t, uint32_t> getToFSensorModeOutputResolution(ToFSensorMode mode) {
    const auto [rawW, rawH] = getToFSensorModeRawResolution(mode);
    const uint32_t contextFrames = getToFSensorModeContextFrames(mode);
    const uint32_t ippDepthW = rawW / 2;
    const uint32_t ippDepthH = (rawH - TOF_VD55H1_STATUS_LINE_NB) / (contextFrames * TOF_VD55H1_SUBFRAME_PER_CONTEXT);
    return {ippDepthH, ippDepthW};
}

inline std::string toString(ToFSensorMode mode) {
    switch(mode) {
        case ToFSensorMode::F1_FULL:
            return "F1_FULL";
        case ToFSensorMode::F2_FULL:
            return "F2_FULL";
        case ToFSensorMode::F3_FULL:
            return "F3_FULL";
        case ToFSensorMode::F2_BINNING_2X2:
            return "F2_BINNING_2X2";
        case ToFSensorMode::F3_BINNING_2X2:
            return "F3_BINNING_2X2";
    }
    return "UNKNOWN";
}

}  // namespace dai

inline std::ostream& operator<<(std::ostream& out, dai::ToFSensorMode mode) {
    return out << dai::toString(mode);
}
