#include "depthai/utility/Serialization.hpp"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include "HAP_farf.h"
#include "StereoDepthDsp.h"
#include "StereoDepthDspConfig.h"
#include "hexagon_types.h"
#include "hexagon_protos.h"
#include "HAP_vtcm_mgr.h"

int StereoDepthDsp_open(const char* uri, remote_handle64* handle) {
    *handle = 0x00DEAD00;

    FARF(ALWAYS, "SpatialLocationCalculator open session\n");

    return 0;
}

int StereoDepthDsp_close(remote_handle64 handle) {
    FARF(ALWAYS, "SpatialLocationCalculator close session\n");

    return 0;
}

int StereoDepthDsp_disparityToDepth(remote_handle64 h,
                                      const unsigned char* inDisparityFrame,
                                      int inDisparityFrameLen,
                                      const unsigned char* inConfig,
                                      int inConfigLen,
                                      unsigned char* outDepthFrame,
                                      int outDepthFrameLen) {

    if(inConfigLen != sizeof(StereoDepthDspConfig)) {
        return __LINE__;
    }

    StereoDepthDspConfig* cfgPtr = (StereoDepthDspConfig*)aligned_alloc(128, sizeof(StereoDepthDspConfig));
    if(cfgPtr == nullptr) {
        return __LINE__;
    }
    *cfgPtr = *(StereoDepthDspConfig*)inConfig;

    StereoDepthDspConfig& config = *cfgPtr;

    int width = config.width;
    int height = config.height;
    int fixedNumerator = config.fixedNumerator;

    void* scratchPad = HAP_request_VTCM(sizeof(HVX_Vector) + sizeof(config.disp2depthLUT), 1);
    if(scratchPad == nullptr) {
        return __LINE__;
    }

    HVX_Vector* VTCM_gather = (HVX_Vector*)scratchPad;

    uint16_t* dispToDepthLut = (uint16_t*)(VTCM_gather + 1);

    memcpy(dispToDepthLut, config.disp2depthLUT, sizeof(config.disp2depthLUT));

    for(int row = 0; row < height; ++row) {
        const uint16_t* __restrict__ disparity = reinterpret_cast<const uint16_t*>(inDisparityFrame) + row * width;
        uint16_t* __restrict__ depth = reinterpret_cast<uint16_t*>(outDepthFrame) + row * width;

        if (row < height - 1) {
            Q6_l2fetch_AR((void*)(disparity + width), width*2 << 16 | width*2 << 8 | 1 << 0);
        }

        #pragma clang loop interleave(enable) unroll(enable)
        for(int col = 0; col < width; col += sizeof(HVX_Vector) / sizeof(uint16_t)) {

            HVX_Vector vuh_disparity = *(HVX_Vector*)&disparity[col];
            HVX_Vector vuh_disparity_double = vuh_disparity * sizeof(uint16_t);

            Q6_vgather_ARMVh(
                VTCM_gather,
                reinterpret_cast<uint32_t>(dispToDepthLut),
                sizeof(config.disp2depthLUT) - 1,
                vuh_disparity_double
            );

            *(HVX_Vector*)&depth[col] = *VTCM_gather;
        }
    }

    HAP_release_VTCM(scratchPad);

    free(cfgPtr);

    return 0;

}


int StereoDepthDsp_leftRightCheck(remote_handle64 h,
                                      const unsigned char* inDisparityLR,
                                      int inDisparityLRLen,
                                      const unsigned char* inDisparityRL,
                                      int inDisparityRLLen,
                                      const unsigned char* inConfig,
                                      int inConfigLen,
                                      unsigned char* outDisparityFrame,
                                      int outDisparityFrameLen) {

    if(inConfigLen != sizeof(StereoDepthDspConfig)) {
        return __LINE__;
    }

    StereoDepthDspConfig config = *(StereoDepthDspConfig*)inConfig;

    int width = config.width;
    int height = config.height;
    int leftRightCheckThreshold = config.leftRightCheckThreshold;
    bool primarySearchR2L = config.primarySearchR2L != 0;

    for(int row = 0; row < height; ++row) {
        const uint16_t* __restrict__ disp_LR = reinterpret_cast<const uint16_t*>(inDisparityLR) + row * width;
        const uint16_t* __restrict__ disp_RL = reinterpret_cast<const uint16_t*>(inDisparityRL) + row * width;
        uint16_t* out = reinterpret_cast<uint16_t*>(outDisparityFrame) + row * width;

        for(int col = 0; col < width; ++col) {
            int16_t dispPrimaryVal = disp_LR[col];
            int16_t dispPrimaryValInt = dispPrimaryVal / 16;
            int32_t oppositeOffset = primarySearchR2L ? col + dispPrimaryValInt : col - dispPrimaryValInt;
            bool valid = oppositeOffset > 0 && oppositeOffset < width;

            int16_t dispOppositeVal = 0;
            if(valid) {
                dispOppositeVal = disp_RL[oppositeOffset];
            }

            int16_t absDiff = abs(dispPrimaryVal - dispOppositeVal);
            if(leftRightCheckThreshold * 16 > absDiff) {
                out[col] = (dispPrimaryVal + dispOppositeVal) / 2;
            } else {
                out[col] = 0;
            }
        }
    }

    return 0;
}

int StereoDepthDsp_confidenceInvalidation(remote_handle64 h,
                                      const unsigned char* inDisparityFrame,
                                      int inDisparityFrameLen,
                                      const unsigned char* inConfidenceFrame,
                                      int inConfidenceFrameLen,
                                      const unsigned char* inConfig,
                                      int inConfigLen,
                                      unsigned char* outDisparityFrame,
                                      int outDisparityFrameLen) {

    if(inConfigLen != sizeof(StereoDepthDspConfig)) {
        return __LINE__;
    }

    StereoDepthDspConfig config = *(StereoDepthDspConfig*)inConfig;

    int width = config.width;
    int height = config.height;
    uint8_t confidenceThreshold = config.confidenceThreshold;

    for(int row = 0; row < height; ++row) {

        const uint16_t* __restrict__ inDisp = reinterpret_cast<const uint16_t*>(inDisparityFrame) + row * width;
        const uint8_t* __restrict__ inConf = reinterpret_cast<const uint8_t*>(inConfidenceFrame) + row * width;
        uint16_t* __restrict__ outDisp = reinterpret_cast<uint16_t*>(outDisparityFrame) + row * width;

        if(row < height - 1) {
            Q6_l2fetch_AR((void*)(inDisp + width), width*2 << 16 | width*2 << 8 | 1 << 0);
            Q6_l2fetch_AR((void*)(inConf + width), width << 16 | width << 8 | 1 << 0);
        }
        // TODO HANDLE ALIGNMENT TODO
        #pragma clang loop interleave(enable)
        for(int col = 0; col < width; col += sizeof(HVX_VectorPair)/2) {
            HVX_VectorPair inDispVec = *(HVX_VectorPair*)&inDisp[col];
            HVX_Vector inConfVec = *(HVX_Vector*)&inConf[col];
            HVX_VectorPair inConfVec_u16 = Q6_Wuh_vunpack_Vub(inConfVec);  // Unpack to u16
            HVX_Vector vConfidenceThreshold = Q6_Vh_vsplat_R(confidenceThreshold);

            HVX_VectorPred pred1 = Q6_Q_vcmp_gt_VuhVuh(vConfidenceThreshold, Q6_V_lo_W(inConfVec_u16));
            HVX_Vector outDispVec1 = Q6_V_vmux_QVV(pred1, Q6_V_vzero(), Q6_V_lo_W(inDispVec));

            HVX_VectorPred pred2 = Q6_Q_vcmp_gt_VuhVuh(vConfidenceThreshold, Q6_V_hi_W(inConfVec_u16));
            HVX_Vector outDispVec2 = Q6_V_vmux_QVV(pred2, Q6_V_vzero(), Q6_V_hi_W(inDispVec));

            *(HVX_Vector*)&outDisp[col] = outDispVec1;
            *(HVX_Vector*)&outDisp[col+sizeof(HVX_Vector)/2] = outDispVec2;
        }
    }

    return 0;

}

int StereoDepthDsp_combineDisparityMaps(remote_handle64 h,
                                      const unsigned char* inDisparityFrame,
                                      int inDisparityFrameLen,
                                      const unsigned char* inDisparityDsFrame,
                                      int inDisparityDsFrameLen,
                                      const unsigned char* inConfig,
                                      int inConfigLen,
                                      unsigned char* outDisparityFrame,
                                      int outDisparityFrameLen) {

    if(inConfigLen != sizeof(StereoDepthDspConfig)) {
        return __LINE__;
    }

    StereoDepthDspConfig config = *(StereoDepthDspConfig*)inConfig;

    int width = config.width;
    int height = config.height;
    int numDisparities = config.numDisparities;

    const uint16_t* __restrict__ reference = reinterpret_cast<const uint16_t*>(inDisparityFrame);
    const uint16_t* __restrict__ downScaled = reinterpret_cast<const uint16_t*>(inDisparityDsFrame);
    uint16_t* __restrict__ output = reinterpret_cast<uint16_t*>(outDisparityFrame);

    uint16_t downscaleFactor = config.downscaleFactor;
    numDisparities -= 1;
    uint16_t half_disp = numDisparities * 16;

    alignas(128) uint16_t downScaledExpanded[width];

    for(int row = 0; row < height; row++) {
        if(row < height - 1) {
            Q6_l2fetch_AR((void*)(reference + width), width*2 << 16 | width*2 << 8 | 1 << 0);
            if(row % 2) {
                Q6_l2fetch_AR((void*)(downScaled + width / 2), width / 2 << 16 | width / 2 << 8 | 1 << 0);
            }
        }

        #pragma clang loop interleave(enable) vectorize(enable) unroll(enable)
        for (int hl_col = 0; hl_col < width; hl_col +=2) {
            int ll_col = hl_col / 2;
            uint16_t expanded = downScaled[ll_col] * downscaleFactor;
            downScaledExpanded[hl_col] = expanded;
            downScaledExpanded[hl_col+1] = expanded;
        }

        HVX_Vector v_half_disp = Q6_Vh_vsplat_R(half_disp);

        // TODO HANDLE ALIGNMENT TODO
        #pragma clang loop interleave(enable) unroll(enable)
        for (int hl_col = 0; hl_col < width; hl_col += sizeof(HVX_Vector) / sizeof(uint16_t)) {

            HVX_Vector v_downScaled = *(HVX_Vector*)&downScaledExpanded[hl_col];
            HVX_Vector v_reference = *(HVX_Vector*)&reference[hl_col];

            HVX_VectorPred pred = Q6_Q_vcmp_gt_VuhVuh(v_downScaled, v_half_disp);

            HVX_Vector v_output = Q6_V_vmux_QVV(pred, v_downScaled, v_reference);

            *(HVX_Vector*)&output[hl_col] = v_output;
        }

        reference += width;
        output += width;

        if(row % 2) downScaled += width / 2;
    }

    return 0;

}
