// needed here
#include "depthai/utility/Serialization.hpp"
// needed here

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <vector>

#include "HAP_farf.h"
#include "ImageAlignDsp.h"
#include "ImageAlignDspConfig.h"
#include "hexagon_types.h"
#include "hexagon_protos.h"

int ImageAlignDsp_open(const char* uri, remote_handle64* handle) {
    // Keep the context here. Now it's just a magic number
    *handle = 0x00DEAD00;

    FARF(ALWAYS, "SpatialLocationCalculator open session\n");

    return 0;
}

/**
 * @param handle, the value returned by open
 * @retval, 0 for success, should always succeed
 */
int ImageAlignDsp_close(remote_handle64 handle) {
    FARF(ALWAYS, "SpatialLocationCalculator close session\n");

    return 0;
}

static int depthLut[65536] __attribute__((aligned(128))) __attribute__((section(".bss.noinit")));
static bool depthLutInitialized = false;
static float cachedShift = 0.0f;

int ImageAlignDsp_process(remote_handle64 h,
                                      const unsigned char* inVec,
                                      int inLen,
                                      const unsigned char* inConfig,
                                      int configLen,
                                      unsigned char* outVec,
                                      int outLen) {

    if(configLen != sizeof(ImageAlignDspConfig)) {
        return __LINE__;
    }

    ImageAlignDspConfig config = *(ImageAlignDspConfig*)inConfig;

    auto& depthIntrinsics = config.depthSourceIntrinsics;
    auto& cameraExtrinsics = config.depthToAlignExtrinsics;

    int width = config.width;
    int height = config.height;
    int stride = config.stride; //todo
    int bpp = 2; //todo

    const uint16_t* plane = reinterpret_cast<const uint16_t*>(inVec);
    uint16_t* alignedPlane = reinterpret_cast<uint16_t*>(outVec);

    // assert(!(bpp != sizeof(*plane)) && "Wrong in/out size");
    // assert(!(bpp != sizeof(*alignedPlane)) && "Wrong in/out size");

    const int lineLength = width * bpp;

    float shiftX = cameraExtrinsics[0][3];

    float depthFx = depthIntrinsics[0][0];
    // float depthCx = depthIntrinsics[0][2];

    float shiftXPreComputed = shiftX * depthFx;

    if(!depthLutInitialized || cachedShift != shiftXPreComputed) {
        for(int i = 1; i < 65536; i++) {
            depthLut[i] = static_cast<int>(roundf(shiftXPreComputed / static_cast<float>(i)));
        }
        cachedShift = shiftXPreComputed;
        depthLutInitialized = true;
    }

    for (int i = 0; i < height; i++) {

        const uint16_t* __restrict__ currentLine = plane + width * i;
        uint16_t* __restrict__ alignedLine = alignedPlane + width * i;

        memset(alignedLine, 0, lineLength);

        if(shiftX > 0) {

            #pragma clang loop interleave(enable) unroll(enable)
            for(int j = width - 1; j >= 0; j--) {

                uint16_t depth = currentLine[j];
                if(depth == 0) continue;
                if(depthLut[depth] == 0) {
                    alignedLine[j] = depth;
                    continue;
                }

                int int_u = j + depthLut[depth];

                if(int_u >= 0 && int_u < width) alignedLine[int_u] = depth;
                if(int_u + 1 >= 0 && int_u + 1 < width) alignedLine[int_u + 1] = depth;
            }

        } else {

            #pragma clang loop interleave(enable) unroll(enable)
            for(int j = 0; j < width; j++) {

                uint16_t depth = currentLine[j];
                if(depth == 0) continue;
                if(depthLut[depth] == 0) {
                    alignedLine[j] = depth;
                    continue;
                }

                int int_u = j + depthLut[depth];

                if(int_u >= 0 && int_u < width) alignedLine[int_u] = depth;
                if(int_u - 1 >= 0 && int_u - 1 < width) alignedLine[int_u - 1] = depth;
            }
        }

    }

    return 0;

}
