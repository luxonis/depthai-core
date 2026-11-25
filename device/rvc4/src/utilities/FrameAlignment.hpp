#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>

#include "AcceleratorMemory.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace gate {

/**
 * Re-lays out NV12 and GRAY8 frames into the stride and plane alignment that the
 * vision accelerator expects (128-byte strides, 32/16-row planes, 4096-byte plane sizes).
 */
class FrameAlignment {
    static constexpr uint32_t alignUp(uint32_t value, uint32_t alignment) {
        return ((value + alignment - 1) / alignment) * alignment;
    }
    static constexpr uint32_t alignedWidth(uint32_t w) {
        return alignUp(w, 128);
    }
    static constexpr uint32_t alignedHeight(uint32_t h) {
        return alignUp(h, 32);
    }
    static constexpr uint32_t alignedHeight2(uint32_t h) {
        return alignUp(h, 16);
    }
    static constexpr uint32_t alignedSize(uint32_t s) {
        return alignUp(s, 4096);
    }

        static size_t getFirstPlaneSize(const dai::ImgFrame& frm) {
            switch(frm.getType()) {
                case ImgFrame::Type::RAW8:
                case ImgFrame::Type::GRAY8:
                case ImgFrame::Type::NV12:
                    {
                        int32_t offsetDiff = (int32_t)frm.fb.p2Offset - (int32_t)frm.fb.p1Offset;
                        if(offsetDiff >= frm.getStride() * frm.getHeight()) {
                            return (size_t)offsetDiff;
                        }
                        if(frm.getType() != dai::ImgFrame::Type::NV12) {
                            return frm.getData().size();
                        } // else fall through
                    }
                default:
                    return (size_t)frm.getPlaneStride(0);
            }
        }

        static size_t getSecondPlaneSize(const dai::ImgFrame& frm) {
            switch(frm.getType()) {
                case ImgFrame::Type::NV12:
                    {
                        int32_t offsetDiff = (int32_t)frm.fb.p3Offset - (int32_t)frm.fb.p2Offset;
                        if(offsetDiff >= frm.getStride() * frm.getHeight() / 2) {
                            return (size_t)offsetDiff;
                        }
                        return frm.getData().size() - frm.fb.p2Offset;
                    }
                default:
                    return (size_t)frm.getPlaneStride(1);
            }
        }

   public:
    // Converts only NV12 and GRAY8. Leaves the rest as is.
    static ImgFrame convertImgFrameMeta(const dai::ImgFrame& frame) {
        ImgFrame dst = frame;
        switch(frame.getType()) {
            case dai::ImgFrame::Type::NV12: {
                if(frame.getStride() != alignedWidth(frame.getStride()) || getFirstPlaneSize(frame) % 4096 != 0 || getSecondPlaneSize(frame) % 4096 != 0) {
                    dst.fb.stride = alignedWidth(frame.getStride());
                    const size_t yPlaneSize = dst.fb.stride * alignedHeight(frame.fb.height);
                    const size_t uvPlaneSize = alignedSize(dst.fb.stride * alignedHeight2(frame.fb.height / 2));

                    dst.fb.p2Offset = frame.fb.p1Offset + yPlaneSize;
                    dst.fb.p3Offset = dst.fb.p2Offset + uvPlaneSize;
                }
                break;
            }
            case dai::ImgFrame::Type::GRAY8: {
                if(frame.getStride() != alignedWidth(frame.getStride()) || getFirstPlaneSize(frame) % 4096 != 0) {
                    dst.fb.stride = alignedWidth(frame.getStride());
                    dst.fb.p2Offset = frame.fb.p1Offset + dst.fb.stride * alignedHeight(frame.fb.height);
                    dst.fb.p3Offset = dst.fb.p2Offset;
                }
                break;
            }
            default:
                break;
        }
        return dst;
    }

    static ImgFrame convertImgFrame(const dai::ImgFrame& frame) {
        ImgFrame dst = convertImgFrameMeta(frame);
        auto copyPlane = [](uint8_t* dstPtr, size_t dstStride, span<const uint8_t> srcData, size_t srcOffset, size_t srcStride, size_t rowBytes, size_t rows) {
            if(rowBytes == 0 || rows == 0 || srcOffset >= srcData.size()) {
                return;
            }

            if(srcStride == dstStride && rowBytes == dstStride) {
                const size_t copyBytes = std::min(dstStride * rows, srcData.size() - srcOffset);
                memcpy(dstPtr, srcData.data() + srcOffset, copyBytes);
                return;
            }

            for(size_t i = 0; i < rows; ++i) {
                const size_t rowOffset = srcOffset + i * srcStride;
                if(rowOffset >= srcData.size()) {
                    return;
                }

                const size_t copyBytes = std::min(rowBytes, srcData.size() - rowOffset);
                memcpy(dstPtr + i * dstStride, srcData.data() + rowOffset, copyBytes);
                if(copyBytes < rowBytes) {
                    return;
                }
            }
        };
        switch(frame.getType()) {
            case dai::ImgFrame::Type::NV12: {
                if(frame.getStride() == alignedWidth(frame.getStride()) && getFirstPlaneSize(frame) % 4096 == 0 && getSecondPlaneSize(frame) % 4096 == 0) {
                    dst.data = AcceleratorMemory::convert(frame.data);
                } else {
                    const size_t uvPlaneSize = alignedSize(dst.fb.stride * alignedHeight2(frame.fb.height / 2));

                    auto mem = dai::AcceleratorMemory::create(dst.fb.p2Offset + uvPlaneSize);
                    auto dstData = mem->getData();
                    const auto srcData = frame.getData();

                    copyPlane(dstData.data() + dst.fb.p1Offset,
                              dst.fb.stride,
                              srcData,
                              frame.fb.p1Offset,
                              frame.getStride(),
                              frame.fb.width,
                              frame.fb.height);
                    copyPlane(dstData.data() + dst.fb.p2Offset,
                              dst.fb.stride,
                              srcData,
                              frame.fb.p2Offset,
                              frame.getStride(),
                              frame.fb.width,
                              frame.fb.height / 2);

                    dst.data = mem;
                }
                break;
            }
            case dai::ImgFrame::Type::GRAY8: {
                if(frame.getStride() == alignedWidth(frame.getStride()) && getFirstPlaneSize(frame) % 4096 == 0) {
                    dst.data = AcceleratorMemory::convert(frame.data);
                } else {
                    auto mem = dai::AcceleratorMemory::create(dst.fb.p2Offset);
                    auto dstData = mem->getData();
                    copyPlane(dstData.data() + dst.fb.p1Offset,
                              dst.fb.stride,
                              frame.getData(),
                              frame.fb.p1Offset,
                              frame.getStride(),
                              frame.fb.width,
                              frame.fb.height);
                    dst.data = mem;
                }
                break;
            }
            default:
                dst.data = frame.data;
                break;
        }
        return dst;
    }

   private:
    FrameAlignment() {}
};

}  // namespace gate
}  // namespace dai
