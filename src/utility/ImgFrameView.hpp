#pragma once

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT

    #include <opencv2/core/mat.hpp>

    #include "depthai/pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace utility {

inline cv::Mat getCvFrameView(const ImgFrame& frame) {  // Caller keeps the frame buffer alive; the pixels are read-only
    ImgFrame view = frame;                              // OpenCV accessors are logically const but have no const overload.
    const auto type = frame.getType();
    if(type == ImgFrame::Type::BGR888i || type == ImgFrame::Type::BGRF16F16F16i || type == ImgFrame::Type::RGB161616 || type == ImgFrame::Type::RAW8
       || type == ImgFrame::Type::RAW10 || type == ImgFrame::Type::RAW12 || type == ImgFrame::Type::RAW14 || type == ImgFrame::Type::RAW16
       || type == ImgFrame::Type::RAW32 || type == ImgFrame::Type::GRAY8 || type == ImgFrame::Type::GRAYF16) {
        return view.getFrame();
    }
    return view.getCvFrame();
}

}  // namespace utility
}  // namespace dai

#endif
