#pragma once
#include <memory>

#include "Deblur.hpp"
namespace dai {
namespace gate {
namespace deblur {
/** Owns a GPU context and reusable staged buffers. No global OpenCL state. */
class OpenCLFilter {
   public:
    OpenCLFilter();
    ~OpenCLFilter();
    OpenCLFilter(const OpenCLFilter&) = delete;
    OpenCLFilter& operator=(const OpenCLFilter&) = delete;
    cv::Mat apply(const Field& field);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
}  // namespace deblur
}  // namespace gate
}  // namespace dai
