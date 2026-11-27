#pragma once

#include <CL/cl.h>
#include "config.hpp"
#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace gpu_stereo {

struct ProfilingData {
    std::map<std::string, double> timings_ms;
    double total_ms = 0.0;
    bool enabled = false;
};

class PyramidStereo {
  public:
    explicit PyramidStereo(cl_context ctx, cl_device_id device, const Config& config);
    ~PyramidStereo();

    void execute(cl_command_queue queue, cl_mem left_buf, cl_mem right_buf, cl_mem disparity_buf);
    void execute(cl_command_queue queue, cl_mem left_buf, cl_mem right_buf, cl_mem disparity_buf, cl_mem confidence_buf);

    void enable_profiling(bool enable);
    const ProfilingData& get_profiling_data() const;

    PyramidStereo(const PyramidStereo&) = delete;
    PyramidStereo& operator=(const PyramidStereo&) = delete;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}
