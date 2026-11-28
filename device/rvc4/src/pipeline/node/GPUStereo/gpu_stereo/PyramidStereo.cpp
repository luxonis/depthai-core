#include "gpu_stereo/PyramidStereo.hpp"
#include "gpu_stereo/config.hpp"
#include "cl_kernels.h"
#include <CL/cl.h>
#include <vector>
#include <string>
#include <stdexcept>
#include <cstring>
#include <iostream>
#include <algorithm>
#include <cstdint>

#ifdef TIMING
#include <chrono>
#define TIMING_START(name) auto t_##name = std::chrono::high_resolution_clock::now()
#define TIMING_CHECKPOINT(queue) clFinish(queue)
#define TIMING_END(name, label) do { \
  auto t_end = std::chrono::high_resolution_clock::now(); \
  double ms = std::chrono::duration<double, std::milli>(t_end - t_##name).count(); \
  add_timing(label, ms); \
} while(0)
#else
#define TIMING_START(name) do {} while(0)
#define TIMING_CHECKPOINT(queue) do {} while(0)
#define TIMING_END(name, label) do {} while(0)
#endif

namespace gpu_stereo {

struct PyramidStereo::Impl {
    cl_context ctx;
    cl_device_id device;
    Config config;
    ProfilingData profiling_data;
    cl_program program = nullptr;

    cl_kernel k_buffer_to_image = nullptr;
    cl_kernel k_downsample_box_2x = nullptr;
    cl_kernel k_buffer_downsample_box_2x = nullptr;
    cl_kernel k_prefilter_gaussian3 = nullptr;
    cl_kernel k_block_match_zncc = nullptr;
    cl_kernel k_block_match_refine_zncc = nullptr;
    cl_kernel k_upsample_disp_2x = nullptr;
    cl_kernel k_subpixel_refine_zncc = nullptr;
    cl_kernel k_right_owner_scatter = nullptr;
    cl_kernel k_right_disp_finalize = nullptr;
    cl_kernel k_lr_check = nullptr;
    cl_kernel k_confidence_filter = nullptr;
    cl_kernel k_winner_cost_to_confidence = nullptr;
    cl_kernel k_median_3x3 = nullptr;

    std::vector<cl_mem> left_images;
    std::vector<cl_mem> right_images;
    std::vector<cl_mem> disp_buffers;
    std::vector<cl_mem> prefilter_tmp_left_;
    std::vector<cl_mem> prefilter_tmp_right_;
    cl_mem right_disp_buffer = nullptr;
    cl_mem right_owner_x_buffer = nullptr;
    cl_mem disp_scratch_u16_buf_ = nullptr;
    cl_mem confidence_winner_cost_buf_ = nullptr;

    int current_w = 0;
    int current_h = 0;
    size_t max_work_group_size = 1;

    double get_event_time_ms(cl_event event) {
        if (!profiling_data.enabled || !event) return 0.0;
        cl_ulong start = 0, end = 0;
        clGetEventProfilingInfo(event, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &start, nullptr);
        clGetEventProfilingInfo(event, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &end, nullptr);
        return (end - start) / 1000000.0;
    }

    void add_timing(const std::string& name, double ms) {
        if (!profiling_data.enabled) return;
        if (profiling_data.timings_ms.find(name) == profiling_data.timings_ms.end()) {
            profiling_data.timings_ms[name] = ms;
        } else {
            profiling_data.timings_ms[name] += ms;
        }
    }

    Impl(cl_context c, cl_device_id d, const Config& cfg)
        : ctx(c), device(d), config(cfg) {
        cl_int err;
        err = clGetDeviceInfo(device, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(max_work_group_size), &max_work_group_size, nullptr);
        if (err != CL_SUCCESS || max_work_group_size == 0) max_work_group_size = 1;
        build_program();
    }

    ~Impl() {
        release_resources();
        release_all_kernels();
        if (program) clReleaseProgram(program);
    }

    void release_all_kernels() {
#define RELK(k) do { if (k) { clReleaseKernel(k); k = nullptr; } } while(0)
        RELK(k_buffer_to_image);
        RELK(k_downsample_box_2x);
        RELK(k_buffer_downsample_box_2x);
        RELK(k_prefilter_gaussian3);
        RELK(k_block_match_zncc);
        RELK(k_block_match_refine_zncc);
        RELK(k_upsample_disp_2x);
        RELK(k_subpixel_refine_zncc);
        RELK(k_right_owner_scatter);
        RELK(k_right_disp_finalize);
        RELK(k_lr_check);
        RELK(k_confidence_filter);
        RELK(k_winner_cost_to_confidence);
        RELK(k_median_3x3);
#undef RELK
    }

    void build_program() {
        if (config.block_match_radius <= 0) {
            throw std::runtime_error("GPUStereo invalid block_match_radius (must be > 0)");
        }
        std::string build_opts =
            "-cl-fast-relaxed-math -DBLOCK_RADIUS=" + std::to_string(config.block_match_radius);
        const char* src = kernels::pyramid_stereo_src();
        size_t len = std::strlen(src);
        cl_int err;
        cl_program prog = clCreateProgramWithSource(ctx, 1, &src, &len, &err);
        if (err != CL_SUCCESS || !prog) throw std::runtime_error("clCreateProgramWithSource failed");
        err = clBuildProgram(prog, 1, &device, build_opts.c_str(), nullptr, nullptr);
        if (err != CL_SUCCESS) {
            size_t log_len = 0;
            clGetProgramBuildInfo(prog, device, CL_PROGRAM_BUILD_LOG, 0, nullptr, &log_len);
            std::string log(log_len, '\0');
            clGetProgramBuildInfo(prog, device, CL_PROGRAM_BUILD_LOG, log_len, &log[0], nullptr);
            std::cerr << "Build log: " << log << std::endl;
            clReleaseProgram(prog);
            throw std::runtime_error("clBuildProgram failed");
        }
        program = prog;
        try {
            k_buffer_to_image = get_kernel("buffer_to_image");
            k_downsample_box_2x = get_kernel("downsample_box_2x");
            k_buffer_downsample_box_2x = get_kernel("buffer_downsample_box_2x");
            k_prefilter_gaussian3 = get_kernel("prefilter_gaussian3");
            k_block_match_zncc = get_kernel("block_match_zncc");
            k_block_match_refine_zncc = get_kernel("block_match_refine_zncc");
            k_upsample_disp_2x = get_kernel("upsample_disp_2x");
            k_subpixel_refine_zncc = get_kernel("subpixel_refine_zncc");
            k_right_owner_scatter = get_kernel("right_disp_from_left_scatter_owner_x");
            k_right_disp_finalize = get_kernel("right_disp_from_left_finalize");
            k_lr_check = get_kernel("lr_check");
            k_confidence_filter = get_kernel("confidence_filter");
            k_winner_cost_to_confidence = get_kernel("winner_cost_to_confidence");
            k_median_3x3 = get_kernel("median_3x3");
        } catch (...) {
            release_all_kernels();
            if (program) {
                clReleaseProgram(program);
                program = nullptr;
            }
            throw;
        }
    }

    cl_kernel get_kernel(const char* name) {
        cl_int err;
        cl_kernel k = clCreateKernel(program, name, &err);
        if (err != CL_SUCCESS) throw std::runtime_error(std::string("clCreateKernel ") + name + " failed");
        return k;
    }

    size_t kernel_max_work_group_size(cl_kernel k) {
        size_t kwg = max_work_group_size;
        cl_int err = clGetKernelWorkGroupInfo(k, device, CL_KERNEL_WORK_GROUP_SIZE, sizeof(kwg), &kwg, nullptr);
        if (err == CL_SUCCESS && kwg > 0) return (kwg < max_work_group_size) ? kwg : max_work_group_size;
        return max_work_group_size;
    }

    void enqueue_2d(cl_command_queue queue, cl_kernel k, size_t gw, size_t gh) {
        size_t lim = kernel_max_work_group_size(k);
        if (gw == 0) gw = 1;
        if (gh == 0) gh = 1;
        size_t lx = 8, ly = 8;
        if (lx > gw) lx = gw;
        if (ly > gh) ly = gh;
        while (lx * ly > lim && (lx > 1 || ly > 1)) {
            if (lx > ly) lx /= 2;
            else ly /= 2;
            if (lx > gw) lx = gw;
            if (ly > gh) ly = gh;
        }
        if (lx == 0) lx = 1;
        if (ly == 0) ly = 1;
        size_t gx = ((gw + lx - 1) / lx) * lx;
        size_t gy = ((gh + ly - 1) / ly) * ly;
        size_t global[2] = { gx, gy };
        size_t local[2] = { lx, ly };
        cl_int err = clEnqueueNDRangeKernel(queue, k, 2, nullptr, global, local, 0, nullptr, nullptr);
        if(err != CL_SUCCESS) {
            char kernelName[256] = {};
            (void)clGetKernelInfo(k, CL_KERNEL_FUNCTION_NAME, sizeof(kernelName), kernelName, nullptr);
            throw std::runtime_error(std::string("clEnqueueNDRangeKernel 2D failed (kernel=") + kernelName + ", err=" + std::to_string(err) + ")");
        }
    }

    void enqueue_1d(cl_command_queue queue, cl_kernel k, size_t gw) {
        size_t lim = kernel_max_work_group_size(k);
        if (gw == 0) gw = 1;
        size_t lw = 64;
        if (lw > lim) lw = lim;
        if (lw > gw) lw = gw;
        if (lw == 0) lw = 1;
        size_t gx = ((gw + lw - 1) / lw) * lw;
        size_t global[1] = { gx };
        size_t local[1] = { lw };
        cl_int err = clEnqueueNDRangeKernel(queue, k, 1, nullptr, global, local, 0, nullptr, nullptr);
        if (err != CL_SUCCESS) throw std::runtime_error("clEnqueueNDRangeKernel 1D failed");
    }

    cl_mem_flags buffer_flags(cl_mem_flags base) const {
        return base | CL_MEM_ALLOC_HOST_PTR;
    }

    void ensure_resources(int w, int h) {
        if (w == current_w && h == current_h) return;
        release_resources();
        current_w = w;
        current_h = h;
        cl_int err;
        cl_image_format fmt = { CL_R, CL_FLOAT };
        int n = config.num_pyramid_levels;
        left_images.resize(n);
        right_images.resize(n);
        disp_buffers.resize(n);
        for (int l = 0; l < n; l++) {
            int lw = w >> l, lh = h >> l;
            if (lw < 1 || lh < 1) break;
            cl_image_desc desc = {};
            desc.image_type = CL_MEM_OBJECT_IMAGE2D;
            desc.image_width = lw;
            desc.image_height = lh;
            left_images[l] = clCreateImage(ctx, CL_MEM_READ_WRITE, &fmt, &desc, nullptr, &err);
            if (err != CL_SUCCESS) throw std::runtime_error("clCreateImage left failed");
            right_images[l] = clCreateImage(ctx, CL_MEM_READ_WRITE, &fmt, &desc, nullptr, &err);
            if (err != CL_SUCCESS) throw std::runtime_error("clCreateImage right failed");
            disp_buffers[l] = clCreateBuffer(ctx, buffer_flags(CL_MEM_READ_WRITE), (size_t)lw * lh * sizeof(cl_ushort), nullptr, &err);
            if (err != CL_SUCCESS) throw std::runtime_error("clCreateBuffer disp failed");
        }
        prefilter_tmp_left_.assign(n, nullptr);
        prefilter_tmp_right_.assign(n, nullptr);
        for (int l = 0; l < n; l++) {
            int lw = w >> l, lh = h >> l;
            if (lw < 1 || lh < 1) break;
            cl_image_desc desc = {};
            desc.image_type = CL_MEM_OBJECT_IMAGE2D;
            desc.image_width = lw;
            desc.image_height = lh;
            prefilter_tmp_left_[l] = clCreateImage(ctx, CL_MEM_READ_WRITE, &fmt, &desc, nullptr, &err);
            if (err != CL_SUCCESS || !prefilter_tmp_left_[l]) throw std::runtime_error("clCreateImage prefilter_tmp_left failed");
            prefilter_tmp_right_[l] = clCreateImage(ctx, CL_MEM_READ_WRITE, &fmt, &desc, nullptr, &err);
            if (err != CL_SUCCESS || !prefilter_tmp_right_[l]) throw std::runtime_error("clCreateImage prefilter_tmp_right failed");
        }
        right_disp_buffer = clCreateBuffer(ctx, buffer_flags(CL_MEM_READ_WRITE), (size_t)w * h * sizeof(cl_ushort), nullptr, &err);
        if (err != CL_SUCCESS) throw std::runtime_error("clCreateBuffer right_disp failed");
        right_owner_x_buffer = clCreateBuffer(ctx, buffer_flags(CL_MEM_READ_WRITE), (size_t)w * h * sizeof(cl_int), nullptr, &err);
        if (err != CL_SUCCESS) throw std::runtime_error("clCreateBuffer right_owner_x failed");
        size_t pix = (size_t)w * (size_t)h;
        disp_scratch_u16_buf_ = clCreateBuffer(ctx, buffer_flags(CL_MEM_READ_WRITE), pix * sizeof(cl_ushort), nullptr, &err);
        if (err != CL_SUCCESS || !disp_scratch_u16_buf_) throw std::runtime_error("clCreateBuffer disp_scratch_u16 failed");
        if (config.confidence_threshold > 0 || config.output_confidence) {
            confidence_winner_cost_buf_ =
                clCreateBuffer(ctx, buffer_flags(CL_MEM_READ_WRITE), pix * sizeof(cl_float), nullptr, &err);
            if (err != CL_SUCCESS || !confidence_winner_cost_buf_)
                throw std::runtime_error("clCreateBuffer confidence_winner_cost failed");
        }
    }

    void release_resources() {
        for (cl_mem m : prefilter_tmp_left_) { if (m) clReleaseMemObject(m); }
        for (cl_mem m : prefilter_tmp_right_) { if (m) clReleaseMemObject(m); }
        prefilter_tmp_left_.clear();
        prefilter_tmp_right_.clear();
        for (cl_mem m : left_images) { if (m) clReleaseMemObject(m); }
        for (cl_mem m : right_images) { if (m) clReleaseMemObject(m); }
        for (cl_mem m : disp_buffers) { if (m) clReleaseMemObject(m); }
        left_images.clear();
        right_images.clear();
        disp_buffers.clear();
        if (right_disp_buffer) { clReleaseMemObject(right_disp_buffer); right_disp_buffer = nullptr; }
        if (right_owner_x_buffer) { clReleaseMemObject(right_owner_x_buffer); right_owner_x_buffer = nullptr; }
        if (disp_scratch_u16_buf_) { clReleaseMemObject(disp_scratch_u16_buf_); disp_scratch_u16_buf_ = nullptr; }
        if (confidence_winner_cost_buf_) { clReleaseMemObject(confidence_winner_cost_buf_); confidence_winner_cost_buf_ = nullptr; }
        current_w = current_h = 0;
    }

    void execute(cl_command_queue queue, cl_mem left_buf, cl_mem right_buf, cl_mem disparity_buf, cl_mem confidence_buf) {
        TIMING_START(total);
#ifdef TIMING
        profiling_data.timings_ms.clear();
        profiling_data.total_ms = 0.0;
#endif

        int w = config.width, h = config.height;
        if (w <= 0 || h <= 0) throw std::runtime_error("invalid config width/height");
        cl_int err = CL_SUCCESS;
        ensure_resources(w, h);
        int n = config.num_pyramid_levels;
        int max_d = config.max_disparity;

        TIMING_START(upload);
        err = clSetKernelArg(k_buffer_to_image, 0, sizeof(cl_mem), &left_buf);
        err = clSetKernelArg(k_buffer_to_image, 1, sizeof(cl_mem), &left_images[0]);
        err = clSetKernelArg(k_buffer_to_image, 2, sizeof(int), &w);
        err = clSetKernelArg(k_buffer_to_image, 3, sizeof(int), &h);
        size_t gwx = (size_t)w, gwy = (size_t)h;
        enqueue_2d(queue, k_buffer_to_image, gwx, gwy);
        err = clSetKernelArg(k_buffer_to_image, 0, sizeof(cl_mem), &right_buf);
        err = clSetKernelArg(k_buffer_to_image, 1, sizeof(cl_mem), &right_images[0]);
        enqueue_2d(queue, k_buffer_to_image, gwx, gwy);
        TIMING_CHECKPOINT(queue);
        TIMING_END(upload, "upload");

        TIMING_START(pyramid_build);
        for (int l = 1; l < n; l++) {
            int lw = w >> l, lh = h >> l;
            if (lw < 1 || lh < 1) break;
            size_t gw = lw, gh = lh;
            if (l == 1) {
                err = clSetKernelArg(k_buffer_downsample_box_2x, 0, sizeof(cl_mem), &left_buf);
                err = clSetKernelArg(k_buffer_downsample_box_2x, 1, sizeof(cl_mem), &left_images[l]);
                err = clSetKernelArg(k_buffer_downsample_box_2x, 2, sizeof(int), &w);
                err = clSetKernelArg(k_buffer_downsample_box_2x, 3, sizeof(int), &h);
                enqueue_2d(queue, k_buffer_downsample_box_2x, gw, gh);
                err = clSetKernelArg(k_buffer_downsample_box_2x, 0, sizeof(cl_mem), &right_buf);
                err = clSetKernelArg(k_buffer_downsample_box_2x, 1, sizeof(cl_mem), &right_images[l]);
                err = clSetKernelArg(k_buffer_downsample_box_2x, 2, sizeof(int), &w);
                err = clSetKernelArg(k_buffer_downsample_box_2x, 3, sizeof(int), &h);
                enqueue_2d(queue, k_buffer_downsample_box_2x, gw, gh);
            } else {
                err = clSetKernelArg(k_downsample_box_2x, 0, sizeof(cl_mem), &left_images[l-1]);
                err = clSetKernelArg(k_downsample_box_2x, 1, sizeof(cl_mem), &left_images[l]);
                enqueue_2d(queue, k_downsample_box_2x, gw, gh);
                err = clSetKernelArg(k_downsample_box_2x, 0, sizeof(cl_mem), &right_images[l-1]);
                err = clSetKernelArg(k_downsample_box_2x, 1, sizeof(cl_mem), &right_images[l]);
                enqueue_2d(queue, k_downsample_box_2x, gw, gh);
            }
        }
        TIMING_CHECKPOINT(queue);
        TIMING_END(pyramid_build, "pyramid_build");

        TIMING_START(prefilter);  // Handles are swapped, not copied: no code may cache left_images/right_images cl_mem across execute()
        for (int l = 0; l < n; l++) {
            int lw = w >> l, lh = h >> l;
            if (lw < 1 || lh < 1) break;
            size_t gw = lw, gh = lh;
            cl_mem tL = prefilter_tmp_left_[l];
            cl_mem tR = prefilter_tmp_right_[l];
            if (!tL || !tR) throw std::runtime_error("prefilter scratch images not allocated");
            err = clSetKernelArg(k_prefilter_gaussian3, 0, sizeof(cl_mem), &left_images[l]);
            err = clSetKernelArg(k_prefilter_gaussian3, 1, sizeof(cl_mem), &tL);
            enqueue_2d(queue, k_prefilter_gaussian3, gw, gh);
            std::swap(left_images[l], prefilter_tmp_left_[l]);
            err = clSetKernelArg(k_prefilter_gaussian3, 0, sizeof(cl_mem), &right_images[l]);
            err = clSetKernelArg(k_prefilter_gaussian3, 1, sizeof(cl_mem), &tR);
            enqueue_2d(queue, k_prefilter_gaussian3, gw, gh);
            std::swap(right_images[l], prefilter_tmp_right_[l]);
        }
        TIMING_CHECKPOINT(queue);
        TIMING_END(prefilter, "prefilter");

        TIMING_START(coarse_match);
        int coarse_l = n - 1;
        int cw = w >> coarse_l, ch = h >> coarse_l;
        int d_max_coarse = max_d >> coarse_l;
        if (d_max_coarse > cw) d_max_coarse = cw;
        size_t gwc = (size_t)cw, ghc = (size_t)ch;
        int min_disp = 0;
        int epipolar_v_half = 0;
        err = clSetKernelArg(k_block_match_zncc, 0, sizeof(cl_mem), &left_images[coarse_l]);
        err = clSetKernelArg(k_block_match_zncc, 1, sizeof(cl_mem), &right_images[coarse_l]);
        err = clSetKernelArg(k_block_match_zncc, 2, sizeof(cl_mem), &disp_buffers[coarse_l]);
        err = clSetKernelArg(k_block_match_zncc, 3, sizeof(int), &min_disp);
        err = clSetKernelArg(k_block_match_zncc, 4, sizeof(int), &d_max_coarse);
        err = clSetKernelArg(k_block_match_zncc, 5, sizeof(int), &cw);
        err = clSetKernelArg(k_block_match_zncc, 6, sizeof(int), &ch);
        err = clSetKernelArg(k_block_match_zncc, 7, sizeof(int), &epipolar_v_half);
        enqueue_2d(queue, k_block_match_zncc, gwc, ghc);
        TIMING_CHECKPOINT(queue);
        TIMING_END(coarse_match, "coarse_match");

        TIMING_START(pyramid_refine);
        for(int l = coarse_l - 1; l >= 0; l--) {
            int lw = w >> l, lh = h >> l;
            int pw = w >> (l + 1), ph = h >> (l + 1);

            err = clSetKernelArg(k_upsample_disp_2x, 0, sizeof(cl_mem), &disp_buffers[l + 1]);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg upsample_disp_2x(0) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_upsample_disp_2x, 1, sizeof(cl_mem), &disp_buffers[l]);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg upsample_disp_2x(1) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_upsample_disp_2x, 2, sizeof(int), &pw);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg upsample_disp_2x(2) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_upsample_disp_2x, 3, sizeof(int), &ph);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg upsample_disp_2x(3) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_upsample_disp_2x, 4, sizeof(int), &lw);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg upsample_disp_2x(4) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_upsample_disp_2x, 5, sizeof(int), &lh);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg upsample_disp_2x(5) failed, err=" + std::to_string(err));
            enqueue_2d(queue, k_upsample_disp_2x, (size_t)lw, (size_t)lh);

            int r = (l == 0) ? config.refinement_radius_full : config.refinement_radius;
            size_t grw = (size_t)lw, grh = (size_t)lh;
            err = clSetKernelArg(k_block_match_refine_zncc, 0, sizeof(cl_mem), &left_images[l]);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(0) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_block_match_refine_zncc, 1, sizeof(cl_mem), &right_images[l]);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(1) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_block_match_refine_zncc, 2, sizeof(cl_mem), &disp_buffers[l]);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(2) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_block_match_refine_zncc, 3, sizeof(int), &r);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(3) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_block_match_refine_zncc, 4, sizeof(int), &lw);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(4) failed, err=" + std::to_string(err));
            err = clSetKernelArg(k_block_match_refine_zncc, 5, sizeof(int), &lh);
            if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(5) failed, err=" + std::to_string(err));

            int d_max_contract = max_d >> l;
            if(d_max_contract > lw - 1) d_max_contract = lw - 1;
            if(d_max_contract < 0) d_max_contract = 0;

	        cl_mem wc_out = confidence_winner_cost_buf_;
	        int store_wc = (confidence_winner_cost_buf_ && l == 0 && (config.confidence_threshold > 0 || config.output_confidence)) ? 1 : 0;
	        err = clSetKernelArg(k_block_match_refine_zncc, 6, sizeof(cl_mem), &wc_out);
	        if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(6) failed, err=" + std::to_string(err));
	        err = clSetKernelArg(k_block_match_refine_zncc, 7, sizeof(int), &store_wc);
	        if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(7) failed, err=" + std::to_string(err));
	        err = clSetKernelArg(k_block_match_refine_zncc, 8, sizeof(int), &epipolar_v_half);
	        if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(8) failed, err=" + std::to_string(err));
	        err = clSetKernelArg(k_block_match_refine_zncc, 9, sizeof(int), &d_max_contract);
	        if(err != CL_SUCCESS) throw std::runtime_error("clSetKernelArg block_match_refine_zncc(9) failed, err=" + std::to_string(err));
	        enqueue_2d(queue, k_block_match_refine_zncc, grw, grh);
	        }
        TIMING_CHECKPOINT(queue);
        TIMING_END(pyramid_refine, "pyramid_refine");

        cl_mem current_disp = disp_buffers[0];
        cl_mem scratch_disp = disp_scratch_u16_buf_;
        if(!scratch_disp) throw std::runtime_error("disp scratch buffer not allocated");

        if(confidence_buf) {  // Before LR-check/median filter on purpose: matches the StereoDepth confidence map semantics
            if(!confidence_winner_cost_buf_) throw std::runtime_error("confidence output requested but winner cost buffer is not allocated");
            float cost_max = 2.0f;  // Winner cost is (1 - ZNCC_rho), typically in [0, 2]

            err = clSetKernelArg(k_winner_cost_to_confidence, 0, sizeof(cl_mem), &confidence_winner_cost_buf_);
            err = clSetKernelArg(k_winner_cost_to_confidence, 1, sizeof(cl_mem), &confidence_buf);
            err = clSetKernelArg(k_winner_cost_to_confidence, 2, sizeof(float), &cost_max);
            err = clSetKernelArg(k_winner_cost_to_confidence, 3, sizeof(int), &w);
            err = clSetKernelArg(k_winner_cost_to_confidence, 4, sizeof(int), &h);
            enqueue_2d(queue, k_winner_cost_to_confidence, (size_t)w, (size_t)h);
        }

        if(config.confidence_threshold > 0) {
            TIMING_START(confidence_filter);
            const float cost_max = 2.0f;  // Winner cost is (1 - ZNCC_rho), typically in [0, 2]; 0 is the best match
            float cost_threshold = (1.0f - (float)config.confidence_threshold / 255.0f) * cost_max;
            int cost_mode_conf = 1;  // ZNCC mode
            cl_mem desc_l = nullptr, desc_r = nullptr;
            int conf_disp_pack_bits = 0;
            err = clSetKernelArg(k_confidence_filter, 0, sizeof(cl_mem), &left_images[0]);
            err = clSetKernelArg(k_confidence_filter, 1, sizeof(cl_mem), &right_images[0]);
            err = clSetKernelArg(k_confidence_filter, 2, sizeof(cl_mem), &current_disp);
            err = clSetKernelArg(k_confidence_filter, 3, sizeof(cl_mem), &scratch_disp);
            err = clSetKernelArg(k_confidence_filter, 4, sizeof(int), &conf_disp_pack_bits);
            err = clSetKernelArg(k_confidence_filter, 5, sizeof(float), &cost_threshold);
            err = clSetKernelArg(k_confidence_filter, 6, sizeof(int), &w);
            err = clSetKernelArg(k_confidence_filter, 7, sizeof(int), &h);
            err = clSetKernelArg(k_confidence_filter, 8, sizeof(int), &cost_mode_conf);
            err = clSetKernelArg(k_confidence_filter, 9, sizeof(cl_mem), &desc_l);
            err = clSetKernelArg(k_confidence_filter, 10, sizeof(cl_mem), &desc_r);
            cl_mem wc_in = confidence_winner_cost_buf_;
            int use_stored_winner = confidence_winner_cost_buf_ ? 1 : 0;
            err = clSetKernelArg(k_confidence_filter, 11, sizeof(cl_mem), &wc_in);
            err = clSetKernelArg(k_confidence_filter, 12, sizeof(int), &use_stored_winner);
            err = clSetKernelArg(k_confidence_filter, 13, sizeof(int), &epipolar_v_half);
            gwx = (size_t)w;
            gwy = (size_t)h;
            enqueue_2d(queue, k_confidence_filter, gwx, gwy);
            std::swap(current_disp, scratch_disp);
            TIMING_CHECKPOINT(queue);
            TIMING_END(confidence_filter, "confidence_filter");
        }

        {
            TIMING_START(subpixel_refine);
            gwx = (size_t)w;
            gwy = (size_t)h;
            err = clSetKernelArg(k_subpixel_refine_zncc, 0, sizeof(cl_mem), &left_images[0]);
            err = clSetKernelArg(k_subpixel_refine_zncc, 1, sizeof(cl_mem), &right_images[0]);
            err = clSetKernelArg(k_subpixel_refine_zncc, 2, sizeof(cl_mem), &current_disp);
            err = clSetKernelArg(k_subpixel_refine_zncc, 3, sizeof(int), &config.subpixel_bits);
            err = clSetKernelArg(k_subpixel_refine_zncc, 4, sizeof(int), &w);
            err = clSetKernelArg(k_subpixel_refine_zncc, 5, sizeof(int), &h);
            err = clSetKernelArg(k_subpixel_refine_zncc, 6, sizeof(int), &epipolar_v_half);
            enqueue_2d(queue, k_subpixel_refine_zncc, gwx, gwy);
            TIMING_CHECKPOINT(queue);
            TIMING_END(subpixel_refine, "subpixel_refine");
        }

        {
            TIMING_START(lr_check);
            gwx = (size_t)w;
            gwy = (size_t)h;
            cl_int neg1 = -1;
            err = clEnqueueFillBuffer(queue, right_owner_x_buffer, &neg1, sizeof(neg1), 0, (size_t)w * h * sizeof(cl_int), 0, nullptr, nullptr);
            if (err != CL_SUCCESS) throw std::runtime_error("clEnqueueFillBuffer right_owner_x failed");
            err = clSetKernelArg(k_right_owner_scatter, 0, sizeof(cl_mem), &current_disp);
            err = clSetKernelArg(k_right_owner_scatter, 1, sizeof(cl_mem), &right_owner_x_buffer);
            err = clSetKernelArg(k_right_owner_scatter, 2, sizeof(int), &w);
            err = clSetKernelArg(k_right_owner_scatter, 3, sizeof(int), &h);
            err = clSetKernelArg(k_right_owner_scatter, 4, sizeof(int), &config.subpixel_bits);
            int d_min_lr = 0;
            int d_max_lr = max_d;
            err = clSetKernelArg(k_right_owner_scatter, 5, sizeof(int), &d_min_lr);
            err = clSetKernelArg(k_right_owner_scatter, 6, sizeof(int), &d_max_lr);
            enqueue_2d(queue, k_right_owner_scatter, gwx, gwy);
            err = clSetKernelArg(k_right_disp_finalize, 0, sizeof(cl_mem), &current_disp);
            err = clSetKernelArg(k_right_disp_finalize, 1, sizeof(cl_mem), &right_owner_x_buffer);
            err = clSetKernelArg(k_right_disp_finalize, 2, sizeof(cl_mem), &right_disp_buffer);
            err = clSetKernelArg(k_right_disp_finalize, 3, sizeof(int), &w);
            err = clSetKernelArg(k_right_disp_finalize, 4, sizeof(int), &h);
            enqueue_2d(queue, k_right_disp_finalize, gwx, gwy);
            int thresh = (1 << config.subpixel_bits);
            err = clSetKernelArg(k_lr_check, 0, sizeof(cl_mem), &current_disp);
            err = clSetKernelArg(k_lr_check, 1, sizeof(cl_mem), &right_disp_buffer);
            err = clSetKernelArg(k_lr_check, 2, sizeof(int), &w);
            err = clSetKernelArg(k_lr_check, 3, sizeof(int), &h);
            err = clSetKernelArg(k_lr_check, 4, sizeof(int), &thresh);
            err = clSetKernelArg(k_lr_check, 5, sizeof(int), &config.subpixel_bits);
            enqueue_2d(queue, k_lr_check, gwx, gwy);
            TIMING_CHECKPOINT(queue);
            TIMING_END(lr_check, "lr_check");
        }

        {
            TIMING_START(median_filter);
            gwx = (size_t)w;
            gwy = (size_t)h;
            err = clSetKernelArg(k_median_3x3, 0, sizeof(cl_mem), &current_disp);
            err = clSetKernelArg(k_median_3x3, 1, sizeof(cl_mem), &disparity_buf);
            err = clSetKernelArg(k_median_3x3, 2, sizeof(int), &w);
            err = clSetKernelArg(k_median_3x3, 3, sizeof(int), &h);
            enqueue_2d(queue, k_median_3x3, gwx, gwy);
            current_disp = disparity_buf;
            TIMING_CHECKPOINT(queue);
            TIMING_END(median_filter, "median_filter");
        }

        TIMING_START(download);
        clFinish(queue);
        TIMING_END(download, "download");

        TIMING_CHECKPOINT(queue);
        TIMING_END(total, "total");
#ifdef TIMING
        profiling_data.total_ms = profiling_data.timings_ms["total"];
#endif
    }
};

PyramidStereo::PyramidStereo(cl_context ctx, cl_device_id device, const Config& config)
    : impl_(std::make_unique<Impl>(ctx, device, config)) {}

PyramidStereo::~PyramidStereo() = default;

void PyramidStereo::execute(cl_command_queue queue, cl_mem left_buf, cl_mem right_buf, cl_mem disparity_buf) {
    impl_->execute(queue, left_buf, right_buf, disparity_buf, nullptr);
}

void PyramidStereo::execute(cl_command_queue queue, cl_mem left_buf, cl_mem right_buf, cl_mem disparity_buf, cl_mem confidence_buf) {
    impl_->execute(queue, left_buf, right_buf, disparity_buf, confidence_buf);
}

void PyramidStereo::enable_profiling(bool enable) {
#ifdef TIMING
    impl_->profiling_data.enabled = enable;
#else
    (void)enable;
#endif
}

const ProfilingData& PyramidStereo::get_profiling_data() const {
    return impl_->profiling_data;
}

}
