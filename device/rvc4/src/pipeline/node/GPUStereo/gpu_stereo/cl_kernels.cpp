#include "cl_kernels.h"

namespace gpu_stereo {
namespace kernels {

const char* pyramid_stereo_src() {
    return R"CLC(
#ifndef BLOCK_RADIUS
#define BLOCK_RADIUS 2
#endif
#define SAD_BLOCK (2*BLOCK_RADIUS+1)

float read_gray(image2d_t img, int x, int y) {
  return read_imagef(img, (int2)(x, y)).x;
}

__kernel void buffer_to_image(__global const uchar* buf, write_only image2d_t img, int width, int height) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= width || y >= height) return;
  float v = (float)buf[y * width + x] / 255.0f;
  write_imagef(img, (int2)(x, y), (float4)(v, 0.0f, 0.0f, 1.0f));
}

__kernel void downsample_box_2x(read_only image2d_t in, write_only image2d_t out) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  int w = get_image_width(out);
  int h = get_image_height(out);
  if (x >= w || y >= h) return;
  float s = 0.0f;
  s += read_gray(in, 2*x,   2*y);
  s += read_gray(in, 2*x+1, 2*y);
  s += read_gray(in, 2*x,   2*y+1);
  s += read_gray(in, 2*x+1, 2*y+1);
  write_imagef(out, (int2)(x, y), (float4)(s * 0.25f, 0.0f, 0.0f, 1.0f));
}

__kernel void buffer_downsample_box_2x(__global const uchar* buf, write_only image2d_t out, int width, int height) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  int w = get_image_width(out);
  int h = get_image_height(out);
  if (x >= w || y >= h) return;
  float s = 0.0f;
  s += (float)buf[(2 * y) * width + (2 * x)] / 255.0f;
  s += (float)buf[(2 * y) * width + (2 * x + 1)] / 255.0f;
  s += (float)buf[(2 * y + 1) * width + (2 * x)] / 255.0f;
  s += (float)buf[(2 * y + 1) * width + (2 * x + 1)] / 255.0f;
  write_imagef(out, (int2)(x, y), (float4)(s * 0.25f, 0.0f, 0.0f, 1.0f));
}

__kernel void prefilter_gaussian3(read_only image2d_t in, write_only image2d_t out) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  int w = get_image_width(in);
  int h = get_image_height(in);
  if (x >= w || y >= h) return;
  const float k[3] = { 0.25f, 0.5f, 0.25f };
  float s = 0.0f;
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++) {
      int px = clamp(x + dx, 0, w - 1), py = clamp(y + dy, 0, h - 1);
      s += k[dy + 1] * k[dx + 1] * read_gray(in, px, py);
    }
  write_imagef(out, (int2)(x, y), (float4)(s, 0.0f, 0.0f, 1.0f));
}

float sad_block(read_only image2d_t left, read_only image2d_t right, int lx, int ly, int rx, int ry, int w, int h) {
  float s = 0.0f;
  int w1 = w - 1, h1 = h - 1;
  for (int dy = -BLOCK_RADIUS; dy <= BLOCK_RADIUS; dy++)
    for (int dx = -BLOCK_RADIUS; dx <= BLOCK_RADIUS; dx++) {
      int ax = clamp(lx + dx, 0, w1), ay = clamp(ly + dy, 0, h1);
      int bx = clamp(rx + dx, 0, w1), by = clamp(ry + dy, 0, h1);
      s += fabs(read_gray(left, ax, ay) - read_gray(right, bx, by));
    }
  return s;
}

uint hamming_bits(uint a, uint b) {
  return popcount(a ^ b);
}

float zncc_block(read_only image2d_t left, read_only image2d_t right, int lx, int ly, int rx, int ry, int w, int h) {
  float sumL = 0.0f, sumR = 0.0f, sumL2 = 0.0f, sumR2 = 0.0f, sumLR = 0.0f;
  int w1 = w - 1, h1 = h - 1;
  const float n = (float)(SAD_BLOCK * SAD_BLOCK);
  for (int dy = -BLOCK_RADIUS; dy <= BLOCK_RADIUS; dy++)
    for (int dx = -BLOCK_RADIUS; dx <= BLOCK_RADIUS; dx++) {
      int ax = clamp(lx + dx, 0, w1), ay = clamp(ly + dy, 0, h1);
      int bx = clamp(rx + dx, 0, w1), by = clamp(ry + dy, 0, h1);
      float L = read_gray(left, ax, ay);
      float R = read_gray(right, bx, by);
      sumL += L;
      sumR += R;
      sumL2 += L * L;
      sumR2 += R * R;
      sumLR += L * R;
    }
  float meanL = sumL / n;
  float meanR = sumR / n;
  float varL = sumL2 / n - meanL * meanL;
  float varR = sumR2 / n - meanR * meanR;
  float cov = sumLR / n - meanL * meanR;
  if (varL * varR < 1e-12f) return 1e30f;
  float rho = cov / sqrt(varL * varR);
  rho = clamp(rho, -1.0f, 1.0f);
  return 1.0f - rho;
}

float zncc_block_min_epipolar_v(read_only image2d_t img_a, read_only image2d_t img_b, int ax, int ay, int bx, int by0, int w, int h, int v_half) {
  int vh = v_half;
  if (vh < 0) vh = 0;
  if (vh > 2) vh = 2;
  float best = 1e30f;
  for (int dv = -vh; dv <= vh; dv++) {
    int by = by0 + dv;
    float c = zncc_block(img_a, img_b, ax, ay, bx, by, w, h);
    if (c < best) best = c;
  }
  return best;
}

__kernel void block_match_zncc(read_only image2d_t left, read_only image2d_t right,
                          __global ushort* disp, int d_min, int d_max, int w, int h, int epipolar_v_half) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w || y >= h) return;
  float best_cost = 1e30f;
  int best_d = d_min;
  for (int d = d_min; d <= d_max; d++) {
    int rx = x - d;
    if (rx < 0) continue;
    float c = zncc_block_min_epipolar_v(left, right, x, y, rx, y, w, h, epipolar_v_half);
    if (c >= 1e29f) c = 1e30f;
    if (c < best_cost) { best_cost = c; best_d = d; }
  }
  disp[y * w + x] = (ushort)best_d;
}

__kernel void block_match_refine_zncc(read_only image2d_t left, read_only image2d_t right,
                                 __global ushort* disp,
                                 int radius, int w, int h,
                                 __global float* winner_cost_out,
                                 int store_winner_cost, int epipolar_v_half,
                                 int max_d) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w || y >= h) return;
  int d0 = (int)disp[y * w + x];
  int d_min = max(0, d0 - radius);
  int d_max = min(w - 1, d0 + radius);
  float best_cost = 1e30f;
  int best_d = d0;
  for (int d = d_min; d <= d_max; d++) {
    int rx = x - d;
    if (rx < 0) continue;
    float c = zncc_block_min_epipolar_v(left, right, x, y, rx, y, w, h, epipolar_v_half);
    if (c >= 1e29f) c = 1e30f;
    if (c < best_cost) { best_cost = c; best_d = d; }
  }
  if (best_d > max_d) best_d = max_d;
  disp[y * w + x] = (ushort)best_d;
  if (store_winner_cost)
    winner_cost_out[y * w + x] = best_cost;
}

__kernel void upsample_disp_2x(__global const ushort* in, __global ushort* out, int w_in, int h_in, int w_out, int h_out) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w_out || y >= h_out) return;
  int ix = min(x / 2, w_in - 1);
  int iy = min(y / 2, h_in - 1);
  ushort v = in[iy * w_in + ix];
  out[y * w_out + x] = v * 2;
}

__kernel void subpixel_refine_zncc(read_only image2d_t left, read_only image2d_t right,
                              __global ushort* disp,
                              int subpixel_bits, int w, int h, int epipolar_v_half) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w || y >= h) return;
  int d = (int)disp[y * w + x];
  if (d == 0) return;
  int scale = 1 << subpixel_bits;
  float c0 = 1e30f, c1 = 1e30f, c2 = 1e30f;
  if (d - 1 >= 0 && x - (d - 1) >= 0) c0 = zncc_block_min_epipolar_v(left, right, x, y, x - (d - 1), y, w, h, epipolar_v_half);
  if (x - d >= 0) c1 = zncc_block_min_epipolar_v(left, right, x, y, x - d, y, w, h, epipolar_v_half);
  if (d + 1 < w && x - (d + 1) >= 0) c2 = zncc_block_min_epipolar_v(left, right, x, y, x - (d + 1), y, w, h, epipolar_v_half);
  if (c0 >= 1e29f) c0 = 1e30f;
  if (c1 >= 1e29f) c1 = 1e30f;
  if (c2 >= 1e29f) c2 = 1e30f;
  float denom = 2.0f * (c0 - 2.0f * c1 + c2);
  float d_sub = (float)d;
  if (fabs(denom) > 1e-6f)
    d_sub = d + (c0 - c2) / denom;
  int out_val = (int)round(d_sub * (float)scale);
  out_val = (out_val < 0) ? 0 : out_val;
  out_val = (out_val > 65535) ? 65535 : out_val;
  disp[y * w + x] = (ushort)out_val;
}

__kernel void median_3x3(__global const ushort* in, __global ushort* out, int w, int h) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w || y >= h) return;
  ushort v[9];
  int n = 0;
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++) {
      int nx = x + dx, ny = y + dy;
      if (nx >= 0 && nx < w && ny >= 0 && ny < h)
        v[n++] = in[ny * w + nx];
    }
  for (int i = 0; i < n - 1; i++)
    for (int j = 0; j < n - 1 - i; j++)
      if (v[j] > v[j+1]) { ushort t = v[j]; v[j] = v[j+1]; v[j+1] = t; }
  out[y * w + x] = v[n / 2];
}

// Faster right disparity construction by projecting each left pixel once.
// owner_x stores the "winner" left x for each (rx,y). We keep the largest x (foreground winner).
#pragma OPENCL EXTENSION cl_khr_global_int32_base_atomics : enable
__kernel void right_disp_from_left_scatter_owner_x(__global const ushort* left_disp, __global int* owner_x,
                                                   int w, int h, int subpixel_bits, int d_min, int d_max) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if ((unsigned)x >= (unsigned)w || (unsigned)y >= (unsigned)h) return;
  ushort d_raw = left_disp[y * w + x];
  if (d_raw == 0) return;
  int d_int = subpixel_bits > 0 ? ((int)d_raw >> subpixel_bits) : (int)d_raw;
  if (d_int < d_min || d_int > d_max) return;
  int rx = x - d_int;
  if ((unsigned)rx >= (unsigned)w) return;
  atomic_max((volatile __global int*)&owner_x[y * w + rx], x);
}

__kernel void right_disp_from_left_finalize(__global const ushort* left_disp, __global const int* owner_x,
                                            __global ushort* right_disp, int w, int h) {
  int rx = get_global_id(0);
  int y = get_global_id(1);
  if ((unsigned)rx >= (unsigned)w || (unsigned)y >= (unsigned)h) return;
  int x = owner_x[y * w + rx];
  right_disp[y * w + rx] = (x >= 0) ? left_disp[y * w + x] : (ushort)0;
}

__kernel void lr_check(__global ushort* left_disp, __global const ushort* right_disp, int w, int h, int thresh, int subpixel_bits) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w || y >= h) return;
  int d_left = (int)left_disp[y * w + x];
  int d_int = subpixel_bits > 0 ? (d_left >> subpixel_bits) : d_left;
  int x_right = x - d_int;
  if (x_right < 0 || x_right >= w) return;
  int d_right = (int)right_disp[y * w + x_right];
  if (abs(d_left - d_right) > thresh)
    left_disp[y * w + x] = 0;
}

__kernel void confidence_filter(read_only image2d_t left, read_only image2d_t right,
                                __global const ushort* disp_in, __global ushort* disp_out,
                                int subpixel_bits, float cost_threshold, int w, int h, int cost_mode,
                                __global const uint* desc_left, __global const uint* desc_right,
                                __global const float* winner_cost_in,
                                int use_stored_winner_cost, int epipolar_v_half) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w || y >= h) return;
  int d_raw = (int)disp_in[y * w + x];
  int d = (subpixel_bits > 0) ? (d_raw >> subpixel_bits) : d_raw;
  int rx = x - d;
  if (rx < 0) {
    disp_out[y * w + x] = 0;
    return;
  }
  float cost;
  if (use_stored_winner_cost && (cost_mode == 1 || cost_mode == 2))
    cost = winner_cost_in[y * w + x];
  else if (cost_mode == 1)
    cost = zncc_block_min_epipolar_v(left, right, x, y, rx, y, w, h, epipolar_v_half);
  else if (cost_mode == 2) {
    uint a = desc_left[y * w + x];
    uint b = desc_right[y * w + rx];
    cost = (float)hamming_bits(a, b);
  } else
    cost = sad_block(left, right, x, y, rx, y, w, h);
  if (cost > cost_threshold)
    disp_out[y * w + x] = 0;
  else
    disp_out[y * w + x] = (ushort)d_raw;
}

__kernel void winner_cost_to_confidence(__global const float* winner_cost_in,
                                        __global uchar* confidence_out,
                                        float cost_max,
                                        int w, int h) {
  int x = get_global_id(0);
  int y = get_global_id(1);
  if (x >= w || y >= h) return;
  float c = winner_cost_in[y * w + x];
  // Winner cost is (1 - ZNCC_rho) for the current implementation.
  // Typical range: [0, 2], where 0 means best match.
  float t = fmax(cost_max, 1e-6f);
  float norm = 1.0f - (c / t);
  norm = clamp(norm, 0.0f, 1.0f);
  confidence_out[y * w + x] = (uchar)rint(norm * 255.0f);
}
)CLC";
}

}
}
