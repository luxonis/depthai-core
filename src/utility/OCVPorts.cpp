/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of OpenCV Foundation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the OpenCV Foundation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "OCVPorts.hpp"

#include <stdexcept>

#include "depthai/common/Point2f.hpp"
#include "depthai/common/RotatedRect.hpp"

#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

using namespace dai;

template <typename T>
inline int sign(T val) {
    return (val > 0) - (val < 0);
}

enum { CALIPERS_MAXHEIGHT = 0, CALIPERS_MINAREARECT = 1, CALIPERS_MAXDIST = 2 };

template <typename T>
struct PointPort {
    T x;
    T y;

    PointPort() : x(0), y(0) {}
    PointPort(T x, T y) : x(x), y(y) {}
};

template <typename _Tp, typename _DotTp>
static int SklanskyPort(PointPort<_Tp>** array, int start, int end, int* stack, int nsign, int sign2) {
    int incr = end > start ? 1 : -1;
    // prepare first triangle
    int pprev = start, pcur = pprev + incr, pnext = pcur + incr;
    int stacksize = 3;

    if(start == end || (array[start]->x == array[end]->x && array[start]->y == array[end]->y)) {
        stack[0] = start;
        return 1;
    }

    stack[0] = pprev;
    stack[1] = pcur;
    stack[2] = pnext;

    end += incr;  // make end = afterend

    while(pnext != end) {
        // check the angle p1,p2,p3
        _Tp cury = array[pcur]->y;
        _Tp nexty = array[pnext]->y;
        _Tp by = nexty - cury;

        if(sign(by) != nsign) {
            _Tp ax = array[pcur]->x - array[pprev]->x;
            _Tp bx = array[pnext]->x - array[pcur]->x;
            _Tp ay = cury - array[pprev]->y;
            _DotTp convexity = (_DotTp)ay * (_DotTp)bx - (_DotTp)ax * (_DotTp)by;  // if >0 then convex angle

            if(sign(convexity) == sign2 && (ax != 0 || ay != 0)) {
                pprev = pcur;
                pcur = pnext;
                pnext += incr;
                stack[stacksize] = pnext;
                stacksize++;
            } else {
                if(pprev == start) {
                    pcur = pnext;
                    stack[1] = pcur;
                    pnext += incr;
                    stack[2] = pnext;
                } else {
                    stack[stacksize - 2] = pnext;
                    pcur = pprev;
                    pprev = stack[stacksize - 4];
                    stacksize--;
                }
            }
        } else {
            pnext += incr;
            stack[stacksize - 1] = pnext;
        }
    }

    return --stacksize;
}

void convexHullPort(std::vector<PointPort<float>>& points, std::vector<PointPort<float>>& _hull, bool clockwise) {
    static_assert(sizeof(PointPort<int>) == sizeof(PointPort<float>), "Both int and float PointPorts should have the same size.");
    int i, total = points.size(), nout = 0;
    int miny_ind = 0, maxy_ind = 0;

    if(total == 0) {
        return;
    }

    bool is_float = true;
    std::vector<PointPort<int>*> _pointer(total);
    std::vector<int> _stack(total + 2), _hullbuf(total);
    PointPort<int>** pointer = _pointer.data();
    PointPort<float>** pointerf = (PointPort<float>**)pointer;
    PointPort<int>* data0 = (PointPort<int>*)points.data();
    int* stack = _stack.data();
    int* hullbuf = _hullbuf.data();

    for(i = 0; i < total; i++) pointer[i] = &data0[i];

    // sort the point set by x-coordinate, find min and max y
    if(!is_float) {
        std::sort(pointer, pointer + total, [](const PointPort<int>* p1, const PointPort<int>* p2) {
            if(p1->x != p2->x) return p1->x < p2->x;
            if(p1->y != p2->y) return p1->y < p2->y;
            return p1 < p2;
        });
        for(i = 1; i < total; i++) {
            int y = pointer[i]->y;
            if(pointer[miny_ind]->y > y) miny_ind = i;
            if(pointer[maxy_ind]->y < y) maxy_ind = i;
        }
    } else {
        std::sort(pointerf, pointerf + total, [](const PointPort<float>* p1, const PointPort<float>* p2) {
            if(p1->x != p2->x) return p1->x < p2->x;
            if(p1->y != p2->y) return p1->y < p2->y;
            return p1 < p2;
        });
        for(i = 1; i < total; i++) {
            float y = pointerf[i]->y;
            if(pointerf[miny_ind]->y > y) miny_ind = i;
            if(pointerf[maxy_ind]->y < y) maxy_ind = i;
        }
    }

    if(pointer[0]->x == pointer[total - 1]->x && pointer[0]->y == pointer[total - 1]->y) {
        hullbuf[nout++] = 0;
    } else {
        // upper half
        int* tl_stack = stack;
        int tl_count =
            !is_float ? SklanskyPort<int, int64_t>(pointer, 0, maxy_ind, tl_stack, -1, 1) : SklanskyPort<float, double>(pointerf, 0, maxy_ind, tl_stack, -1, 1);
        int* tr_stack = stack + tl_count;
        int tr_count = !is_float ? SklanskyPort<int, int64_t>(pointer, total - 1, maxy_ind, tr_stack, -1, -1)
                                 : SklanskyPort<float, double>(pointerf, total - 1, maxy_ind, tr_stack, -1, -1);

        // gather upper part of convex hull to output
        if(!clockwise) {
            std::swap(tl_stack, tr_stack);
            std::swap(tl_count, tr_count);
        }

        for(i = 0; i < tl_count - 1; i++) hullbuf[nout++] = int(pointer[tl_stack[i]] - data0);
        for(i = tr_count - 1; i > 0; i--) hullbuf[nout++] = int(pointer[tr_stack[i]] - data0);
        int stop_idx = tr_count > 2 ? tr_stack[1] : tl_count > 2 ? tl_stack[tl_count - 2] : -1;

        // lower half
        int* bl_stack = stack;
        int bl_count =
            !is_float ? SklanskyPort<int, int64_t>(pointer, 0, miny_ind, bl_stack, 1, -1) : SklanskyPort<float, double>(pointerf, 0, miny_ind, bl_stack, 1, -1);
        int* br_stack = stack + bl_count;
        int br_count = !is_float ? SklanskyPort<int, int64_t>(pointer, total - 1, miny_ind, br_stack, 1, 1)
                                 : SklanskyPort<float, double>(pointerf, total - 1, miny_ind, br_stack, 1, 1);

        if(clockwise) {
            std::swap(bl_stack, br_stack);
            std::swap(bl_count, br_count);
        }

        if(stop_idx >= 0) {
            int check_idx = bl_count > 2 ? bl_stack[1] : bl_count + br_count > 2 ? br_stack[2 - bl_count] : -1;
            if(check_idx == stop_idx || (check_idx >= 0 && pointer[check_idx]->x == pointer[stop_idx]->x && pointer[check_idx]->y == pointer[stop_idx]->y)) {
                // if all the points lie on the same line, then
                // the bottom part of the convex hull is the mirrored top part
                // (except the extreme points).
                bl_count = std::min(bl_count, 2);
                br_count = std::min(br_count, 2);
            }
        }

        for(i = 0; i < bl_count - 1; i++) hullbuf[nout++] = int(pointer[bl_stack[i]] - data0);
        for(i = br_count - 1; i > 0; i--) hullbuf[nout++] = int(pointer[br_stack[i]] - data0);

        // try to make the convex hull indices form
        // an ascending or descending sequence by the cyclic
        // shift of the output sequence.
        if(nout >= 3) {
            int min_idx = 0, max_idx = 0, lt = 0;
            for(i = 1; i < nout; i++) {
                int idx = hullbuf[i];
                lt += hullbuf[i - 1] < idx;
                if(lt > 1 && lt <= i - 2) break;
                if(idx < hullbuf[min_idx]) min_idx = i;
                if(idx > hullbuf[max_idx]) max_idx = i;
            }
            int mmdist = std::abs(max_idx - min_idx);
            if((mmdist == 1 || mmdist == nout - 1) && (lt <= 1 || lt >= nout - 2)) {
                int ascending = (max_idx + 1) % nout == min_idx;
                int i0 = ascending ? min_idx : max_idx, j = i0;
                if(i0 > 0) {
                    for(i = 0; i < nout; i++) {
                        int curr_idx = stack[i] = hullbuf[j];
                        int next_j = j + 1 < nout ? j + 1 : 0;
                        int next_idx = hullbuf[next_j];
                        if(i < nout - 1 && (ascending != (curr_idx < next_idx))) break;
                        j = next_j;
                    }
                    if(i == nout) memcpy(hullbuf, stack, nout * sizeof(hullbuf[0]));
                }
            }
        }
    }

    _hull.clear();
    _hull.reserve(nout);
    for(i = 0; i < nout; i++) _hull.push_back(*(PointPort<float>*)&data0[hullbuf[i]]);
}

static void rotate90CCWPort(const PointPort<float>& in, PointPort<float>& out) {
    out.x = -in.y;
    out.y = in.x;
}

static void rotate90CWPort(const PointPort<float>& in, PointPort<float>& out) {
    out.x = in.y;
    out.y = -in.x;
}

static void rotate180Port(const PointPort<float>& in, PointPort<float>& out) {
    out.x = -in.x;
    out.y = -in.y;
}

static bool firstVecIsRightPort(const PointPort<float>& vec1, const PointPort<float>& vec2) {
    PointPort<float> tmp;
    rotate90CWPort(vec1, tmp);
    return tmp.x * vec2.x + tmp.y * vec2.y < 0;
}

static void rotatingCalipersPort(const PointPort<float>* points, int n, int mode, float* out) {
    float minarea = std::numeric_limits<float>::max();
    float max_dist = 0;
    char buffer[32] = {};
    int i, k;
    std::vector<float> abuf(n * 3);
    float* inv_vect_length = abuf.data();
    PointPort<float>* vect = (PointPort<float>*)(inv_vect_length + n);
    int left = 0, bottom = 0, right = 0, top = 0;
    int seq[4] = {-1, -1, -1, -1};
    PointPort<float> rot_vect[4];

    /* rotating calipers sides will always have coordinates
     (a,b) (-b,a) (-a,-b) (b, -a)
     */
    /* this is a first base vector (a,b) initialized by (1,0) */
    float orientation = 0;
    float base_a;
    float base_b = 0;

    float left_x, right_x, top_y, bottom_y;
    PointPort<float> pt0 = points[0];

    left_x = right_x = pt0.x;
    top_y = bottom_y = pt0.y;

    for(i = 0; i < n; i++) {
        double dx, dy;

        if(pt0.x < left_x) left_x = pt0.x, left = i;

        if(pt0.x > right_x) right_x = pt0.x, right = i;

        if(pt0.y > top_y) top_y = pt0.y, top = i;

        if(pt0.y < bottom_y) bottom_y = pt0.y, bottom = i;

        PointPort<float> pt = points[(i + 1) & (i + 1 < n ? -1 : 0)];

        dx = (double)(pt.x - pt0.x);
        dy = (double)(pt.y - pt0.y);

        vect[i].x = (float)dx;
        vect[i].y = (float)dy;
        inv_vect_length[i] = (float)(1. / std::sqrt(dx * dx + dy * dy));

        pt0 = pt;
    }

    // find convex hull orientation
    {
        double ax = (double)vect[n - 1].x;
        double ay = (double)vect[n - 1].y;

        for(i = 0; i < n; i++) {
            double bx = (double)vect[i].x;
            double by = (double)vect[i].y;

            double convexity = ax * by - ay * bx;

            if(convexity != 0) {
                orientation = (convexity > 0) ? 1.f : (-1.f);
                break;
            }
            ax = bx;
            ay = by;
        }
        if(orientation == 0) throw std::runtime_error("All points of the convex hull are collinear");
    }
    base_a = orientation;

    /*****************************************************************************************/
    /*                         init calipers position */
    seq[0] = bottom;
    seq[1] = right;
    seq[2] = top;
    seq[3] = left;
    /*****************************************************************************************/
    /*                         Main loop - evaluate angles and rotate calipers */

    /* all of edges will be checked while rotating calipers by 90 degrees */
    for(k = 0; k < n; k++) {
        /* number of calipers edges, that has minimal angle with edge */
        int main_element = 0;

        /* choose minimum angle between calipers side and polygon edge by dot
         * product sign */
        rot_vect[0] = vect[seq[0]];
        rotate90CWPort(vect[seq[1]], rot_vect[1]);
        rotate180Port(vect[seq[2]], rot_vect[2]);
        rotate90CCWPort(vect[seq[3]], rot_vect[3]);
        for(i = 1; i < 4; i++) {
            if(firstVecIsRightPort(rot_vect[i], rot_vect[main_element])) main_element = i;
        }

        /*rotate calipers*/
        {
            // get next base
            int pindex = seq[main_element];
            float lead_x = vect[pindex].x * inv_vect_length[pindex];
            float lead_y = vect[pindex].y * inv_vect_length[pindex];
            switch(main_element) {
                case 0:
                    base_a = lead_x;
                    base_b = lead_y;
                    break;
                case 1:
                    base_a = lead_y;
                    base_b = -lead_x;
                    break;
                case 2:
                    base_a = -lead_x;
                    base_b = -lead_y;
                    break;
                case 3:
                    base_a = -lead_y;
                    base_b = lead_x;
                    break;
                default:
                    throw std::runtime_error("main_element should be 0, 1, 2 or 3");
            }
        }
        /* change base point of main edge */
        seq[main_element] += 1;
        seq[main_element] = (seq[main_element] == n) ? 0 : seq[main_element];

        switch(mode) {
            case CALIPERS_MAXHEIGHT: {
                /* now main element lies on edge aligned to calipers side */

                /* find opposite element i.e. transform  */
                /* 0->2, 1->3, 2->0, 3->1                */
                int opposite_el = main_element ^ 2;

                float dx = points[seq[opposite_el]].x - points[seq[main_element]].x;
                float dy = points[seq[opposite_el]].y - points[seq[main_element]].y;
                float dist;

                if(main_element & 1)
                    dist = (float)fabsf(dx * base_a + dy * base_b);
                else
                    dist = (float)fabsf(dx * (-base_b) + dy * base_a);

                if(dist > max_dist) max_dist = dist;
            } break;
            case CALIPERS_MINAREARECT:
                /* find area of rectangle */
                {
                    float height;
                    float area;

                    /* find vector left-right */
                    float dx = points[seq[1]].x - points[seq[3]].x;
                    float dy = points[seq[1]].y - points[seq[3]].y;

                    /* dotproduct */
                    float width = dx * base_a + dy * base_b;

                    /* find vector left-right */
                    dx = points[seq[2]].x - points[seq[0]].x;
                    dy = points[seq[2]].y - points[seq[0]].y;

                    /* dotproduct */
                    height = -dx * base_b + dy * base_a;

                    area = width * height;
                    if(area <= minarea) {
                        float* buf = (float*)buffer;

                        minarea = area;
                        /* leftist point */
                        ((int*)buf)[0] = seq[3];
                        buf[1] = base_a;
                        buf[2] = width;
                        buf[3] = base_b;
                        buf[4] = height;
                        /* bottom point */
                        ((int*)buf)[5] = seq[0];
                        buf[6] = area;
                    }
                }
                break;
        } /*switch */
    } /* for */

    switch(mode) {
        case CALIPERS_MINAREARECT: {
            float* buf = (float*)buffer;

            float A1 = buf[1];
            float B1 = buf[3];

            float A2 = -buf[3];
            float B2 = buf[1];

            float C1 = A1 * points[((int*)buf)[0]].x + points[((int*)buf)[0]].y * B1;
            float C2 = A2 * points[((int*)buf)[5]].x + points[((int*)buf)[5]].y * B2;

            float idet = 1.f / (A1 * B2 - A2 * B1);

            float px = (C1 * B2 - C2 * B1) * idet;
            float py = (A1 * C2 - A2 * C1) * idet;

            out[0] = px;
            out[1] = py;

            out[2] = A1 * buf[2];
            out[3] = B1 * buf[2];

            out[4] = A2 * buf[4];
            out[5] = B2 * buf[4];
        } break;
        case CALIPERS_MAXHEIGHT: {
            out[0] = max_dist;
        } break;
    }
}

RotatedRect minAreaRectPort(std::vector<PointPort<float>> _points) {
    std::vector<PointPort<float>> hull;
    PointPort<float> out[3];
    RotatedRect box;

    convexHullPort(_points, hull, false);

    int n = hull.size();
    const PointPort<float>* hpoints = hull.data();

    if(n > 2) {
        rotatingCalipersPort(hpoints, n, CALIPERS_MINAREARECT, (float*)out);
        box.center.x = out[0].x + (out[1].x + out[2].x) * 0.5f;
        box.center.y = out[0].y + (out[1].y + out[2].y) * 0.5f;
        box.size.width = (float)std::sqrt((double)out[1].x * (double)out[1].x + (double)out[1].y * (double)out[1].y);
        box.size.height = (float)std::sqrt((double)out[2].x * (double)out[2].x + (double)out[2].y * (double)out[2].y);
        box.angle = (float)atan2((double)out[1].y, (double)out[1].x);
    } else if(n == 2) {
        box.center.x = (hpoints[0].x + hpoints[1].x) * 0.5f;
        box.center.y = (hpoints[0].y + hpoints[1].y) * 0.5f;
        double dx = (double)hpoints[1].x - (double)hpoints[0].x;
        double dy = (double)hpoints[1].y - (double)hpoints[0].y;
        box.size.width = (float)std::sqrt(dx * dx + dy * dy);
        box.size.height = 0;
        box.angle = (float)atan2(dy, dx);
    } else {
        if(n == 1) {
            box.center.x = hpoints[0].x;
            box.center.y = hpoints[0].y;
        }
    }

    box.angle = (float)(box.angle * 180 / (float)M_PI);
    return box;
}

dai::RotatedRect dai::utility::getOuterRotatedRect(const std::vector<std::array<float, 2>>& points) {
    std::vector<PointPort<float>> daiPoints;
    daiPoints.reserve(points.size());
    for(const auto& p : points) {
        daiPoints.emplace_back(p[0], p[1]);
    }
    return minAreaRectPort(daiPoints);
}
