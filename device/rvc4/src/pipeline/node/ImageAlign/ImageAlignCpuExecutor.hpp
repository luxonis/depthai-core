#pragma once

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/task_arena.h>

#include <algorithm>
#include <opencv2/core.hpp>

namespace dai {
namespace gate {

// Bound a node's fused depth-alignment CPU passes to two threads (including the caller)
// without changing process-wide OpenCV/TBB settings. Shared by ImageAlign and Align.
class ImageAlignCpuExecutor {
    tbb::task_arena arena{2};

   public:
    template <typename Body>
    void operator()(const cv::Range& range, const Body& body, double stripes) {
        const int grain = std::max(1, cvRound(range.size() / std::max(1.0, stripes)));
        arena.execute([&] {
            tbb::parallel_for(tbb::blocked_range<int>(range.start, range.end, grain),
                              [&](const tbb::blocked_range<int>& rows) { body(cv::Range(rows.begin(), rows.end())); });
        });
    }
};

}  // namespace gate
}  // namespace dai
