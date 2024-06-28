#include "depthai/pipeline/node/host/ImageManipHost.hpp"

#include <cmath>

#include "pipeline/datatype/ImgFrame.hpp"

namespace dai {
namespace node {

void ImageManipHost::run() {
    impl::ImageManipOperations manip(logger);
    auto base = initialConfig.base;

    bool initialized = false;

    while(isRunning()) {
        auto frame = inputImage.get<ImgFrame>();
        if(!initialized) {
            uint32_t numPlanes = frame->getType() == ImgFrame::Type::GRAY8 || frame->getType() == ImgFrame::Type::RAW8 ? 1 : 3;
            manip.build(base, frame->getWidth(), frame->getHeight(), numPlanes);
            initialized = true;
        }
        const auto [w, h, c] = manip.getOutputSize();
        ImgFrame dst(*frame);
        dst.setData(std::vector<std::uint8_t>(w * h * c));
        dst.fb.height = h;
        dst.fb.width = w;

        switch(dst.getType()) {
            case dai::ImgFrame::Type::RGB888p:
            case dai::ImgFrame::Type::BGR888p:
                dst.fb.p1Offset = 0;
                dst.fb.p2Offset = w * h;
                dst.fb.p3Offset = 2 * w * h;
                break;
            case dai::ImgFrame::Type::NV12:
                dst.fb.p1Offset = 0;
                dst.fb.p2Offset = w * h;
                break;
            case dai::ImgFrame::Type::YUV420p:
                dst.fb.p1Offset = 0;
                dst.fb.p2Offset = w * h;
                dst.fb.p3Offset = w * h / 4;
                break;
            default:
                break;
        }

        manip.apply(frame, dst.getData());
        out.send(std::make_shared<ImgFrame>(dst));
    }
}
}  // namespace node

namespace impl {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    #define USE_OPENCV
    #include <opencv2/opencv.hpp>
#endif

// helper type for the visitor #4
template <class... Ts>
struct overloaded : Ts... {
    using Ts::operator()...;
};
// explicit deduction guide (not needed as of C++20)
template <class... Ts>
overloaded(Ts...) -> overloaded<Ts...>;

bool float_eq(float a, float b) {
    return fabs(a - b) <= 1e-6;
}

bool isSingleChannelu8(const std::shared_ptr<dai::ImgFrame> img) {
    return img->getType() == dai::ImgFrame::Type::GRAY8 || img->getType() == dai::ImgFrame::Type::RAW8;
}

template <typename T>
std::string getOpStr(const T& op) {
    return op.toStr();
}

std::string getConfigString(const dai::ImageManipOpsBase& ops) {
    std::stringstream configSS;
    const auto operations = ops.getOperations();
    for(auto i = 0U; i < operations.size(); ++i) {
        configSS << std::visit([](auto&& op) { return getOpStr(op); }, operations[i].op);
        if(i != operations.size() - 1) configSS << " ";
    }
    return configSS.str();
}

std::array<std::array<float, 3>, 3> matmul(std::array<std::array<float, 3>, 3> A, std::array<std::array<float, 3>, 3> B) {
    return {{{A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0],
              A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1],
              A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2]},
             {A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0],
              A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1],
              A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2]},
             {A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0],
              A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1],
              A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2]}}};
}

std::array<float, 2> matvecmul(std::array<std::array<float, 3>, 3> M, std::array<float, 2> vec) {
    auto x = M[0][0] * vec[0] + M[0][1] * vec[1] + M[0][2];
    auto y = M[1][0] * vec[0] + M[1][1] * vec[1] + M[1][2];
    auto z = M[2][0] * vec[0] + M[2][1] * vec[1] + M[2][2];
    return {x / z, y / z};
}

std::tuple<float, float, float, float> getOuterRect(const std::vector<std::array<float, 2>> points) {
    float minx = points[0][0];
    float maxx = points[0][0];
    float miny = points[0][1];
    float maxy = points[0][1];

    for(auto i = 0U; i < points.size(); ++i) {
        minx = std::min(points[i][0], minx);
        maxx = std::max(points[i][0], maxx);
        miny = std::min(points[i][1], miny);
        maxy = std::max(points[i][1], maxy);
    }
    return {minx, maxx, miny, maxy};
}

std::vector<std::array<float, 2>> getHull(const std::vector<std::array<float, 2>> points) {
    std::vector<std::array<float, 2>> remaining(points.rbegin(), points.rend() - 1);
    std::vector<std::array<float, 2>> hull{points.front()};
    hull.reserve(points.size());
    while(remaining.size() > 0) {
        auto pt = remaining.back();
        remaining.pop_back();
        while(hull.size() >= 2) {
            auto last1 = hull.size() - 1;
            auto last2 = hull.size() - 2;
            std::array<float, 2> v1 = {hull[last1][0] - hull[last2][0], hull[last1][1] - hull[last2][1]};
            std::array<float, 2> v2 = {pt[0] - hull[last1][0], pt[1] - hull[last1][1]};
            std::array<float, 2> v3 = {hull[0][0] - pt[0], hull[0][1] - pt[1]};
            auto cross1 = v1[0] * v2[1] - v1[1] * v2[0];
            auto cross2 = v2[0] * v3[1] - v2[1] * v3[0];
            if(cross1 < 0 || cross2 < 0) {
                remaining.push_back(hull.back());
                hull.pop_back();
            } else if(cross1 == 0 || cross2 == 0) {
                throw std::runtime_error("Colinear points");
            } else {
                break;
            }
        }
        hull.push_back(pt);
    }
    return hull;
}

std::array<std::array<float, 2>, 2> getInverse2x2(const std::array<std::array<float, 2>, 2> mat) {
    auto det = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    if(det == 0) {
        throw std::runtime_error("Determinant is zero");
    }
    return {{{mat[1][1] / det, -mat[0][1] / det}, {-mat[1][0] / det, mat[0][0] / det}}};
}

std::array<std::array<float, 2>, 4> getOuterRotatedRect(const std::vector<std::array<float, 2>>& points) {
    auto hull = getHull(points);
    float minArea = std::numeric_limits<float>::max();
    std::array<std::array<float, 2>, 4> minAreaPoints;

    for(size_t i = 1; i < hull.size(); ++i) {
        std::array<float, 2> vec = {hull[i][0] - hull[i - 1][0], hull[i][1] - hull[i - 1][1]};
        std::array<float, 2> vecOrth = {-vec[1], vec[0]};
        std::array<std::array<float, 2>, 2> mat = {{{vec[0], vecOrth[0]}, {vec[1], vecOrth[1]}}};
        std::array<std::array<float, 2>, 2> matInv = getInverse2x2(mat);

        std::vector<std::array<float, 2>> rotatedHull;
        for(const auto& pt : hull) {
            float newX = matInv[0][0] * pt[0] + matInv[0][1] * pt[1];
            float newY = matInv[1][0] * pt[0] + matInv[1][1] * pt[1];
            rotatedHull.push_back({newX, newY});
        }

        const auto [minx, maxx, miny, maxy] = getOuterRect(rotatedHull);
        float area = (maxx - minx) * (maxy - miny);

        if(area < minArea) {
            minArea = area;
            std::array<std::array<float, 2>, 4> rectPoints = {{{minx, miny}, {maxx, miny}, {maxx, maxy}, {minx, maxy}}};
            for(auto i = 0U; i < rectPoints.size(); ++i) {
                auto& pt = rectPoints[i];
                float origX = mat[0][0] * pt[0] + mat[0][1] * pt[1];
                float origY = mat[1][0] * pt[0] + mat[1][1] * pt[1];
                minAreaPoints[i] = {origX, origY};
            }
        }
    }

    return minAreaPoints;
}

std::array<std::array<float, 3>, 3> getResizeMat(Resize o, float width, float height, uint32_t outputWidth, uint32_t outputHeight) {
    if(o.mode == Resize::VALUE) {
        if(o.width > 0 && o.height <= 0) {
            o.height = o.normalized ? o.width : height * (o.width / width);
        } else if(o.width <= 0 && o.height > 0) {
            o.width = o.normalized ? o.height : width * (o.height / height);
        }
    } else if(o.mode == Resize::FIT || o.mode == Resize::FILL) {
        if(outputWidth > 0 && outputHeight > 0) {
            float ratio = width / height;
            if((float)outputWidth / (float)outputHeight > ratio) {
                if(o.mode == Resize::FIT) {
                    o.width = outputHeight * ratio;
                    o.height = outputHeight;
                } else {
                    o.width = outputWidth;
                    o.height = outputWidth / ratio;
                }
            } else {
                if(o.mode == Resize::FIT) {
                    o.width = outputWidth;
                    o.height = outputWidth / ratio;
                } else {
                    o.width = outputHeight * ratio;
                    o.height = outputHeight;
                }
            }
        } else {
            throw std::runtime_error("Neither output size nor resize dimensions are set");
        }
    }
    if(!o.normalized) {
        o.width /= width;
        o.height /= height;
    }
    return {{{o.width, 0, 0}, {0, o.height, 0}, {0, 0, 1}}};
}

std::tuple<std::array<std::array<float, 3>, 3>, std::array<std::array<float, 2>, 4>> getTransform(
    const std::vector<ManipOp>& ops, uint32_t inputWidth, uint32_t inputHeight, uint32_t outputWidth, uint32_t outputHeight) {
    std::array<std::array<float, 3>, 3> transform{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    std::array<std::array<float, 2>, 4> imageCorners{{{0, 0}, {(float)inputWidth, 0}, {(float)inputWidth, (float)inputHeight}, {0, (float)inputHeight}}};
    for(const auto& op : ops) {
        std::array<std::array<float, 3>, 3> mat = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};

        float centerX = (imageCorners[0][0] + imageCorners[1][0] + imageCorners[2][0] + imageCorners[3][0]) / 4;
        float centerY = (imageCorners[0][1] + imageCorners[1][1] + imageCorners[2][1] + imageCorners[3][1]) / 4;

        const auto [_minx, _maxx, _miny, _maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
        float minx = _minx;
        float maxx = _maxx;
        float miny = _miny;
        float maxy = _maxy;
        float width = maxx - minx;
        float height = maxy - miny;

        std::visit(
            overloaded{[](auto _) {},
                       [&](Translate o) {
                           if(o.normalized) {
                               o.offsetX *= width;
                               o.offsetY *= height;
                           }
                           mat = {{{1, 0, o.offsetX}, {0, 1, o.offsetY}, {0, 0, 1}}};
                       },
                       [&](Rotate o) {
                           float cos = std::cos(o.angle);
                           float sin = std::sin(o.angle);
                           float moveX = centerX;
                           float moveY = centerY;
                           if(o.center) {
                               mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                           }
                           mat = matmul({{{cos, -sin, 0}, {sin, cos, 0}, {0, 0, 1}}}, mat);
                           if(o.center) {
                               mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                           }
                       },
                       [&](Resize o) { mat = getResizeMat(o, width, height, outputWidth, outputHeight); },
                       [&](Flip o) {
                           float moveX = centerX;
                           float moveY = centerY;
                           switch(o.direction) {
                               case Flip::HORIZONTAL: {
                                   if(o.center) {
                                       mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                                   }
                                   mat = matmul({{{-1, 0, 0}, {0, 1, 0}, {0, 0, 1}}}, mat);
                                   if(o.center) {
                                       mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                                   }
                                   break;
                               }
                               case Flip::VERTICAL: {
                                   if(o.center) {
                                       mat = {{{1, 0, -moveX}, {0, 1, -moveY}, {0, 0, 1}}};
                                   }
                                   mat = matmul({{{1, 0, 0}, {0, -1, 0}, {0, 0, 1}}}, mat);
                                   if(o.center) {
                                       mat = matmul({{{1, 0, moveX}, {0, 1, moveY}, {0, 0, 1}}}, mat);
                                   }
                                   break;
                               }
                           }
                       },
                       [&](FourPoints o) {
                           if(o.normalized) {
                               for(auto i = 0; i < 4; ++i) {
                                   o.src[i].x *= width;
                                   o.src[i].y *= height;
                                   o.dst[i].x *= width;
                                   o.dst[i].y *= height;
                               }
                               std::array<float, 9> coeff = {};
                               std::array<float, 8> srcData = {o.src[0].x, o.src[0].y, o.src[1].x, o.src[1].y, o.src[2].x, o.src[2].y, o.src[3].x, o.src[3].y};
                               std::array<float, 8> dstData = {o.dst[0].x, o.dst[0].y, o.dst[1].x, o.dst[1].y, o.dst[2].x, o.dst[2].y, o.dst[3].x, o.dst[3].y};
#ifdef USE_OPENCV
                               cv::Mat srcPoints(4, 2, CV_32F, srcData.data());
                               cv::Mat dstPoints(4, 2, CV_32F, dstData.data());
#else
    #ifdef DEPTHAI_HAVE_FASTCV_SUPPORT
                               fcvGetPerspectiveTransformf32(srcData.data(), dstData.data(), coeff.data());
    #endif
#endif
                               mat = {{{coeff[0], coeff[1], coeff[2]}, {coeff[3], coeff[4], coeff[5]}, {coeff[6], coeff[7], coeff[8]}}};
                           }
                       },
                       [&](Affine o) { mat = {{{o.matrix[0], o.matrix[1], 0}, {o.matrix[2], o.matrix[3], 0}, {0, 0, 1}}}; },
                       [&](Perspective o) {
                           mat = {{{o.matrix[0], o.matrix[1], o.matrix[2]}, {o.matrix[3], o.matrix[4], o.matrix[5]}, {o.matrix[6], o.matrix[7], o.matrix[8]}}};
                       },
                       [&](Crop o) {
                           if(o.normalized) {
                               o.width *= width;
                               o.height *= height;
                           }
                           if(o.width > 0 && o.height <= 0)
                               o.height = roundf(height * ((float)o.width / width));
                           else if(o.height > 0 && o.width <= 0)
                               o.width = roundf(width * (o.height / height));
                           else if(o.height <= 0 && o.width <= 0) {
                               o.width = roundf(maxx);
                               o.height = roundf(maxy);
                           }

                           outputWidth = o.width;
                           outputHeight = o.height;

                           if(o.center) {
                               std::array<std::array<float, 3>, 3> _mat = {
                                   {{1, 0, -minx + (outputWidth - (maxx - minx)) / 2}, {0, 1, -miny + (outputHeight - (maxy - miny)) / 2}, {0, 0, 1}}};
                               transform = matmul(_mat, transform);
                           }

                           imageCorners = {{{0, 0}, {(float)outputWidth, 0}, {(float)outputWidth, (float)outputHeight}, {0, (float)outputHeight}}};
                       }},
            op.op);
        /*printf("Mat: %f %f %f %f %f %f %f %f %f\n", mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1], mat[1][2], mat[2][0], mat[2][1], mat[2][2]);*/
        imageCorners = getOuterRotatedRect(
            {matvecmul(mat, imageCorners[0]), matvecmul(mat, imageCorners[1]), matvecmul(mat, imageCorners[2]), matvecmul(mat, imageCorners[3])});
        transform = matmul(mat, transform);
    }
    /*printf("Transform: %f %f %f %f %f %f %f %f %f\n", transform[0][0], transform[0][1], transform[0][2], transform[1][0], transform[1][1], transform[1][2],
     * transform[2][0], transform[2][1], transform[2][2]);*/
    return {transform, imageCorners};
}

ImageManipOperations& ImageManipOperations::build(const ImageManipOpsBase& newBase, uint32_t inputWidth, uint32_t inputHeight, uint8_t bpp) {
    assert(inputWidth > 0 && inputHeight > 0);
    matrix = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    base = newBase;

    auto operations = base.getOperations();

    auto [transformMat, imageCorners] = getTransform(operations, inputWidth, inputHeight, base.outputWidth, base.outputHeight);
    matrix = transformMat;

    if(logger) {
        logger->trace("|{} {}|{} {}|", imageCorners[0][0], imageCorners[0][1], imageCorners[1][0], imageCorners[1][1]);
        logger->trace("-------------");
        logger->trace("|{} {}|{} {}|", imageCorners[3][0], imageCorners[3][1], imageCorners[2][0], imageCorners[2][1]);
    }
    if(logger) {
        logger->trace("|{} {} {}|", matrix[0][0], matrix[0][1], matrix[0][2]);
        logger->trace("-------------");
        logger->trace("|{} {} {}|", matrix[1][0], matrix[1][1], matrix[1][2]);
        logger->trace("-------------");
        logger->trace("|{} {} {}|", matrix[2][0], matrix[2][1], matrix[2][2]);
    }
    {
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
        if(base.outputWidth == 0) base.outputWidth = maxx;
        if(base.outputHeight == 0) base.outputHeight = maxy;
    }

    if(base.resizeMode != ImageManipOpsBase::ResizeMode::NONE) {
        Resize res;
        switch(base.resizeMode) {
            case ImageManipOpsBase::ResizeMode::NONE:
                break;
            case ImageManipOpsBase::ResizeMode::STRETCH:
                res = Resize(base.outputWidth, base.outputHeight);
                break;
            case ImageManipOpsBase::ResizeMode::LETTERBOX:
                res = Resize::fit();
                break;
            case ImageManipOpsBase::ResizeMode::CENTER_CROP:
                res = Resize::fill();
                break;
        }
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
        auto mat = getResizeMat(res, maxx - minx, maxy - miny, base.outputWidth, base.outputHeight);
        imageCorners = {
            {{matvecmul(mat, imageCorners[0])}, {matvecmul(mat, imageCorners[1])}, {matvecmul(mat, imageCorners[2])}, {matvecmul(mat, imageCorners[2])}}};
        matrix = matmul(mat, matrix);
    }

    if(base.center) {
        float width = base.outputWidth;
        float height = base.outputHeight;
        auto [minx, maxx, miny, maxy] = getOuterRect(std::vector(imageCorners.begin(), imageCorners.end()));
        std::array<std::array<float, 3>, 3> mat = {{{1, 0, -minx + (width - (maxx - minx)) / 2}, {0, 1, -miny + (height - (maxy - miny)) / 2}, {0, 0, 1}}};
        imageCorners = {
            {{matvecmul(mat, imageCorners[0])}, {matvecmul(mat, imageCorners[1])}, {matvecmul(mat, imageCorners[2])}, {matvecmul(mat, imageCorners[2])}}};
        matrix = matmul(mat, matrix);
    }

    size_t newAuxSize = base.outputWidth * base.outputHeight * bpp;
    if(!trAuxFrame || trAuxFrame->getData().size() < newAuxSize) trAuxFrame = std::make_shared<Buffer>(newAuxSize);
    numPlanes = bpp;
    if(numPlanes == 1) numPlanes = base.colormap == Colormap::NONE ? 1 : 3;

    return *this;
}

bool ImageManipOperations::apply(const std::shared_ptr<ImgFrame> src, span<uint8_t> dst) {
#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
    uint8_t* transformDst = base.colormap == Colormap::NONE ? dst.data() : trAuxFrame->getData().data();
    if(float_eq(matrix[2][0], 0) && float_eq(matrix[2][1], 0) && float_eq(matrix[2][2], 1)) {
        // Do affine
        float affine[6] = {matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[1][1], matrix[1][2]};
    #ifdef USE_OPENCV
        cv::Mat cvSrc(src->getHeight(), src->getWidth(), isSingleChannelu8(src) ? CV_8UC1 : CV_8UC3, src->getData().data());
        cv::Mat cvDst(base.outputHeight, base.outputWidth, numPlanes == 1 ? CV_8UC1 : CV_8UC3, transformDst);
        assert(dst.size() >= base.outputHeight * base.outputWidth * numPlanes);
        cv::Mat cvAffine(2, 3, CV_32F, affine);

        if(logger)
            logger->trace("cvSrc {}x{} ({}), cvDst {}x{} ({}), cvAffine {}x{}",
                          cvSrc.rows,
                          cvSrc.cols,
                          src->getData().size(),
                          cvDst.rows,
                          cvDst.cols,
                          dst.size(),
                          cvAffine.rows,
                          cvAffine.cols);
        cv::warpAffine(cvSrc, cvDst, cvAffine, cv::Size(base.outputWidth, base.outputHeight));

    #else
        #ifdef DEPTHAI_HAVE_FASTCV_SUPPORT
        if(isSingleChannelu8(src)) {
            fcvTransformAffineClippedu8_v3(src->getData().data(),
                                           src->getWidth(),
                                           src->getHeight(),
                                           src->getWidth(),
                                           affine,
                                           transformDst,
                                           base.outputWidth,
                                           base.outputHeight,
                                           base.outputWidth,
                                           nullptr,
                                           FASTCV_INTERPOLATION_TYPE_BILINEAR,
                                           FASTCV_BORDER_CONSTANT,
                                           0);
        } else {
            fcv3ChannelTransformAffineClippedBCu8(src->getData().data(),
                                                  src->getWidth(),
                                                  src->getHeight(),
                                                  src->getStride(),
                                                  affine,
                                                  transformDst,
                                                  base.outputWidth,
                                                  base.outputHeight,
                                                  base.outputWidth * 3,
                                                  nullptr);
        }
        #endif
    #endif
    } else {
        // Do perspective
        assert(src->getWidth() % 8 == 0);
        assert(src->getHeight() % 8 == 0);
        assert(base.outputWidth % 8 == 0);
        assert(base.outputHeight % 8 == 0);
        float projection[9] = {matrix[0][0], matrix[0][1], matrix[0][2], matrix[1][0], matrix[1][1], matrix[1][2], matrix[2][0], matrix[2][1], matrix[2][2]};
    #ifdef USE_OPENCV
        cv::Mat cvSrc(src->getHeight(), src->getWidth(), isSingleChannelu8(src) ? CV_8UC1 : CV_8UC3, src->getData().data());
        cv::Mat cvDst(base.outputHeight, base.outputWidth, numPlanes == 3 ? CV_8UC3 : CV_8UC1, dst.data());
        cv::Mat cvProjection(3, 3, CV_32F, projection);
        cv::warpPerspective(cvSrc, cvDst, cvProjection, cv::Size(base.outputWidth, base.outputHeight));
    #else
        #ifdef DEPTHAI_HAVE_FASTCV_SUPPORT
        fcvStatus status = fcvStatus::FASTCV_SUCCESS;
        if(isSingleChannelu8(src))
            status = fcvWarpPerspectiveu8_v4(src->getData().data(),
                                             src->getWidth(),
                                             src->getHeight(),
                                             src->getWidth(),
                                             transformDst,
                                             base.outputWidth,
                                             base.outputHeight,
                                             base.outputWidth,
                                             projection,
                                             FASTCV_INTERPOLATION_TYPE_BILINEAR,
                                             FASTCV_BORDER_CONSTANT,
                                             0);
        else
            fcv3ChannelWarpPerspectiveu8_v2(src->getData().data(),
                                            src->getWidth(),
                                            src->getHeight(),
                                            src->getStride(),
                                            transformDst,
                                            base.outputWidth,
                                            base.outputHeight,
                                            base.outputWidth * 3,
                                            projection);
        if(status != fcvStatus::FASTCV_SUCCESS) {
            if(logger) logger->error("FastCV operation failed with error code {}", status);
            return false;
        }
        #endif
    #endif
    }
    if(isSingleChannelu8(src)) {
        if(base.colormap != Colormap::NONE) {
            cv::Mat gray(base.outputWidth, base.outputHeight, CV_8UC1, trAuxFrame->getData().data());
            cv::Mat color(base.outputWidth, base.outputHeight, CV_8UC3, dst.data());
            cv::applyColorMap(gray, color, cv::COLORMAP_JET);  // TODO(asahtik) set colormap from `Colormap`
        }
    }
    return true;  // TODO(asahtik): Handle failed transformation
#else
    return false;
#endif
}

std::tuple<uint32_t, uint32_t, uint8_t> ImageManipOperations::getOutputSize() const {
    return {base.outputWidth, base.outputHeight, numPlanes};
}
}  // namespace impl
}  // namespace dai
