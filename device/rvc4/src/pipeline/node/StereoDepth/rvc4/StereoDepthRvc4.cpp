#include <assert.h>
#include <fcntl.h>  // for open
#include <unistd.h>

#include <array>
#include <cmath>
#include <iostream>
#include <opencv2/calib3d.hpp>

#include "../StereoDepth.hpp"
#include "depthai/common/Extrinsics.hpp"
#include "depthai/common/ImgTransformations.hpp"
#include "depthai/pipeline/Assets.hpp"
#include "depthai/utility/matrixOps.hpp"
#include "pipeline/PipelineBuilder.hpp"
// std

#include <chrono>
#include <memory>
#include <vector>

#include "../BrightnessFilter/BrightnessFilter.hpp"
#include "../DecimationFilter/DecimationFilter.hpp"
#include "../MedianFilter/MedianFilter.hpp"
#include "../SpatialFilter/SpatialFilter.hpp"
#include "../SpeckleFilter/SpeckleFilter.hpp"
#include "../TemporalFilter/TemporalFilter.hpp"
#include "../ThresholdFilter/ThresholdFilter.hpp"
#include "depthai/utility/LockingQueue.hpp"
#include "hexagon/StereoDepthDsp/shared/StereoDepthDspConfig.h"
#include "hexagon/utilities/utilities.hpp"
#include "pipeline/Pool.hpp"
#include "spdlog/spdlog.h"
#include "utilities/AcceleratorMemory.hpp"
#include "vendor/DeviceError.hpp"
#include "vendor/dai_device_dsp.h"

namespace {

constexpr int CACHE_LINE_SIZE = 64;
constexpr long ALIGN_UP(long S, long a) {
    return ((((S) + (a)-1)) & ~((a)-1));
};

static constexpr const int STEREO_HW_CFG_OUT_RM_THR_MIN = 0;
static constexpr const int STEREO_HW_CFG_OUT_RM_THR_MAX = 49;
static constexpr const int STEREO_HW_CFG_OUT_CS_THR_MIN = 32;
static constexpr const int STEREO_HW_CFG_OUT_CS_THR_MAX = 255;
static constexpr const int STEREO_HW_CFG_OUT_DIFF_THR_MIN = 1;
static constexpr float CAM_A_PROXIMITY_SWITCH_RATIO = 0.8f;

template <typename T>
std::vector<T> flatten(const std::vector<std::vector<T>>& orig) {
    std::vector<T> ret;
    for(const auto& v : orig) ret.insert(ret.end(), v.begin(), v.end());
    return ret;
}

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<std::vector<float>>& orig) {
    if((int)orig.size() != rows) {
        throw std::runtime_error(fmt::format("Invalid matrix row count: expected {}, got {}", rows, orig.size()));
    }
    for(const auto& row : orig) {
        if((int)row.size() != cols) {
            throw std::runtime_error(fmt::format("Invalid matrix column count: expected {}, got {}", cols, row.size()));
        }
    }
    std::vector<float> flat = flatten(orig);
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, flat.data(), flat.size() * sizeof(flat[0]));
    return cvMat;
}

cv::Mat vecToCvMat(int rows, int cols, int type, const std::vector<float>& orig) {
    if((int)orig.size() != rows * cols) {
        throw std::runtime_error(fmt::format("Invalid matrix element count: expected {}, got {}", rows * cols, orig.size()));
    }
    cv::Mat cvMat = cv::Mat(rows, cols, type);
    memcpy(cvMat.data, orig.data(), orig.size() * sizeof(orig[0]));
    return cvMat;
}

std::array<std::array<float, 3>, 3> cvMatToArray3x3(const cv::Mat& mat) {
    if(mat.rows != 3 || mat.cols != 3) {
        throw std::runtime_error("Expected 3x3 matrix");
    }
    cv::Mat mat32f;
    if(mat.type() == CV_32FC1) {
        mat32f = mat;
    } else {
        mat.convertTo(mat32f, CV_32FC1);
    }

    std::array<std::array<float, 3>, 3> out{};
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            out[i][j] = mat32f.at<float>(i, j);
        }
    }
    return out;
}

struct MeshPoint {
    float y = 0.f;
    float x = 0.f;
    MeshPoint() {};
    MeshPoint(float y, float x) {
        this->y = y;
        this->x = x;
    };
};

std::vector<MeshPoint> downSampleMesh(
    int imageWidth, int imageHeight, int meshWidth, int meshHeight, int stepWidth, int stepHeight, const cv::Mat& mapX, const cv::Mat& mapY) {
    std::vector<MeshPoint> mesh;

    mesh.reserve(meshWidth * meshHeight);

    for(int y = 0; y <= mapX.rows; y += stepHeight) {
        int insertedX = 0;
        for(int x = 0; x <= mapX.cols; x += stepWidth) {
            if(y == mapX.rows && x == mapX.cols) {
                MeshPoint pt = MeshPoint(mapY.at<float>(y - 1, x - 1), mapX.at<float>(y - 1, x - 1));
                mesh.push_back(pt);
            } else if(y == mapX.rows) {
                MeshPoint pt = MeshPoint(mapY.at<float>(y - 1, x), mapX.at<float>(y - 1, x));
                mesh.push_back(pt);
            } else if(x == mapX.cols) {
                MeshPoint pt = MeshPoint(mapY.at<float>(y, x - 1), mapX.at<float>(y, x - 1));
                mesh.push_back(pt);
            } else {
                MeshPoint pt = MeshPoint(mapY.at<float>(y, x), mapX.at<float>(y, x));
                mesh.push_back(pt);
            }
            insertedX++;
        }
        for(int i = insertedX; i < meshWidth; i++) {
            MeshPoint pt = MeshPoint(-1, -1);
            mesh.push_back(pt);
        }
    }

    return mesh;
}

struct RectificationMatrices {
    cv::Mat R1;
    cv::Mat R2;
    cv::Mat KRect1;
    cv::Mat KRect2;
};

RectificationMatrices computeRectificationMatrices(cv::Mat M1,
                                                   cv::Mat d1,
                                                   cv::Mat M2,
                                                   cv::Mat d2,
                                                   cv::Size imageSize,
                                                   cv::Mat R,
                                                   cv::Mat T,
                                                   cv::Size outSize,
                                                   const std::optional<float>& alphaScaling) {
    cv::Mat R1, R2, P1, P2, Q;
    cv::Mat _cv_M1, _cv_M2, _cv_d1, _cv_d2, _cv_R, _cv_T;
    M1.convertTo(_cv_M1, CV_64FC1);
    M2.convertTo(_cv_M2, CV_64FC1);
    d1.convertTo(_cv_d1, CV_64FC1);
    d2.convertTo(_cv_d2, CV_64FC1);
    R.convertTo(_cv_R, CV_64FC1);
    T.convertTo(_cv_T, CV_64FC1);
    _cv_T = _cv_T.t();

    cv::stereoRectify(_cv_M1,
                      _cv_d1,
                      _cv_M2,
                      _cv_d2,
                      imageSize,
                      _cv_R,
                      _cv_T,
                      R1,
                      R2,
                      P1,
                      P2,
                      Q,
                      cv::CALIB_ZERO_DISPARITY,
                      alphaScaling.value_or(-1.0),
                      outSize);

    RectificationMatrices rectification;
    R1.convertTo(rectification.R1, CV_32FC1);
    R2.convertTo(rectification.R2, CV_32FC1);
    P1(cv::Rect(0, 0, 3, 3)).convertTo(rectification.KRect1, CV_32FC1);
    P2(cv::Rect(0, 0, 3, 3)).convertTo(rectification.KRect2, CV_32FC1);
    return rectification;
}

/** Raw-image position of every mesh point: the ray of rectified pixel (x * stepWidth, y * stepHeight) through
 * targetK and rotation R (raw to rectified, 3x3 float), projected by the camera's B-spline lens model. */
void splineMeshGrid(const dai::BsplineCameraCalibration& lens,
                    const cv::Mat& R,
                    const cv::Mat& targetK,
                    int stepWidth,
                    int stepHeight,
                    int meshWidth,
                    int meshHeight,
                    std::vector<double>& gridX,
                    std::vector<double>& gridY) {
    const double fx = targetK.at<float>(0, 0), fy = targetK.at<float>(1, 1), cx = targetK.at<float>(0, 2), cy = targetK.at<float>(1, 2);
    gridX.reserve(meshWidth * meshHeight);
    gridY.reserve(meshWidth * meshHeight);
    for(int y = 0; y < meshHeight; ++y) {
        for(int x = 0; x < meshWidth; ++x) {
            const std::array<double, 3> rectified{{(x * stepWidth - cx) / fx, (y * stepHeight - cy) / fy, 1}};
            std::array<double, 3> raw{{0, 0, 0}};  // R^T * rectified
            for(int i = 0; i < 3; ++i)
                for(int j = 0; j < 3; ++j) raw[i] += R.at<float>(j, i) * rectified[j];
            if(raw[2] <= 0) throw std::runtime_error("B-spline mesh contains a ray behind the raw camera");
            const auto pixel = lens.project(raw);
            gridX.push_back(pixel[0]);
            gridY.push_back(pixel[1]);
        }
    }
}

/** The camera's B-spline lens model, checked to match the input: same resolution, no upstream crop or scale. */
dai::BsplineCameraCalibration nativeBsplineLens(const dai::CalibrationHandler& calib, dai::CameraBoardSocket socket, const dai::ImgFrame& input, int width, int height) {
    const auto lens = calib.getBsplineCameraCalibration(socket);
    if(!lens) throw std::runtime_error(fmt::format("BSPLINE rectification selected but camera {} has no B-spline lens model", static_cast<int>(socket)));
    lens->validate();
    if(lens->width != width || lens->height != height) {
        throw std::runtime_error(
            fmt::format("B-spline lens model of camera {} is {}x{}, the input is {}x{}", static_cast<int>(socket), lens->width, lens->height, width, height));
    }
    if(input.transformation.isValid()) {
        const auto nativeK = calib.getCameraIntrinsics(socket, width, height);
        const auto inputK = input.transformation.getIntrinsicMatrix();
        for(int i = 0; i < 3; ++i)
            for(int j = 0; j < 3; ++j)
                if(std::abs(inputK[i][j] - nativeK[i][j]) > 0.01) throw std::runtime_error("B-spline rectification does not support cropped or scaled inputs yet");
    }
    return *lens;
}

template <typename T>
float translationMagnitudeMeters(const T& translationVec) {
    const auto x = static_cast<float>(translationVec[0]);
    const auto y = static_cast<float>(translationVec[1]);
    const auto z = static_cast<float>(translationVec[2]);
    return std::sqrt(x * x + y * y + z * z) / 100.f;
}

float computeDepthNumeratorWithOverrides(float baselineMeters,
                                         float focalPixels,
                                         float unitsPerMeter,
                                         const std::optional<float>& baselineOverrideCm,
                                         const std::optional<float>& focalOverridePixels) {
    if(baselineOverrideCm.has_value()) {
        if(!std::isfinite(*baselineOverrideCm) || *baselineOverrideCm <= 0) {
            throw std::runtime_error("StereoDepth baseline override must be finite and positive");
        }
        baselineMeters = *baselineOverrideCm / 100.f;
    }
    if(focalOverridePixels.has_value()) {
        if(!std::isfinite(*focalOverridePixels) || *focalOverridePixels <= 0) {
            throw std::runtime_error("StereoDepth focal length override must be finite and positive");
        }
        focalPixels = *focalOverridePixels;
    }
    return baselineMeters * unitsPerMeter * focalPixels;
}

bool usesRightPrimaryAlignment(const dai::StereoDepthConfig::AlgorithmControl::DepthAlign align) {
    return align == dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT
           || align == dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RIGHT;
}

bool usesRectifiedOutputAlignment(const dai::StereoDepthConfig::AlgorithmControl::DepthAlign align) {
    return align == dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT
           || align == dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT;
}

dai::StereoDepthConfig::AlgorithmControl::DepthAlign resolveEffectiveAlignment(const dai::StereoDepthConfig::AlgorithmControl::DepthAlign requested,
                                                                               const dai::StereoDepthConfig::AlgorithmControl::DepthAlign autoAlignment) {
    return requested == dai::StereoDepthConfig::AlgorithmControl::DepthAlign::AUTO ? autoAlignment : requested;
}

void setFrameTransformation(dai::ImgFrame& frame, dai::ImgTransformation transformation) {
    if(transformation.isValid()) {
        const auto [sourceWidth, sourceHeight] = transformation.getSourceSize();
        frame.setSourceSize(sourceWidth, sourceHeight);
    }
    frame.transformation = std::move(transformation);
}

template <typename T>
dai::StereoDepthConfig::AlgorithmControl::DepthAlign decideAutoAlignmentFromCamAProximity(const T& camATranslationToLeft, const T& camATranslationToRight) {
    const auto leftDistance = translationMagnitudeMeters(camATranslationToLeft);
    const auto rightDistance = translationMagnitudeMeters(camATranslationToRight);
    return rightDistance < leftDistance * CAM_A_PROXIMITY_SWITCH_RATIO ? dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT
                                                                       : dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT;
}

}  // namespace

namespace dai {
namespace gate {

using namespace std::chrono;

void StereoDepth::_ThreadWarp() {
    int fps = 30;  // todo
    float syncThresholdMs = 1000.f / fps / 2;
    std::shared_ptr<Pool<ImgFrame>> rectifiedFrameLeftPool, rectifiedFrameRightPool;

    if(enableRectification) {
        rectifiedFrameLeftPool = std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outRectifiedFrameSize);
        rectifiedFrameRightPool = std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outRectifiedFrameSize);

        if(rectifiedFrameLeftPool == nullptr) {
            throw std::runtime_error(fmt::format("Stereo allocation error for rectified left frame pool"));
        }
        if(rectifiedFrameRightPool == nullptr) {
            throw std::runtime_error(fmt::format("Stereo allocation error for rectified right frame pool"));
        }
    }

    uint32_t currentEepromId = getParentPipeline().getEepromId();

    while(isRunning() && threadsRunning) {
        auto startTs = std::chrono::steady_clock::now();  // Main loop timing
        auto warpBlock = this->blockEvent(PipelineEvent::Type::CUSTOM, "warp");

        auto tStartSync = steady_clock::now();
        std::shared_ptr<const dai::ImgFrame> leftIn, rightIn;
        {
            auto blockEvent = this->inputBlockEvent();
            if(!lazyLoad) {
                leftIn = std::static_pointer_cast<const dai::ImgFrame>(left.get<dai::ImgFrame>());
                rightIn = std::static_pointer_cast<const dai::ImgFrame>(right.get<dai::ImgFrame>());
            } else {
                leftIn = _leftIn;
                rightIn = _rightIn;
                _leftIn = nullptr;
                _rightIn = nullptr;
                lazyLoad = false;
            }
            tStartSync = steady_clock::now();
            while(1) {
                auto tsL = leftIn->getTimestampDevice();
                auto tsR = rightIn->getTimestampDevice();
                auto diffMs = duration_cast<milliseconds>(tsL - tsR).count();

                if(std::abs(diffMs) > syncThresholdMs) {
                    if(diffMs < 0) {
                        leftIn = std::static_pointer_cast<const dai::ImgFrame>(left.get<dai::ImgFrame>());
                    } else {
                        rightIn = std::static_pointer_cast<const dai::ImgFrame>(right.get<dai::ImgFrame>());
                    }
                } else {
                    break;
                }
            }
        }

        this->pipelineEventDispatcher->startTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", leftIn->sequenceNum, startTs);

        uint32_t latestEepromId = getParentPipeline().getEepromId();

        if(latestEepromId > currentEepromId) {
            logger->debug("EEPROM data changed (ID: {} -> {}), reconfiguring calibration with latest frames.", currentEepromId, latestEepromId);
            initCalib(leftIn, rightIn);
            currentEepromId = latestEepromId;
        }

        auto tEndSync = steady_clock::now();
        auto syncRuntime = duration_cast<microseconds>(tEndSync - tStartSync).count() / 1000.f;
        logger->debug("Stereo frame sync took {} ms", syncRuntime);

        auto syncedTs = leftIn->ts;
        if(alignedSocket == rightSocket) {
            syncedTs = rightIn->ts;
        }

        auto tWarpAllocationStart = steady_clock::now();
        auto tWarpMemConvertStart = tWarpAllocationStart;

        auto leftInAccel = std::make_shared<ImgFrame>();
        auto rightInAccel = std::make_shared<ImgFrame>();
        leftInAccel->setMetadata(*leftIn);
        rightInAccel->setMetadata(*rightIn);
        leftInAccel->data = AcceleratorMemory::convert(leftIn->data);
        rightInAccel->data = AcceleratorMemory::convert(rightIn->data);

        auto tWarpMemConvertEnd = steady_clock::now();

        auto warpConvertRuntime = duration_cast<microseconds>(tWarpMemConvertEnd - tWarpMemConvertStart).count() / 1000.f;
        logger->debug("Stereo rectification memory convert took {} ms", warpConvertRuntime);

        auto tStartWarp = steady_clock::now();
        auto tWarpAllocationEnd = tStartWarp;

        std::shared_ptr<dai::ImgFrame> rectifiedLeftFrame, rectifiedRightFrame;

        if(enableRectification) {
            rectifiedLeftFrame = rectifiedFrameLeftPool->get();
            // rectifiedLeftFrame->set(leftIn->get());
            rectifiedLeftFrame->setMetadata(*leftIn);
            rectifiedLeftFrame->setWidth(outFrameWidth);
            rectifiedLeftFrame->setHeight(outFrameHeight);
            rectifiedLeftFrame->setType(dai::ImgFrame::Type::RAW8);
            if(rectifiedLeftFrame->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            rectifiedRightFrame = rectifiedFrameRightPool->get();
            // rectifiedRightFrame->set(rightIn->get());
            rectifiedRightFrame->setMetadata(*rightIn);
            rectifiedRightFrame->setWidth(outFrameWidth);
            rectifiedRightFrame->setHeight(outFrameHeight);
            rectifiedRightFrame->setType(dai::ImgFrame::Type::RAW8);
            if(rectifiedRightFrame->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            tWarpAllocationEnd = steady_clock::now();
            tStartWarp = tWarpAllocationEnd;

            if(0) {
                // cv::remap(leftIn->getFrame(), rectifiedLeftFrame->getFrame(), map_x_l, map_y_l, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
                // cv::remap(rightIn->getFrame(), rectifiedRightFrame->getFrame(), map_x_r, map_y_r, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0,
                // 0));
            } else {
                warp.RunAsync(leftInAccel, rectifiedLeftFrame, 0);
                warp.Wait();

                warp.RunAsync(rightInAccel, rectifiedRightFrame, 1);
                warp.Wait();
            }

            auto rectifiedLeftTransformation = rectifiedLeftFrame->transformation;
            auto rectifiedRightTransformation = rectifiedRightFrame->transformation;
            setRectifiedTransformations(rectifiedLeftTransformation, cv_R_left, cv_KRect_left);
            setRectifiedTransformations(rectifiedRightTransformation, cv_R_right, cv_KRect_right);
            setFrameTransformation(*rectifiedLeftFrame, std::move(rectifiedLeftTransformation));
            setFrameTransformation(*rectifiedRightFrame, std::move(rectifiedRightTransformation));
        }

        auto warpAllocRuntime = duration_cast<microseconds>(tWarpAllocationEnd - tWarpAllocationStart).count() / 1000.f;
        logger->debug("Stereo rectification memory allocation took {} ms", warpAllocRuntime);

        auto tEndWarp = steady_clock::now();
        auto warpRuntime = duration_cast<microseconds>(tEndWarp - tStartWarp).count() / 1000.f;
        logger->debug("Stereo rectification took {} ms", warpRuntime);
        WarpOut warpOutFrames;
        warpOutFrames.leftIn = leftIn;
        warpOutFrames.rightIn = rightIn;
        if(enableRectification) {
            warpOutFrames.rectifiedLeftFrame = rectifiedLeftFrame;
            warpOutFrames.rectifiedRightFrame = rectifiedRightFrame;
        } else {
            warpOutFrames.rectifiedLeftFrame = leftInAccel;
            warpOutFrames.rectifiedRightFrame = rightInAccel;
        }
        warpOutFrames.performanceMetrics.tStartSync = tStartSync;
        warpOutFrames.performanceMetrics.tEndSync = tEndSync;
        warpOutFrames.performanceMetrics.tWarpAllocationStart = tWarpAllocationStart;
        warpOutFrames.performanceMetrics.tWarpAllocationEnd = tWarpAllocationEnd;
        warpOutFrames.performanceMetrics.tStartWarp = tStartWarp;
        warpOutFrames.performanceMetrics.tEndWarp = tEndWarp;
        auto tWarpPushStart = steady_clock::now();
        auto warpTotal = duration_cast<microseconds>(tWarpPushStart - tStartSync).count() / 1000.f;
        logger->debug("Stereo rectification (total) took {} ms", warpTotal);
        warpOutFrames.performanceMetrics.tWarpPushStart = tWarpPushStart;

        warpOut.push(warpOutFrames);
        auto tWarpPushEnd = steady_clock::now();
        auto warpPushTime = duration_cast<microseconds>(tWarpPushEnd - tWarpPushStart).count() / 1000.f;
        logger->debug("Stereo rectification message push took {} ms", warpPushTime);
    }
}

template <typename T>
void StereoDepth::combineLevels(T* reference, T* downScaled, T* out, uint32_t referenceWidth, uint32_t numDisparities) {
    const int downscaleFactor = 2;
    numDisparities -= 1;
    uint32_t half_disp = numDisparities * 16;
    for(uint32_t hl_col = 0; hl_col < referenceWidth; ++hl_col) {
        uint32_t ll_col = hl_col / 2;
        uint16_t expanded = downscaleFactor * downScaled[ll_col];
        out[hl_col] = expanded > half_disp ? expanded : reference[hl_col];
    }
}

void StereoDepth::combineDispMaps(std::shared_ptr<dai::ImgFrame>& disparityFrame,
                                  std::shared_ptr<dai::ImgFrame>& disparityFrameDs,
                                  std::shared_ptr<dai::ImgFrame>& result,
                                  uint16_t maxDisp,
                                  int downscaled) {
    uint16_t* ref = (std::uint16_t*)disparityFrame->data->getData().data();
    uint16_t* dscaled = (std::uint16_t*)disparityFrameDs->data->getData().data();
    uint16_t* out = (std::uint16_t*)result->data->getData().data();

    for(int n = 0; n < outFrameHeight; n++) {
        // Combine levels foreach disparity reference line
        combineLevels(ref, dscaled, out, outFrameWidth, maxDisp);

        ref += outFrameWidth;
        out += outFrameWidth;

        if(n % 2) dscaled += outFrameWidth / downscaled;
    }
}

void StereoDepth::disparityToDepth(std::shared_ptr<dai::ImgFrame>& disparityFrame, std::shared_ptr<dai::ImgFrame>& depthFrame, int scaleFactor) {
    uint16_t* disp = (std::uint16_t*)disparityFrame->data->getData().data();
    uint16_t* depth = (std::uint16_t*)depthFrame->data->getData().data();

    if(disparityFrame->getWidth() != depthFrame->getWidth()) {
        logger->error("Disparity and depth frame width mismatch!");
        return;
    }
    if(disparityFrame->getHeight() != depthFrame->getHeight()) {
        logger->error("Disparity and depth frame height mismatch!");
        return;
    }
    for(int i = 0; i < disparityFrame->getWidth() * disparityFrame->getHeight(); i++) {
        if(disp[i] == 0) {
            depth[i] = 0;
        } else {
            depth[i] = fixedNumerator * 16 * scaleFactor / disp[i];
        }
    }
}

void StereoDepth::leftRightCheck(uint16_t* _disp_LR,
                                 uint16_t* _disp_RL,
                                 uint8_t* _conf_LR,
                                 uint8_t* _conf_RL,
                                 uint16_t* _out,
                                 uint32_t width,
                                 uint32_t height,
                                 uint32_t threshold,
                                 bool primarySearchR2L) {
    for(int32_t row = 0; row < height; ++row) {
        uint16_t* disp_LR = _disp_LR + row * width;
        uint16_t* disp_RL = _disp_RL + row * width;
        uint8_t* conf_LR = _conf_LR + row * width;
        uint8_t* conf_RL = _conf_RL + row * width;
        uint16_t tmp_buff[width];  // TODO

        for(int32_t col = 0; col < width; ++col) {
            int16_t disp_LR_val = disp_LR[col];

            int16_t disp_RL_val_int = disp_LR_val / 16;

            int32_t RL_offset = primarySearchR2L ? col + disp_RL_val_int : col - disp_RL_val_int;
            bool valid_mask = RL_offset > 0 && RL_offset < static_cast<int32_t>(width);

            int16_t disp_RL_val = 0;

            if(valid_mask) disp_RL_val = disp_RL[RL_offset];

            int16_t abs_diff = std::abs(disp_LR_val - disp_RL_val);

            if(threshold * 16 > abs_diff) {
                tmp_buff[col] = (disp_LR_val + disp_RL_val) / 2;
            } else {
                tmp_buff[col] = 0;
            }
        }

        uint16_t* out = _out + row * width;
        for(int32_t col = 0; col < width; ++col) {
            out[col] = tmp_buff[col];
        }
    }
}

void StereoDepth::confidenceInvalidation(uint16_t* _inDisp, uint8_t* _inConf, uint16_t* _outDisp, uint32_t width, uint32_t height, uint32_t threshold) {
    for(int32_t row = 0; row < height; ++row) {
        uint16_t* inDisp = _inDisp + row * width;
        uint8_t* inConf = _inConf + row * width;
        uint16_t* outDisp = _outDisp + row * width;
        for(uint32_t col = 0; col < width; ++col) {
            if(threshold > inConf[col]) {
                outDisp[col] = 0;
            } else {
                outDisp[col] = inDisp[col];
            }
        }
    }
}

void StereoDepth::_ThreadStereo() {
    auto disparityFramePool = std::make_shared<Pool<ImgFrame>>(std::max(4, properties.numFramesPool), outDisparityFrameSize);
    std::shared_ptr<Pool<ImgFrame>> disparityDsFramePool =
        extendedMode ? std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outDisparityFrameSize) : nullptr;
    std::shared_ptr<Pool<ImgFrame>> confidenceDsFramePool =
        extendedMode ? std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outConfidenceFrameSize) : nullptr;
    auto confidenceFramePool = std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outConfidenceFrameSize);

    auto disparityFramePool_RL = std::make_shared<Pool<ImgFrame>>(std::max(4, properties.numFramesPool), outDisparityFrameSize);
    auto confidenceFramePool_RL = std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outConfidenceFrameSize);

    std::shared_ptr<Pool<ImgFrame>> disparityDsFramePool_RL =
        extendedMode ? std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outDisparityFrameSize) : nullptr;
    std::shared_ptr<Pool<ImgFrame>> confidenceDsFramePool_RL =
        extendedMode ? std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outConfidenceFrameSize) : nullptr;

    std::shared_ptr<Pool<ImgFrame>> pyrFrameLeftPool =
        extendedMode ? std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, pyrImg.GetMinPyrImgBytes()) : nullptr;
    std::shared_ptr<Pool<ImgFrame>> pyrFrameRightPool =
        extendedMode ? std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, pyrImg.GetMinPyrImgBytes()) : nullptr;

    StereoConfig stereoConfig(properties.initialConfig);
    auto err = stereoDepth.Config(stereoConfig);
    if(err) {
        logger->error("StereoDepth config failed with error: {}", err);
    }

    StereoConfig stereoConfigDs = stereoConfig;
    if(extendedMode) {
        err = stereoDepthDs.Config(stereoConfigDs);
        if(err) {
            logger->error("StereoDepth config failed with error: {}", err);
        }
    }

    auto inConfigData = AcceleratorMemory::create(sizeof(StereoDepthDspConfig));
    auto& dspConfig = *reinterpret_cast<StereoDepthDspConfig*>(inConfigData->data());

    decltype(high_resolution_clock::now()) startTimer, stopTimer;

    while(isRunning() && threadsRunning) {
        auto warpBlock = this->blockEvent(PipelineEvent::Type::CUSTOM, "stereo");

        WarpOut warpIn;
        bool success = warpOut.waitAndPop(warpIn);
        if(!success) {
            logger->error("Stereo thread failed to receive input!");
            break;
        }
        auto& performanceMetrics = warpIn.performanceMetrics;

        auto tStereoPop = steady_clock::now();
        auto stereoPopTime = duration_cast<microseconds>(tStereoPop - performanceMetrics.tWarpPushStart).count() / 1000.f;
        logger->debug("Stereo main thread pop took {} ms", stereoPopTime);

        performanceMetrics.tStereoPop = tStereoPop;
        auto& tStartSync = performanceMetrics.tStartSync;

        auto& leftIn = warpIn.leftIn;
        auto& rightIn = warpIn.rightIn;
        auto& rectifiedLeftFrame = warpIn.rectifiedLeftFrame;
        auto& rectifiedRightFrame = warpIn.rectifiedRightFrame;

        auto& referenceFrame = alignedSocket == rightSocket ? rightIn : leftIn;

        auto tStereoAllocationStart = steady_clock::now();

        std::shared_ptr<dai::ImgFrame> disparityFrame = disparityFramePool->get();
        // disparityFrame->set(referenceFrame->get());
        disparityFrame->setMetadata(*referenceFrame);
        disparityFrame->setWidth(outFrameWidth);
        disparityFrame->setHeight(outFrameHeight);
        disparityFrame->setType(dai::ImgFrame::Type::RAW16);
        disparityFrame->fb.stride = disparityFrame->fb.width * disparityFrame->fb.bytesPP;
        if(disparityFrame->data == nullptr) {
            logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
            continue;
        }

        std::shared_ptr<dai::ImgFrame> confidenceFrame = confidenceFramePool->get();
        // confidenceFrame->set(referenceFrame->get());
        confidenceFrame->setMetadata(*referenceFrame);
        confidenceFrame->setWidth(outFrameWidth);
        confidenceFrame->setHeight(outFrameHeight);
        confidenceFrame->setType(dai::ImgFrame::Type::RAW8);
        confidenceFrame->fb.stride = confidenceFrame->fb.width * confidenceFrame->fb.bytesPP;
        if(confidenceFrame->data == nullptr) {
            logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
            continue;
        }

        bool newConfig = false;
        std::shared_ptr<StereoDepthConfig> pConfig;
        if(inputConfig.getWaitForMessage()) {
            pConfig = inputConfig.get<StereoDepthConfig>();
        } else {
            pConfig = inputConfig.tryGet<StereoDepthConfig>();
        }

        if(pConfig) {
            logger->debug("Got new stereoConfig..");
            auto rawCfg = *pConfig;

            logger->debug("Applying new stereoConfig..");
            stereoConfig = StereoConfig(rawCfg);
            stereoDepth.Config(stereoConfig);

            bool oldExtendedMode = extendedMode;

            if(!oldExtendedMode && rawCfg.algorithmControl.enableExtended) {
                logger->error("Extended mode runtime config is not yet supported!");
                continue;
            }

            extendedMode = rawCfg.algorithmControl.enableExtended;

            if(extendedMode) {
                stereoConfigDs = StereoConfig(rawCfg);
                stereoDepthDs.Config(stereoConfigDs);
            }

            latestConfig = pConfig;
            newConfig = true;
        }

        bool runPostProcessingOnCPU = latestConfig->filtersBackend == dai::ProcessorType::CPU;
        const auto effectiveAlignment = resolveEffectiveAlignment(latestConfig->algorithmControl.depthAlign, autoAlignment);
        const bool primarySearchR2L = usesRightPrimaryAlignment(effectiveAlignment);

        logger->debug("Extended mode is  - {}", extendedMode ? "on" : "off");

        auto tStereoAllocationEnd = steady_clock::now();
        auto tStartStereo = tStereoAllocationEnd;

        stereoConfig.eSearchDir = primarySearchR2L ? StereoSearchDir::R2L : StereoSearchDir::L2R;
        err = stereoDepth.Config(stereoConfig);
        if(err) {
            logger->error("StereoDepth config failed with error: {}", err);
        }
        stereoDepth.RunAsync(primarySearchR2L ? rectifiedRightFrame : rectifiedLeftFrame,
                             primarySearchR2L ? rectifiedLeftFrame : rectifiedRightFrame,
                             disparityFrame,
                             confidenceFrame,
                             nullptr);

        std::shared_ptr<dai::ImgFrame> disparityFrameDs = nullptr;
        std::shared_ptr<dai::ImgFrame> rectifiedLeftFrameDs = nullptr;
        std::shared_ptr<dai::ImgFrame> rectifiedRightFrameDs = nullptr;
        std::shared_ptr<dai::ImgFrame> confidenceFrameDs = nullptr;

        if(extendedMode) {
            auto t1 = steady_clock::now();

            PyrImgConfig pyrImgCfg = {.eDsInterpolation = ScaledownInterpolation::BILINEAR};
            pyrImg.Config(pyrImgCfg);

            // Starting downscale - left
            rectifiedLeftFrameDs = pyrFrameLeftPool->get();
            // rectifiedLeftFrameDs->set(rectifiedLeftFrame->get());
            rectifiedLeftFrameDs->setMetadata(*rectifiedLeftFrame);
            rectifiedLeftFrameDs->setWidth(outFrameWidth / 2);
            rectifiedLeftFrameDs->setHeight(outFrameHeight / 2);
            rectifiedLeftFrameDs->setType(dai::ImgFrame::Type::RAW8);

            outLeftPyrFrames.clear();
            outLeftPyrFrames.push_back(nullptr);
            outLeftPyrFrames.push_back(rectifiedLeftFrameDs);

            pyrImg.RunAsync(rectifiedLeftFrame, outLeftPyrFrames);
            pyrImg.Wait();

            // Starting downscale - right
            rectifiedRightFrameDs = pyrFrameRightPool->get();
            // rectifiedRightFrameDs->set(rectifiedRightFrame->get());
            rectifiedRightFrameDs->setMetadata(*rectifiedRightFrame);
            rectifiedRightFrameDs->setWidth(outFrameWidth / 2);
            rectifiedRightFrameDs->setHeight(outFrameHeight / 2);
            rectifiedRightFrameDs->setType(dai::ImgFrame::Type::RAW8);

            outRightPyrFrames.clear();
            outRightPyrFrames.push_back(nullptr);
            outRightPyrFrames.push_back(rectifiedRightFrameDs);

            pyrImg.RunAsync(rectifiedRightFrame, outRightPyrFrames);

            auto t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->debug("Stereo DS configuration took {} ms", elapsed);
        }

        stereoDepth.Wait();

        if(latestConfig->costMatching.enableSwConfidenceThresholding) {
            logger->debug("Confidence threshold: {}", latestConfig->costMatching.confidenceThreshold);

            startTimer = high_resolution_clock::now();

            if(runPostProcessingOnCPU) {
                uint16_t* disp_LR = (uint16_t*)disparityFrame->data->getData().data();
                uint8_t* conf_LR = (uint8_t*)confidenceFrame->data->getData().data();

                confidenceInvalidation(
                    disp_LR, conf_LR, disp_LR, disparityFrame->getWidth(), disparityFrame->getHeight(), latestConfig->costMatching.confidenceThreshold);

            } else {
                auto inDisparityFrame = AcceleratorMemory::convert(disparityFrame->data);
                auto inConfidenceFrame = AcceleratorMemory::convert(confidenceFrame->data);
                auto outDisparityFrame = inDisparityFrame;

                dspConfig.width = disparityFrame->getWidth();
                dspConfig.height = disparityFrame->getHeight();
                dspConfig.confidenceThreshold = latestConfig->costMatching.confidenceThreshold;
                const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityFrame), dspIn1 = dai::device::toDeviceBuf(*inConfidenceFrame),
                                     dspConfig = dai::device::toDeviceBuf(*inConfigData), dspOut = dai::device::toDeviceBuf(*outDisparityFrame);
                if(int err = dai_device_dsp_confidence_invalidation(&dspIn0, &dspIn1, &dspConfig, &dspOut)) {
                    logger->error("DSP: Confidence invalidation failed with error: {}, {}", hexagon::errorToString(err), __LINE__);
                }
            }

            stopTimer = high_resolution_clock::now();
            logger->debug("Confidence invalidation time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);
        }

        std::shared_ptr<dai::ImgFrame> disparityFrameSecondary = nullptr;
        std::shared_ptr<dai::ImgFrame> confidenceFrameSecondary = nullptr;

        if(latestConfig->algorithmControl.enableSwLeftRightCheck) {
            stereoConfig.eSearchDir = primarySearchR2L ? StereoSearchDir::L2R : StereoSearchDir::R2L;
            err = stereoDepth.Config(stereoConfig);
            if(err) {
                logger->error("StereoDepth config failed with error: {}", err);
            }

            disparityFrameSecondary = disparityFramePool_RL->get();
            disparityFrameSecondary->setMetadata(*referenceFrame);
            disparityFrameSecondary->setWidth(outFrameWidth);
            disparityFrameSecondary->setHeight(outFrameHeight);
            disparityFrameSecondary->setType(dai::ImgFrame::Type::RAW16);
            disparityFrameSecondary->fb.stride = disparityFrameSecondary->fb.width * disparityFrameSecondary->fb.bytesPP;
            if(disparityFrameSecondary->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            confidenceFrameSecondary = confidenceFramePool_RL->get();
            confidenceFrameSecondary->setMetadata(*referenceFrame);
            confidenceFrameSecondary->setWidth(outFrameWidth);
            confidenceFrameSecondary->setHeight(outFrameHeight);
            confidenceFrameSecondary->setType(dai::ImgFrame::Type::RAW8);
            confidenceFrameSecondary->fb.stride = disparityFrameSecondary->fb.width * disparityFrameSecondary->fb.bytesPP;
            if(confidenceFrameSecondary->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            stereoDepth.Run(primarySearchR2L ? rectifiedLeftFrame : rectifiedRightFrame,
                            primarySearchR2L ? rectifiedRightFrame : rectifiedLeftFrame,
                            disparityFrameSecondary,
                            confidenceFrameSecondary,
                            nullptr);

            if(latestConfig->costMatching.enableSwConfidenceThresholding) {
                startTimer = high_resolution_clock::now();

                if(runPostProcessingOnCPU) {
                    uint16_t* disp_RL = (uint16_t*)disparityFrameSecondary->data->getData().data();
                    uint8_t* conf_RL = (uint8_t*)confidenceFrameSecondary->data->getData().data();

                    confidenceInvalidation(disp_RL,
                                           conf_RL,
                                           disp_RL,
                                           disparityFrameSecondary->getWidth(),
                                           disparityFrameSecondary->getHeight(),
                                           latestConfig->costMatching.confidenceThreshold);
                } else {
                    auto inDisparityFrame = AcceleratorMemory::convert(disparityFrameSecondary->data);
                    auto inConfidenceFrame = AcceleratorMemory::convert(confidenceFrameSecondary->data);
                    auto outDisparityFrame = inDisparityFrame;

                    dspConfig.width = disparityFrame->getWidth();
                    dspConfig.height = disparityFrame->getHeight();
                    dspConfig.confidenceThreshold = latestConfig->costMatching.confidenceThreshold;
                    const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityFrame), dspIn1 = dai::device::toDeviceBuf(*inConfidenceFrame),
                                         dspConfig = dai::device::toDeviceBuf(*inConfigData), dspOut = dai::device::toDeviceBuf(*outDisparityFrame);
                    if(int err = dai_device_dsp_confidence_invalidation(&dspIn0, &dspIn1, &dspConfig, &dspOut)) {
                        logger->error("DSP: Confidence invalidation failed with error: {}, {}", hexagon::errorToString(err), __LINE__);
                    }
                }

                stopTimer = high_resolution_clock::now();
                logger->debug("Confidence invalidation time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);
            }

            logger->debug("Left-right check enabled with threshold: {}", latestConfig->algorithmControl.leftRightCheckThreshold);

            startTimer = high_resolution_clock::now();
            if(runPostProcessingOnCPU) {
                uint16_t* disp_LR = (uint16_t*)disparityFrame->data->getData().data();
                uint8_t* conf_LR = (uint8_t*)confidenceFrame->data->getData().data();
                uint16_t* disp_RL = (uint16_t*)disparityFrameSecondary->data->getData().data();
                uint8_t* conf_RL = (uint8_t*)confidenceFrameSecondary->data->getData().data();

                leftRightCheck(disp_LR,
                               disp_RL,
                               conf_LR,
                               conf_RL,
                               disp_LR,
                               disparityFrame->getWidth(),
                               disparityFrame->getHeight(),
                               latestConfig->algorithmControl.leftRightCheckThreshold,
                               primarySearchR2L);
            } else {
                auto inDisparityLR = AcceleratorMemory::convert(disparityFrame->data);
                auto inDisparityRL = AcceleratorMemory::convert(disparityFrameSecondary->data);
                auto outDisparityFrame = inDisparityLR;

                dspConfig.width = disparityFrame->getWidth();
                dspConfig.height = disparityFrame->getHeight();
                dspConfig.leftRightCheckThreshold = latestConfig->algorithmControl.leftRightCheckThreshold;
                dspConfig.primarySearchR2L = primarySearchR2L ? 1 : 0;
                const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityLR), dspIn1 = dai::device::toDeviceBuf(*inDisparityRL),
                                     dspConfig = dai::device::toDeviceBuf(*inConfigData), dspOut = dai::device::toDeviceBuf(*outDisparityFrame);
                if(int err = dai_device_dsp_left_right_check(&dspIn0, &dspIn1, &dspConfig, &dspOut)) {
                    logger->error("DSP: Left-right check failed with error: {}, {}", hexagon::errorToString(err), __LINE__);
                }
            }
            stopTimer = high_resolution_clock::now();
            logger->debug("Left-right check time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);
        }

        auto stereoAllocRuntime = duration_cast<microseconds>(tStereoAllocationEnd - tStereoAllocationStart).count() / 1000.f;
        logger->debug("Stereo memory allocation took {} ms", stereoAllocRuntime);

        auto tEndStereo = steady_clock::now();
        auto stereoRuntime = duration_cast<microseconds>(tEndStereo - tStartStereo).count() / 1000.f;
        logger->debug("Stereo disparity took {} ms", stereoRuntime);

        if(extendedMode) {
            auto t1 = steady_clock::now();

            logger->debug("Preparing for DS..");

            disparityFrameDs = disparityDsFramePool->get();
            // disparityFrameDs->set(referenceFrame->get());
            disparityFrameDs->setMetadata(*referenceFrame);
            disparityFrameDs->setWidth(outFrameWidth / 2);
            disparityFrameDs->setHeight(outFrameHeight / 2);
            disparityFrameDs->setType(dai::ImgFrame::Type::RAW16);
            disparityFrameDs->fb.stride = disparityFrameDs->fb.width * disparityFrameDs->fb.bytesPP;
            if(disparityFrameDs->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            confidenceFrameDs = confidenceDsFramePool->get();
            // confidenceFrameDs->set(referenceFrame->get());
            confidenceFrameDs->setMetadata(*referenceFrame);
            confidenceFrameDs->setWidth(outFrameWidth / 2);
            confidenceFrameDs->setHeight(outFrameHeight / 2);
            confidenceFrameDs->setType(dai::ImgFrame::Type::RAW8);
            confidenceFrameDs->fb.stride = confidenceFrameDs->fb.width * confidenceFrameDs->fb.bytesPP;
            if(confidenceFrameDs->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            logger->debug("Waiting for DS..");

            pyrImg.Wait();

            logger->debug("Starting stereo for DS..");

            stereoConfigDs.eSearchDir = primarySearchR2L ? StereoSearchDir::R2L : StereoSearchDir::L2R;
            err = stereoDepthDs.Config(stereoConfigDs);
            if(err) {
                logger->error("StereoDepth config failed with error: {}, {}", err, __LINE__);
            }
            stereoDepthDs.Run(primarySearchR2L ? rectifiedRightFrameDs : rectifiedLeftFrameDs,
                              primarySearchR2L ? rectifiedLeftFrameDs : rectifiedRightFrameDs,
                              disparityFrameDs,
                              confidenceFrameDs,
                              nullptr);

            if(latestConfig->costMatching.enableSwConfidenceThresholding) {
                startTimer = high_resolution_clock::now();
                if(runPostProcessingOnCPU) {
                    uint16_t* disp_LR_ds = (uint16_t*)disparityFrameDs->data->getData().data();
                    uint8_t* conf_LR_ds = (uint8_t*)confidenceFrameDs->data->getData().data();

                    confidenceInvalidation(disp_LR_ds,
                                           conf_LR_ds,
                                           disp_LR_ds,
                                           disparityFrameDs->getWidth(),
                                           disparityFrameDs->getHeight(),
                                           latestConfig->costMatching.confidenceThreshold);
                } else {
                    auto inDisparityFrame = AcceleratorMemory::convert(disparityFrameDs->data);
                    auto inConfidenceFrame = AcceleratorMemory::convert(confidenceFrameDs->data);
                    auto outDisparityFrame = inDisparityFrame;

                    dspConfig.width = disparityFrameDs->getWidth();
                    dspConfig.height = disparityFrameDs->getHeight();
                    dspConfig.confidenceThreshold = latestConfig->costMatching.confidenceThreshold;
                    const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityFrame), dspIn1 = dai::device::toDeviceBuf(*inConfidenceFrame),
                                         dspConfig = dai::device::toDeviceBuf(*inConfigData), dspOut = dai::device::toDeviceBuf(*outDisparityFrame);
                    if(int err = dai_device_dsp_confidence_invalidation(&dspIn0, &dspIn1, &dspConfig, &dspOut)) {
                        logger->error("DSP: Confidence invalidation failed with error: {}, {}", hexagon::errorToString(err), __LINE__);
                    }
                }
                stopTimer = high_resolution_clock::now();
                logger->debug("Confidence invalidation time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);
            }

            auto disparityFrameDsSecondary = disparityDsFramePool_RL->get();
            disparityFrameDsSecondary->setMetadata(*referenceFrame);
            disparityFrameDsSecondary->setWidth(outFrameWidth / 2);
            disparityFrameDsSecondary->setHeight(outFrameHeight / 2);
            disparityFrameDsSecondary->setType(dai::ImgFrame::Type::RAW16);
            disparityFrameDsSecondary->fb.stride = disparityFrameDsSecondary->fb.width * disparityFrameDsSecondary->fb.bytesPP;
            if(disparityFrameDsSecondary->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            auto confidenceFrameDsSecondary = confidenceDsFramePool_RL->get();
            confidenceFrameDsSecondary->setMetadata(*referenceFrame);
            confidenceFrameDsSecondary->setWidth(outFrameWidth / 2);
            confidenceFrameDsSecondary->setHeight(outFrameHeight / 2);
            confidenceFrameDsSecondary->setType(dai::ImgFrame::Type::RAW8);
            confidenceFrameDsSecondary->fb.stride = confidenceFrameDsSecondary->fb.width * confidenceFrameDsSecondary->fb.bytesPP;
            if(confidenceFrameDsSecondary->data == nullptr) {
                logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                continue;
            }

            if(latestConfig->algorithmControl.enableSwLeftRightCheck) {
                stereoConfigDs.eSearchDir = primarySearchR2L ? StereoSearchDir::L2R : StereoSearchDir::R2L;
                err = stereoDepthDs.Config(stereoConfigDs);
                if(err) {
                    logger->error("StereoDepth config failed with error: {}, {}", err, __LINE__);
                }
                stereoDepthDs.Run(primarySearchR2L ? rectifiedLeftFrameDs : rectifiedRightFrameDs,
                                  primarySearchR2L ? rectifiedRightFrameDs : rectifiedLeftFrameDs,
                                  disparityFrameDsSecondary,
                                  confidenceFrameDsSecondary,
                                  nullptr);

                if(latestConfig->costMatching.enableSwConfidenceThresholding) {
                    startTimer = high_resolution_clock::now();
                    if(runPostProcessingOnCPU) {
                        uint16_t* disp_RL_ds = (uint16_t*)disparityFrameDsSecondary->data->getData().data();
                        uint8_t* conf_RL_ds = (uint8_t*)confidenceFrameDsSecondary->data->getData().data();

                        confidenceInvalidation(disp_RL_ds,
                                               conf_RL_ds,
                                               disp_RL_ds,
                                               disparityFrameDsSecondary->getWidth(),
                                               disparityFrameDsSecondary->getHeight(),
                                               latestConfig->costMatching.confidenceThreshold);

                    } else {
                        auto inDisparityFrame = AcceleratorMemory::convert(disparityFrameDsSecondary->data);
                        auto inConfidenceFrame = AcceleratorMemory::convert(confidenceFrameDsSecondary->data);
                        auto outDisparityFrame = inDisparityFrame;

                        dspConfig.width = disparityFrameDsSecondary->getWidth();
                        dspConfig.height = disparityFrameDsSecondary->getHeight();
                        dspConfig.confidenceThreshold = latestConfig->costMatching.confidenceThreshold;
                        const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityFrame), dspIn1 = dai::device::toDeviceBuf(*inConfidenceFrame),
                                             dspConfig = dai::device::toDeviceBuf(*inConfigData), dspOut = dai::device::toDeviceBuf(*outDisparityFrame);
                        if(int err = dai_device_dsp_confidence_invalidation(&dspIn0, &dspIn1, &dspConfig, &dspOut)) {
                            logger->error("DSP: Confidence invalidation failed with error: {}, {}", hexagon::errorToString(err), __LINE__);
                        }
                    }
                    stopTimer = high_resolution_clock::now();
                    logger->debug("Confidence invalidation time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);
                }

                startTimer = high_resolution_clock::now();
                if(runPostProcessingOnCPU) {
                    uint16_t* disp_LR_ds = (uint16_t*)disparityFrameDs->data->getData().data();
                    uint8_t* conf_LR_ds = (uint8_t*)confidenceFrameDs->data->getData().data();
                    uint16_t* disp_RL_ds = (uint16_t*)disparityFrameDsSecondary->data->getData().data();
                    uint8_t* conf_RL_ds = (uint8_t*)confidenceFrameDsSecondary->data->getData().data();

                    leftRightCheck(disp_LR_ds,
                                   disp_RL_ds,
                                   conf_LR_ds,
                                   conf_RL_ds,
                                   disp_LR_ds,
                                   disparityFrameDs->getWidth(),
                                   disparityFrameDs->getHeight(),
                                   latestConfig->algorithmControl.leftRightCheckThreshold,
                                   primarySearchR2L);
                } else {
                    auto inDisparityLR = AcceleratorMemory::convert(disparityFrameDs->data);
                    auto inDisparityRL = AcceleratorMemory::convert(disparityFrameDsSecondary->data);
                    auto outDisparityFrame = inDisparityLR;

                    dspConfig.width = disparityFrameDs->getWidth();
                    dspConfig.height = disparityFrameDs->getHeight();
                    dspConfig.leftRightCheckThreshold = latestConfig->algorithmControl.leftRightCheckThreshold;
                    dspConfig.primarySearchR2L = primarySearchR2L ? 1 : 0;
                    const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityLR), dspIn1 = dai::device::toDeviceBuf(*inDisparityRL),
                                         dspConfig = dai::device::toDeviceBuf(*inConfigData), dspOut = dai::device::toDeviceBuf(*outDisparityFrame);

                    if(int err = dai_device_dsp_left_right_check(&dspIn0, &dspIn1, &dspConfig, &dspOut)) {
                        logger->error("DSP: Left-right check failed with error: {}, {}", hexagon::errorToString(err), __LINE__);
                    }
                }
                stopTimer = high_resolution_clock::now();
                logger->debug("Left-right check time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);
            }

            // Process the disparity output

            logger->debug("Stereo for DS finished. Merging the disparities ...");

            startTimer = high_resolution_clock::now();
            // auto combinedDisparity = dai::AcceleratorMemory::create(outDisparityFrameSize);
            if(runPostProcessingOnCPU) {
                combineDispMaps(disparityFrame, disparityFrameDs, disparityFrame, 64, 2);
            } else {
                auto inDisparityFrame = AcceleratorMemory::convert(disparityFrame->data);
                auto inDisparityFrameDs = AcceleratorMemory::convert(disparityFrameDs->data);
                auto outDisparityFrame = inDisparityFrame;

                dspConfig.width = disparityFrame->getWidth();
                dspConfig.height = disparityFrame->getHeight();
                dspConfig.numDisparities = 64;
                dspConfig.downscaleFactor = 2;
                const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityFrame), dspIn1 = dai::device::toDeviceBuf(*inDisparityFrameDs),
                                     dspConfig = dai::device::toDeviceBuf(*inConfigData), dspOut = dai::device::toDeviceBuf(*outDisparityFrame);
                if(int err = dai_device_dsp_combine_disparity_maps(&dspIn0, &dspIn1, &dspConfig, &dspOut)) {
                    logger->error("DSP: combineDisparityMaps failed with error code: {}, {}", hexagon::errorToString(err), __LINE__);
                }
            }
            stopTimer = high_resolution_clock::now();
            logger->debug("Disparity combination time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);
            // disparityFrame->data = combinedDisparity;

            auto t2 = steady_clock::now();
            auto elapsed = duration_cast<microseconds>(t2 - t1).count() / 1000.f;
            logger->debug("Stereo DS image took {} ms", elapsed);
        }

        if((disparityConnected || depthConnected) && !usesRectifiedOutputAlignment(effectiveAlignment)) {
            if(!map_x_reverse.empty() && !map_y_reverse.empty()) {
                cv::remap(disparityFrame->getFrame(),
                          disparityFrame->getFrame(),
                          map_x_reverse,
                          map_y_reverse,
                          cv::INTER_NEAREST,
                          cv::BORDER_CONSTANT,
                          cv::Scalar(0, 0, 0));
            }
        }

        int extendedMultiplier = extendedMode ? 2 : 1;
        int maxDisparity = (64 - 1) * extendedMultiplier * 16;

        auto stereoRuntimeTotal = duration_cast<microseconds>(tEndStereo - tStartSync).count() / 1000.f;
        logger->debug("Stereo took {} ms", stereoRuntimeTotal);

        performanceMetrics.tStereoAllocationStart = tStereoAllocationStart;
        performanceMetrics.tStereoAllocationEnd = tStereoAllocationEnd;
        performanceMetrics.tStartStereo = tStartStereo;
        performanceMetrics.tEndStereo = tEndStereo;

        auto tStereoPushStart = steady_clock::now();

        StereoOut _stereoOut;
        _stereoOut.disparityFrame = disparityFrame;
        _stereoOut.confidenceFrame = confidenceFrame;
        _stereoOut.warpPassthrough = warpIn;
        _stereoOut.newConfig = newConfig;
        _stereoOut.stereoConfig = latestConfig;
        _stereoOut.maxDisparity = maxDisparity;
        _stereoOut.warpPassthrough.performanceMetrics.tStereoPushStart = tStereoPushStart;
        _stereoOut.runPostProcessingOnCPU = runPostProcessingOnCPU;

        stereoOut.push(_stereoOut);

        auto tStereoPushEnd = steady_clock::now();
        auto stereoPushTime = duration_cast<microseconds>(tStereoPushEnd - tStereoPushStart).count() / 1000.f;
        logger->debug("Stereo message push took {} ms", stereoPushTime);
    }
}
void StereoDepth::setRectifiedTransformations(dai::ImgTransformation& transformation, const cv::Mat& cv_R, const cv::Mat& cv_KRect) const {
    const auto toDeviceId = transformation.getExtrinsics().toDeviceId;
    if(cv_R.empty() || cv_KRect.empty()) {
        // Custom rectification meshes don't carry a recoverable pinhole camera model.
        // Leave the output transformation invalid instead of advertising stale intrinsics as undistorted.
        transformation = dai::ImgTransformation();
        dai::Extrinsics extrinsics;
        extrinsics.toDeviceId = toDeviceId;
        transformation.setExtrinsics(extrinsics);
        return;
    }

    const auto distortionModel = transformation.getDistortionModel();
    auto currentExtrinsics = transformation.getExtrinsics();
    if(!currentExtrinsics.rotationMatrix.empty()) {
        auto currentRotationMat = matrix::vectorMatrixToMatrix3x3(currentExtrinsics.getRotationMatrix());
        currentExtrinsics.rotationMatrix = matrix::matrix3x3ToVectorMatrix(matrix::matMul(currentRotationMat, cvMatToArray3x3(cv_R.t())));
    }

    transformation = dai::ImgTransformation(outFrameWidth, outFrameHeight, cvMatToArray3x3(cv_KRect));
    transformation.setExtrinsics(currentExtrinsics);
    transformation.setDistortionCoefficients({});
    transformation.setDistortionModel(distortionModel);
}

bool isIdentityIntrinsics(const std::array<std::array<float, 3>, 3>& m, float eps = 1e-6f) {
    for(size_t i = 0; i < 3; ++i) {
        for(size_t j = 0; j < 3; ++j) {
            const float expected = (i == j) ? 1.f : 0.f;
            if(std::abs(m[i][j] - expected) > eps) {
                return false;
            }
        }
    }
    return true;
}

std::array<std::array<float, 3>, 3> intrinsicsFromVec(const std::vector<std::vector<float>>& m) {
    if(m.size() != 3 || m[0].size() != 3 || m[1].size() != 3 || m[2].size() != 3) {
        throw std::runtime_error("Invalid intrinsics matrix size, expected 3x3.");
    }

    return {{{m[0][0], m[0][1], m[0][2]}, {m[1][0], m[1][1], m[1][2]}, {m[2][0], m[2][1], m[2][2]}}};
}

std::array<std::array<float, 3>, 3> getIntrinsicsWithCalibrationFallback(
    const std::shared_ptr<const dai::ImgFrame>& frame, const CameraBoardSocket socket, CalibrationHandler& calib, uint32_t width, uint32_t height) {
    if(frame) {
        auto intrinsics = frame->transformation.getIntrinsicMatrix();
        if(!isIdentityIntrinsics(intrinsics)) {
            return intrinsics;
        }
    }

    return intrinsicsFromVec(calib.getCameraIntrinsics(socket, width, height));
}

void StereoDepth::_ThreadPostProcessing() {
    auto depthFramePool = std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outDisparityFrameSize);
    auto decimatedDisparityFramePool = std::make_shared<Pool<ImgFrame>>(properties.numFramesPool, outDisparityFrameSize);

    DecimationFilter decimationFilter;
    bool decimationFilterInit = false;

    TemporalFilter<uint16_t> temporalFilter;
    bool temporalFilterInit = false;

    SpatialFilter spatialFilter;
    bool spatialFilterInit = false;

    SpeckleFilter speckleFilter;
    ThresholdFilter thresholdFilter;
    dai2::MedianFilter medianFilter;
    BrightnessFilter brightnessFilter;

    auto inConfigData = AcceleratorMemory::create(sizeof(StereoDepthDspConfig));
    auto& dspConfig = *reinterpret_cast<StereoDepthDspConfig*>(inConfigData->data());

    while(isRunning() && threadsRunning) {
        auto postBlock = this->blockEvent(PipelineEvent::Type::CUSTOM, "postProcessing");

        int scaleFactor = 1;

        StereoOut stereoIn;
        bool success = stereoOut.waitAndPop(stereoIn);
        if(!success) {
            logger->error("PostProcessing thread failed to receive input!");
            break;
        }
        auto& performanceMetrics = stereoIn.warpPassthrough.performanceMetrics;

        auto tPostProcessingPop = steady_clock::now();
        auto popTime = duration_cast<microseconds>(tPostProcessingPop - performanceMetrics.tStereoPushStart).count() / 1000.f;
        logger->debug("PostProcessing main thread pop took {} ms", popTime);

        performanceMetrics.tPostProcessingPop = tPostProcessingPop;
        auto& tStartSync = performanceMetrics.tStartSync;

        auto warpIn = stereoIn.warpPassthrough;

        auto& leftIn = warpIn.leftIn;
        auto& rightIn = warpIn.rightIn;
        auto& rectifiedLeftFrame = warpIn.rectifiedLeftFrame;
        auto& rectifiedRightFrame = warpIn.rectifiedRightFrame;

        auto& referenceFrame = alignedSocket == rightSocket ? rightIn : leftIn;

        auto& disparityFrame = stereoIn.disparityFrame;
        auto& confidenceFrame = stereoIn.confidenceFrame;
        auto stereoConfig = stereoIn.stereoConfig;
        const auto effectiveAlignment = resolveEffectiveAlignment(stereoConfig->algorithmControl.depthAlign, autoAlignment);

        if(stereoIn.newConfig) {
            decimationFilterInit = false;
            temporalFilterInit = false;
            spatialFilterInit = false;
        }

        auto tStartPostProcessing = steady_clock::now();

        int maxDisparity = stereoIn.maxDisparity;

        bool runPostProcessingOnCPU = stereoIn.runPostProcessingOnCPU;

        if(stereoConfig->postProcessing.brightnessFilter.minBrightness > 0 || stereoConfig->postProcessing.brightnessFilter.maxBrightness < 255) {
            logger->debug("Brightness filter enabled, min: {}, max: {}",
                          stereoConfig->postProcessing.brightnessFilter.minBrightness,
                          stereoConfig->postProcessing.brightnessFilter.maxBrightness);
            brightnessFilter.process(rectifiedLeftFrame,
                                     disparityFrame,
                                     stereoConfig->postProcessing.brightnessFilter.minBrightness,
                                     stereoConfig->postProcessing.brightnessFilter.maxBrightness);
        }

        auto decimationFactor = stereoConfig->postProcessing.decimationFilter.decimationFactor;
        bool decimationEnabled = decimationFactor > 1;
        bool decimationApplied = false;
        bool medianFilterEnabled = stereoConfig->postProcessing.median != MedianFilter::MEDIAN_OFF;

        std::vector<dai::StereoDepthConfig::PostProcessing::Filter> filtersToExecute;

        for(auto filter : stereoConfig->postProcessing.filteringOrder) {
            switch(filter) {
                case StereoDepthConfig::PostProcessing::Filter::SPECKLE:
                    if(stereoConfig->postProcessing.speckleFilter.enable) {
                        filtersToExecute.push_back(filter);
                    }
                    break;
                case StereoDepthConfig::PostProcessing::Filter::TEMPORAL:
                    if(stereoConfig->postProcessing.temporalFilter.enable) {
                        filtersToExecute.push_back(filter);
                    }
                    break;
                case StereoDepthConfig::PostProcessing::Filter::SPATIAL:
                    if(stereoConfig->postProcessing.spatialFilter.enable) {
                        filtersToExecute.push_back(filter);
                    }
                    break;
                case StereoDepthConfig::PostProcessing::Filter::DECIMATION:
                    if(decimationEnabled) {
                        filtersToExecute.push_back(filter);
                    }
                    break;
                case StereoDepthConfig::PostProcessing::Filter::MEDIAN:
                    if(medianFilterEnabled) {
                        filtersToExecute.push_back(filter);
                    }
                    break;
                default:
                    break;
            }
        }

        bool postProcessingEnabled = filtersToExecute.size() > 0;

        if(postProcessingEnabled) {
            logger->debug("Post-processing enabled, executing filters..");

            scaleFactor = (1 << 13) / maxDisparity;

            logger->debug("Scaling disparity by factor: {}", scaleFactor);

            uint16_t* data = (uint16_t*)disparityFrame->data->getData().data();
            for(int i = 0; i < disparityFrame->getWidth() * disparityFrame->getHeight(); i++) {
                data[i] = data[i] * scaleFactor;
            }

            for(auto filter : filtersToExecute) {
                switch(filter) {
                    case StereoDepthConfig::PostProcessing::Filter::SPECKLE: {
                        int maxDiff = stereoConfig->postProcessing.speckleFilter.differenceThreshold * 16;
                        logger->debug("Speckle filter enabled, maxDiff: {}", maxDiff);
                        maxDiff = maxDiff * scaleFactor;
                        logger->debug("Executing speckle filter..");
                        speckleFilter.process(disparityFrame, stereoConfig->postProcessing.speckleFilter.speckleRange, maxDiff);
                    } break;
                    case StereoDepthConfig::PostProcessing::Filter::TEMPORAL: {
                        if(temporalFilterInit == false) {
                            temporalFilterInit = true;
                            auto frameSize = disparityFrame->getWidth() * disparityFrame->getHeight() * sizeof(uint16_t);
                            float alpha = stereoConfig->postProcessing.temporalFilter.alpha;
                            int delta = stereoConfig->postProcessing.temporalFilter.delta;
                            if(delta == 0) {
                                delta = dai::StereoDepthConfig::PostProcessing::TemporalFilter::DEFAULT_DELTA_VALUE * 16;
                            }
                            logger->debug("Temporal filter enabled, alpha: {}, delta: {}", alpha, delta);
                            delta = delta * scaleFactor;

                            temporalFilter.Init(frameSize, alpha, delta, (int)stereoConfig->postProcessing.temporalFilter.persistencyMode);
                        }
                        logger->debug("Executing temporal filter..");

                        // Apply temporal filter
                        uint8_t* currentFrameData = disparityFrame->data->getData().data();
                        size_t frameHeight = disparityFrame->getHeight();
                        size_t frameWidth = disparityFrame->getWidth();

                        temporalFilter.process(currentFrameData, frameHeight, frameWidth);
                    } break;
                    case StereoDepthConfig::PostProcessing::Filter::SPATIAL: {
                        if(spatialFilterInit == false) {
                            spatialFilterInit = true;
                            float alpha = stereoConfig->postProcessing.spatialFilter.alpha;
                            int delta = stereoConfig->postProcessing.spatialFilter.delta;
                            if(delta == 0) {
                                delta = dai::StereoDepthConfig::PostProcessing::SpatialFilter::DEFAULT_DELTA_VALUE * 16;
                            }
                            int iterationNr = stereoConfig->postProcessing.spatialFilter.numIterations;
                            int holeFillingRadius = stereoConfig->postProcessing.spatialFilter.holeFillingRadius;
                            logger->debug("Spatial filter enabled, alpha: {}, delta: {}, iterations: {}, holeFillingRadius: {}",
                                          alpha,
                                          delta,
                                          iterationNr,
                                          holeFillingRadius);
                            delta = delta * scaleFactor;

                            spatialFilter.Init(alpha, delta, iterationNr, holeFillingRadius);
                        }
                        logger->debug("Executing spatial filter..");
                        spatialFilter.process(disparityFrame);
                    } break;
                    case StereoDepthConfig::PostProcessing::Filter::DECIMATION: {
                        if(decimationFilterInit == false) {
                            decimationFilterInit = true;
                            logger->debug("Decimation filter enabled, factor: {}", decimationFactor);
                            decimationFilter.Init(decimationFactor, (DecimationMode)stereoConfig->postProcessing.decimationFilter.decimationMode);
                        }

                        auto postDecimationWidth = ALIGN_UP(disparityFrame->getWidth() / decimationFactor, 16);
                        auto postDecimationHeight = ALIGN_UP(disparityFrame->getHeight() / decimationFactor, decimationFactor);

                        auto decimatedDisparitySize = postDecimationWidth * postDecimationHeight * sizeof(uint16_t);

                        auto decimatedDisparity = decimatedDisparityFramePool->get();
                        decimatedDisparity->setMetadata(*referenceFrame);
                        decimatedDisparity->setWidth(postDecimationWidth);
                        decimatedDisparity->setHeight(postDecimationHeight);
                        decimatedDisparity->setType(dai::ImgFrame::Type::RAW16);
                        decimatedDisparity->fb.stride = decimatedDisparity->fb.width * decimatedDisparity->fb.bytesPP;
                        if(decimatedDisparity->data == nullptr) {
                            logger->error("Stereo allocation error: {}, skipping frame!", __LINE__);
                            continue;
                        }

                        logger->debug("Executing decimation filter..");
                        decimationFilter.process(disparityFrame, decimatedDisparity);
                        disparityFrame = decimatedDisparity;
                        decimationApplied = true;
                    } break;
                    case StereoDepthConfig::PostProcessing::Filter::MEDIAN: {
                        if(stereoConfig->postProcessing.median == MedianFilter::KERNEL_7x7) {
                            stereoConfig->postProcessing.median = MedianFilter::KERNEL_5x5;
                        }
                        int medianSize = (int)stereoConfig->postProcessing.median;
                        logger->debug("Executing median filter..");
                        medianFilter.process(disparityFrame, medianSize);
                    } break;
                    default:
                        break;
                }
            }
        }

        auto tEndPostProcessing = steady_clock::now();
        auto stereoPostProcessingRuntime = duration_cast<microseconds>(tEndPostProcessing - tStartPostProcessing).count() / 1000.f;
        logger->debug("Stereo post-processing took {} ms", stereoPostProcessingRuntime);

        auto stereoTotalRuntime = duration_cast<microseconds>(tEndPostProcessing - tStartSync).count() / 1000.f;
        logger->debug("Stereo (total) took {} ms", stereoTotalRuntime);

        performanceMetrics.tStartPostProcessing = tStartPostProcessing;
        performanceMetrics.tEndPostProcessing = tEndPostProcessing;

        auto tStereoSendStart = steady_clock::now();

        {
            auto blockEvent = this->outputBlockEvent();
            // TODO(Remove this once the pipeline framework supports const shared ptrs)
            rectifiedLeft.send(std::const_pointer_cast<dai::ImgFrame>(rectifiedLeftFrame));
            rectifiedRight.send(std::const_pointer_cast<dai::ImgFrame>(rectifiedRightFrame));
            // pixelDescriptorsLeft.send(stereoDescriptorFrameLeft);
            // pixelDescriptorsRight.send(stereoDescriptorFrameRight);

            logger->debug("Sending data..");

            syncedLeft.send(std::const_pointer_cast<dai::ImgFrame>(leftIn));
            syncedRight.send(std::const_pointer_cast<dai::ImgFrame>(rightIn));
            // debugDispCostDump.send(costDumpFrame);
            // debugDispLrCheckIt1.send(wtaDumpFrame);
            outConfig.send(stereoConfig);

            dai::ImgTransformation alignTransformation;
            switch(effectiveAlignment) {
                case StereoDepthConfig::AlgorithmControl::DepthAlign::LEFT:
                    alignTransformation = leftIn->transformation;
                    break;
                case StereoDepthConfig::AlgorithmControl::DepthAlign::RIGHT:
                    alignTransformation = rightIn->transformation;
                    break;
                case StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT:
                    alignTransformation = rectifiedLeftFrame->transformation;
                    break;
                case StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_RIGHT:
                    alignTransformation = rectifiedRightFrame->transformation;
                    break;
                default:
                    throw std::runtime_error("Unsupported alignment type");
            }
            if(decimationApplied) {
                alignTransformation.addScale(1.0f / decimationFactor, 1.0f / decimationFactor);
            }

            if(disparityConnected) {
                setFrameTransformation(*disparityFrame, alignTransformation);
                disparity.send(disparityFrame);
            }
            if(depthConnected) {
                std::shared_ptr<dai::ImgFrame> depthFrame = depthFramePool->get();
                // depthFrame->set(referenceFrame->get());
                depthFrame->setMetadata(*disparityFrame);
                depthFrame->setType(dai::ImgFrame::Type::RAW16);
                if(depthFrame->data == nullptr) {
                    logger->error("Stereo post-processing allocation error: {}, skipping frame!", __LINE__);
                    continue;
                }

                auto startTimer = high_resolution_clock::now();
                if(runPostProcessingOnCPU) {
                    disparityToDepth(disparityFrame, depthFrame, scaleFactor);
                } else {
                    auto inDisparityFrame = AcceleratorMemory::convert(disparityFrame->data);
                    auto outDepthFrame = AcceleratorMemory::convert(depthFrame->data);

                    if(dspConfig.scaleFactor != scaleFactor || (dspConfig.fixedNumerator != fixedNumerator)) {
                        uint16_t* disp2depthLUT = dspConfig.disp2depthLUT;
                        logger->debug("Generating disparity to depth LUT..");
                        disp2depthLUT[0] = 0;
                        for(int i = 1; i < 65536; i++) {
                            disp2depthLUT[i] = fixedNumerator * 16 * scaleFactor / i;
                        }
                    }
                    dspConfig.width = disparityFrame->getWidth();
                    dspConfig.height = disparityFrame->getHeight();
                    dspConfig.fixedNumerator = fixedNumerator;
                    dspConfig.scaleFactor = scaleFactor;
                    const dai_device_buf dspIn0 = dai::device::toDeviceBuf(*inDisparityFrame), dspConfig = dai::device::toDeviceBuf(*inConfigData),
                                         dspOut = dai::device::toDeviceBuf(*outDepthFrame);
                    if(int err = dai_device_dsp_disparity_to_depth(&dspIn0, &dspConfig, &dspOut)) {
                        logger->error("DSP: Disparity to depth failed with error: {}, {}", hexagon::errorToString(err), __LINE__);
                    }
                }
                auto stopTimer = high_resolution_clock::now();
                logger->debug("Disparity to depth time: {} ms", duration_cast<microseconds>(stopTimer - startTimer).count() / 1000.0f);

                thresholdFilter.process(
                    depthFrame, stereoConfig->postProcessing.thresholdFilter.minRange, stereoConfig->postProcessing.thresholdFilter.maxRange);

                setFrameTransformation(*depthFrame, alignTransformation);
                depth.send(depthFrame);
            }
            if(confidenceMapConnected) {
                confidenceMap.send(confidenceFrame);
            }
        }
        auto tStereoSendEnd = steady_clock::now();
        auto stereoSendTime = duration_cast<microseconds>(tStereoSendEnd - tStereoSendStart).count() / 1000.f;
        logger->debug("Stereo send took {} ms", stereoSendTime);

        this->pipelineEventDispatcher->endTrackedEvent(PipelineEvent::Type::LOOP, "_mainLoop", leftIn->sequenceNum);
    }
}

void StereoDepth::initCalib(const std::shared_ptr<const dai::ImgFrame>& leftIn, const std::shared_ptr<const dai::ImgFrame>& rightIn) {
    std::cout << "Loading calibration data from socket leftSocket " << (int)leftSocket << " and rightSocket " << (int)rightSocket << std::endl;

    auto pipeline = getParentPipeline();
    const bool splineRectification = properties.rectificationModel == dai::StereoDepthProperties::RectificationModel::BSPLINE;

    bool isCalib = false;
    CalibrationHandler calib;

    if(pipeline.isCalibrationDataAvailable()) {
        logger->info("Calibration data available. Reading ...");
        // TODO: This has to be reviewed why causes the excep.
        // terminate called after throwing an instance of 'std::out_of_range'

        isCalib = true;
        calib = pipeline.getCalibrationData();
    } else {
        logger->info("No calibration data available.");

        calib = CalibrationHandler();
        isCalib = false;
        if(enableRectification) {
            logger->warn("No calibration data provided!");
        }
    }

    // bool isVertical = properties.verticalStereo;
    bool isVertical = false;
    int baselineIdx = isVertical ? 1 : 0;

    if(isCalib) {
        try {
            auto T = calib.getCameraTranslationVector(leftSocket, rightSocket, true);
            isVertical = std::abs(T[1]) > std::abs(T[0]);
            baselineIdx = std::abs(T[1]) > std::abs(T[0]);
        } catch(const std::exception& ex) {
        }
    }

    if(!enableRectification) isVertical = false;

    if(isVertical) {
        auto tmp = outFrameWidth;
        outFrameWidth = outFrameHeight;
        outFrameHeight = tmp;
    }

    if(splineRectification && (!isCalib || !enableRectification || isVertical || properties.mesh.meshSize)) {
        throw std::runtime_error("B-spline rectification requires a calibration, enabled horizontal stereo and no custom mesh");
    }
    if(properties.mesh.stepWidth <= 0 || properties.mesh.stepHeight <= 0) throw std::runtime_error("Mesh steps must be positive");
    stepWidth = properties.mesh.stepWidth;
    stepHeight = properties.mesh.stepHeight;
    meshWidth = outFrameWidth / stepWidth + 1;
    meshHeight = outFrameHeight / stepHeight + 1;
    meshElemSize = sizeof(MeshPoint);
    meshSize = meshWidth * meshHeight * sizeof(MeshPoint);
    meshStride = ALIGN_UP(meshWidth * meshElemSize, 16);  // HW constraints, not applied to host data
    meshStrideElems = meshStride / meshElemSize;

    bool userMeshRectification = !!properties.mesh.meshSize;

    autoAlignment = dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RECTIFIED_LEFT;
    if(isCalib) {
        try {
            const auto camATranslationToLeft = calib.getCameraTranslationVector(leftSocket, dai::CameraBoardSocket::CAM_A, false);
            const auto camATranslationToRight = calib.getCameraTranslationVector(rightSocket, dai::CameraBoardSocket::CAM_A, false);
            autoAlignment = decideAutoAlignmentFromCamAProximity(camATranslationToLeft, camATranslationToRight);
        } catch(const std::exception&) {
        }
    }

    const auto requestedAlignment = resolveEffectiveAlignment(properties.initialConfig.algorithmControl.depthAlign, autoAlignment);

    alignedSocket = leftSocket;
    dai::ImgTransformation alignedTransformation = leftIn->transformation;
    dai::ImgTransformation nonAlignedTransformation = rightIn->transformation;
    if(usesRightPrimaryAlignment(requestedAlignment)) {
        alignedSocket = rightSocket;
        alignedTransformation = rightIn->transformation;
        nonAlignedTransformation = leftIn->transformation;
    }

    float calib_H1[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float calib_H2[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    // default
    float baseline = 0.075;
    float fov = 71.86;
    float focal = 0;
    const float customMultiplier = initialConfig->algorithmControl.depthUnit == dai::DepthUnit::CUSTOM
                                       ? initialConfig->algorithmControl.customDepthUnitMultiplier
                                       : dai::getLengthUnitMultiplier(initialConfig->algorithmControl.depthUnit);
    int baselineIndex = isVertical ? 1 : 0;
    fixedNumerator = 0;
    if(depthConnected) {
        if(isCalib) {
            try {
                auto T = calib.getCameraTranslationVector(leftSocket, rightSocket, false);
                baseline = translationMagnitudeMeters(T);
                fov = calib.getFov(alignedSocket, false);

                if(!leftIn || !rightIn) {  // should never trigger
                    throw std::runtime_error("Left or Right ImgFrame is nullptr, cannot initialize calibration.");
                }

                std::array<std::array<float, 3>, 3> intrinsicsMatrix;
                if(rightSocket == alignedSocket) {
                    intrinsicsMatrix = getIntrinsicsWithCalibrationFallback(rightIn, rightSocket, calib, inWarpWidth, inWarpHeight);
                } else if(leftSocket == alignedSocket) {
                    intrinsicsMatrix = getIntrinsicsWithCalibrationFallback(leftIn, leftSocket, calib, inWarpWidth, inWarpHeight);
                } else {
                    throw std::runtime_error("Aligned socket does not match left or right socket.");
                }

                baselineIndex = std::abs(T[0]) > std::abs(T[1]) ? 0 : 1;
                focal = intrinsicsMatrix[baselineIndex][baselineIndex];
                fixedNumerator = baseline * customMultiplier * focal;  // mm

            } catch(const std::exception& ex) {
                logger->warn(
                    "Depth output requested, but there is no extrinsic calibration found to determine baseline and FOV. Defaulting to baseline: '{}' and FOV: "
                    "'{}'",
                    baseline,
                    fov);
                // std::cout << "Error reading calib " << ex.what() << std::endl;
            }
        } else {
            logger->warn(
                "Depth output requested, but there is no extrinsic calibration found to determine baseline and FOV. Defaulting to baseline: '{}' and FOV: '{}'",
                baseline,
                fov);
        }
        logger->debug("Baseline: {}", baseline);
        logger->debug("Fov: {}", fov);
        logger->debug("Focal: {}", focal);
        logger->debug("FixedNumerator: {}", fixedNumerator);
    }

    leftRawGridX.clear();
    leftRawGridY.clear();
    rightRawGridX.clear();
    rightRawGridY.clear();
    map_x_reverse.release();
    map_y_reverse.release();
    cv_R_left.release();
    cv_R_right.release();
    cv_KRect_left.release();
    cv_KRect_right.release();

    if(enableRectification) {
        if(userMeshRectification) {
            if(!usesRectifiedOutputAlignment(requestedAlignment)) {
                throw std::runtime_error("LEFT/RIGHT alignment is not supported together with custom rectification mesh on RVC4");
            }
            meshSize = *properties.mesh.meshSize;
            int expectedMeshSize = meshWidth * meshHeight * meshElemSize;
            if(meshSize != expectedMeshSize) {
                throw std::runtime_error(fmt::format("Mesh size {} not matching resolution {}x{} with step {}x{}, expected {}.",
                                                     meshSize,
                                                     inWarpWidth,
                                                     inWarpHeight,
                                                     properties.mesh.stepWidth,
                                                     properties.mesh.stepHeight,
                                                     expectedMeshSize));
            }

            auto meshResource = loadResource(properties.mesh.meshLeftUri);
            logger->debug("Loaded left mesh size: {}", meshResource.size());
            WarpDispatcher::ParseMeshResourceData(meshResource, meshWidth, meshHeight, leftRawGridX, leftRawGridY, false);

            meshResource = loadResource(properties.mesh.meshRightUri);
            logger->debug("Loaded right mesh size: {}", meshResource.size());
            WarpDispatcher::ParseMeshResourceData(meshResource, meshWidth, meshHeight, rightRawGridX, rightRawGridY, false);

        } else if(isCalib) {
            if(!leftIn || !rightIn) {  // should never trigger
                throw std::runtime_error("Left or Right ImgFrame is nullptr, cannot initialize calibration.");
            }

            auto M1 = getIntrinsicsWithCalibrationFallback(leftIn, leftSocket, calib, inWarpWidth, inWarpHeight);
            auto M2 = getIntrinsicsWithCalibrationFallback(rightIn, rightSocket, calib, inWarpWidth, inWarpHeight);

            auto cv_M1 = vecToCvMat(3, 3, CV_32FC1, matrix::matrix3x3ToVectorMatrix(M1));
            auto cv_M2 = vecToCvMat(3, 3, CV_32FC1, matrix::matrix3x3ToVectorMatrix(M2));

            /**
             * (aljazkonec) Need to robustly handle calibration updates and missing metadata:
             * - ImgTransformations need to save EEPROM ID from which they were created. If EEPROM ID updates in both ImgFrames, recompute the rectification
             * mesh.
             * - (holistic) record & replay can record frames without metadata, which causes the transformations to have empty extrinsics.
             * This is the reason we use calibration handler for getting intrinsics, extrinsics and distortion, instead of relying on the transformations.
             */
            // auto d1 = alignedTransformation.getDistortionCoefficients();
            // auto d2 = nonAlignedTransformation.getDistortionCoefficients();
            auto d1 = calib.getDistortionCoefficients(leftSocket);
            auto d2 = calib.getDistortionCoefficients(rightSocket);

            auto cv_d1 = vecToCvMat(1, d1.size(), CV_32FC1, d1);
            auto cv_d2 = vecToCvMat(1, d2.size(), CV_32FC1, d2);

            // std::vector<std::vector<float>> R = matrix::matrix3x3ToVectorMatrix(alignedTransformation.getRotationMatrixTo(nonAlignedTransformation));
            std::vector<std::vector<float>> R = calib.getCameraRotationMatrix(leftSocket, rightSocket);

            auto cv_R = vecToCvMat(3, 3, CV_32FC1, R);
            // auto T = alignedTransformation.getTranslationVectorTo(nonAlignedTransformation);
            auto T = calib.getCameraTranslationVector(leftSocket, rightSocket, false);
            auto cv_T = vecToCvMat(1, 3, CV_32FC1, {T[0], T[1], T[2]});

            cv::Size imageSize = cv::Size(inWarpWidth, inWarpHeight);
            auto rectification = computeRectificationMatrices(
                cv_M1, cv_d1, cv_M2, cv_d2, imageSize, cv_R, cv_T, cv::Size(outFrameWidth, outFrameHeight), properties.alphaScaling);
            auto cv_meshSize = cv::Size(outFrameWidth, outFrameHeight);
            auto cv_R1 = rectification.R1.clone();
            auto cv_R2 = rectification.R2.clone();
            auto cv_targetCamMatrix = usesRightPrimaryAlignment(requestedAlignment) ? rectification.KRect2.clone() : rectification.KRect1.clone();

            if(isVertical) {
                // TODO rather rotate matrix to avoid floating point errors
                std::vector<std::vector<float>> rot90_cw = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
                auto cv_rot90cw = vecToCvMat(3, 3, CV_32FC1, rot90_cw);
                cv_R1 = cv_R1 * cv_rot90cw;
                cv_R2 = cv_R2 * cv_rot90cw;

                // TODO swap or extract from iamge width/height
                auto* targetCamMatrixData = cv_targetCamMatrix.ptr<float>();
                std::swap(targetCamMatrixData[2], targetCamMatrixData[5]);
                std::swap(targetCamMatrixData[0], targetCamMatrixData[4]);
            }

            if(depthConnected && isCalib) {
                focal = cv_targetCamMatrix.at<float>(baselineIndex, baselineIndex);
                fixedNumerator = baseline * customMultiplier * focal;
                logger->debug("Using rectified focal for depth conversion: {}", focal);
                logger->debug("Updated fixedNumerator after rectification: {}", fixedNumerator);
            }

            cv_R_left = cv_R1.clone();
            cv_R_right = cv_R2.clone();
            cv_KRect_left = cv_targetCamMatrix.clone();
            cv_KRect_right = cv_targetCamMatrix.clone();

            if(!usesRectifiedOutputAlignment(requestedAlignment)) {
                if(requestedAlignment == dai::StereoDepthConfig::AlgorithmControl::DepthAlign::LEFT) {
                    cv::initUndistortRectifyMap(cv_targetCamMatrix, cv::noArray(), cv_R1.inv(), cv_M1, cv_meshSize, CV_32FC1, map_x_reverse, map_y_reverse);
                } else if(requestedAlignment == dai::StereoDepthConfig::AlgorithmControl::DepthAlign::RIGHT) {
                    cv::initUndistortRectifyMap(cv_targetCamMatrix, cv::noArray(), cv_R2.inv(), cv_M2, cv_meshSize, CV_32FC1, map_x_reverse, map_y_reverse);
                }
            }

            if(splineRectification) {
                // Same rectification as legacy; only the lens model maps each rectified ray to a raw pixel
                const auto started = steady_clock::now();
                splineMeshGrid(nativeBsplineLens(calib, leftSocket, *leftIn, inWarpWidth, inWarpHeight), cv_R1, cv_targetCamMatrix, stepWidth, stepHeight, meshWidth, meshHeight, leftRawGridX, leftRawGridY);
                splineMeshGrid(nativeBsplineLens(calib, rightSocket, *rightIn, inWarpWidth, inWarpHeight), cv_R2, cv_targetCamMatrix, stepWidth, stepHeight, meshWidth, meshHeight, rightRawGridX, rightRawGridY);
                logger->info("B-spline rectification mesh {}x{} built in {} ms",
                             meshWidth,
                             meshHeight,
                             duration_cast<microseconds>(steady_clock::now() - started).count() / 1000.0);
            } else {
                cv::initUndistortRectifyMap(cv_M1, cv_d1, cv_R1, cv_targetCamMatrix, cv_meshSize, CV_32FC1, map_x_l, map_y_l);

                auto leftMesh = downSampleMesh(outFrameWidth, outFrameHeight, meshWidth, meshHeight, stepWidth, stepHeight, map_x_l, map_y_l);

                cv::initUndistortRectifyMap(cv_M2, cv_d2, cv_R2, cv_targetCamMatrix, cv_meshSize, CV_32FC1, map_x_r, map_y_r);

                auto rightMesh = downSampleMesh(outFrameWidth, outFrameHeight, meshWidth, meshHeight, stepWidth, stepHeight, map_x_r, map_y_r);

                for(auto& p : leftMesh) {
                    leftRawGridX.push_back(p.x);
                    leftRawGridY.push_back(p.y);
                }

                for(auto& p : rightMesh) {
                    rightRawGridX.push_back(p.x);
                    rightRawGridY.push_back(p.y);
                }
            }

            if(1) {
                cv::Mat cv_H1 = (cv_targetCamMatrix * cv_R1) * cv_M1.inv();
                cv::Mat cv_H2 = (cv_targetCamMatrix * cv_R2) * cv_M2.inv();

                if(!cv_H1.isContinuous()) {
                    cv_H1 = cv_H1.clone();
                }
                if(!cv_H2.isContinuous()) {
                    cv_H2 = cv_H2.clone();
                }

                for(int i = 0; i < 9; i++) {
                    calib_H1[i] = cv_H1.at<float>(i);
                    calib_H2[i] = cv_H2.at<float>(i);
                }
            }

        } else {
            auto cv_meshSize = cv::Size(outFrameWidth, outFrameHeight);

            float dummy_d[4] = {0, 0, 0, 0};
            auto d = cv::Mat(1, 4, CV_32FC1, dummy_d);

            auto identity_R = cv::Mat(cv::Mat::eye(3, 3, CV_32FC1));

            std::vector<std::vector<float>> dummy_M = {{854, 0, inWarpWidth / 2.f}, {0, 854, inWarpHeight / 2.f}, {0, 0, 1}};

            auto M = vecToCvMat(3, 3, CV_32FC1, dummy_M);

            auto targetCamMatrix = M.clone();

            if(isVertical) {
                std::vector<std::vector<float>> rot90_cw = {{0, -1, 0}, {1, 0, 0}, {0, 0, 1}};
                auto cv_rot90cw = vecToCvMat(3, 3, CV_32FC1, rot90_cw);
                identity_R = identity_R * cv_rot90cw;
                // TODO substract or swap?
                std::swap(targetCamMatrix.at<float>(0, 2), targetCamMatrix.at<float>(1, 2));
                std::swap(targetCamMatrix.at<float>(0, 0), targetCamMatrix.at<float>(1, 1));
            }

            cv::Mat map_x, map_y;
            cv::initUndistortRectifyMap(M, d, identity_R, targetCamMatrix, cv_meshSize, CV_32FC1, map_x, map_y);
            auto identityMesh = downSampleMesh(outFrameWidth, outFrameHeight, meshWidth, meshHeight, stepWidth, stepHeight, map_x, map_y);

            for(auto& p : identityMesh) {
                leftRawGridX.push_back(p.x);
                leftRawGridY.push_back(p.y);

                rightRawGridX.push_back(p.x);
                rightRawGridY.push_back(p.y);
            }

            cv_KRect_left = targetCamMatrix.clone();
            cv_KRect_right = targetCamMatrix.clone();

            cv_R_left = identity_R.clone();
            cv_R_right = identity_R.clone();
        }
    }

    // Custom meshes define a virtual camera independently of EEPROM calibration.
    // Apply explicit depth-geometry overrides after generated rectification has
    // selected its focal length, so both paths honor the public SDK API.
    if(depthConnected && (properties.baseline.has_value() || properties.focalLength.has_value())) {
        fixedNumerator = computeDepthNumeratorWithOverrides(baseline, focal, customMultiplier, properties.baseline, properties.focalLength);
        logger->debug("FixedNumerator after depth geometry overrides: {}", fixedNumerator);
    }

    dai::WarpProperties warpConfig;
    warpConfig.outputWidth = outFrameWidth;
    warpConfig.outputHeight = outFrameHeight;
    warpConfig.meshWidth = meshWidth;
    warpConfig.meshHeight = meshHeight;
    warpConfig.interpolation = dai::Interpolation::BILINEAR;

    if(enableRectification) {
        int warpErr;

        if(!warpInitialized) {
            warpErr = warp.Init(warpConfig, outFrameWidth, outFrameHeight, dai::ImgFrame::Type::RAW8, dai::ImgFrame::Type::RAW8);
            if(warpErr != 0) {
                throw std::runtime_error(fmt::format("Failed initializing warp node: {}!", warpErr));
            }
            warpInitialized = true;
        }

        warp.SetWarpMesh(leftRawGridX, leftRawGridY, 0);
        warp.SetWarpMesh(rightRawGridX, rightRawGridY, 1);

        outRectifiedFrameSize = warp.getMinOutImgSize();
    }
}

void StereoDepth::buildStage1() {
    if(properties.initialConfig.filtersBackend != dai::ProcessorType::CPU) {
        // This warms up the dsp so it works afterwards when called.
        // Without this it fails in some cases with:
        // Remote_session_control failed
        // as it has some race conditions with some unknown stuff.
        // Main suspects are the hardware accelerator and fastCV. fastCV and the accelerator already had a race condition,
        // where fastCV had to be configured before the accelerator or the accelerator init would fail.
        //
        // THEORY:
        // As the accelerator init also uses CDSP this is probably the same s**t.
        // fastCV maybe reconfigures / overrides some global CDSP state
        // and that doesn't play nice with other CDSP stuff.
        dai::device::openDspSession(dai_device_dsp_open_stereo, "dai_device_dsp_open_stereo");
    }
}

void StereoDepth::run() {
    using namespace std::chrono;
    logger->info("{} running on {}.", this->getName(), runOnHost() ? "host" : "device");

    if(inputAlignTo.isConnected()) {
        throw std::runtime_error("inputAlignTo is not supported!");
    }

    // customInputPixelDescriptors = properties.customPixelDescriptors;

    disparityConnected = disparity.getConnections().size();
    depthConnected = depth.getConnections().size();
    syncedLeftConnected = syncedLeft.getConnections().size();
    syncedRightConnected = syncedRight.getConnections().size();
    rectifiedLeftConnected = rectifiedLeft.getConnections().size();
    rectifiedRightConnected = rectifiedRight.getConnections().size();
    confidenceMapConnected = confidenceMap.getConnections().size();

    // unsupported
    outConfigConnected = outConfig.getConnections().size();
    debugDispLrCheckIt1Connected = debugDispLrCheckIt1.getConnections().size();
    debugDispLrCheckIt2Connected = debugDispLrCheckIt2.getConnections().size();
    debugExtDispLrCheckIt1Connected = debugExtDispLrCheckIt1.getConnections().size();
    debugExtDispLrCheckIt2Connected = debugExtDispLrCheckIt2.getConnections().size();
    debugDispCostDumpConnected = debugDispCostDump.getConnections().size();

    if(debugDispLrCheckIt1Connected) {
        throw std::runtime_error(fmt::format("'{}' output is not supported!", debugDispLrCheckIt1.toString()));
    }
    if(debugDispLrCheckIt2Connected) {
        throw std::runtime_error(fmt::format("'{}' output is not supported!", debugDispLrCheckIt2.toString()));
    }
    if(debugExtDispLrCheckIt1Connected) {
        throw std::runtime_error(fmt::format("'{}' output is not supported!", debugExtDispLrCheckIt1.toString()));
    }
    if(debugExtDispLrCheckIt2Connected) {
        throw std::runtime_error(fmt::format("'{}' output is not supported!", debugExtDispLrCheckIt2.toString()));
    }
    if(debugDispCostDumpConnected) {
        throw std::runtime_error(fmt::format("'{}' output is not supported!", debugDispCostDump.toString()));
    }

    enableRectification = properties.enableRectification;
    // TODO(jakgra) check why the accelerator unwarp triggers ASAN
    // enableRectification = false;

    // if(customInputPixelDescriptors && enableRectification) {
    //     throw std::runtime_error("Custom input pixel descriptors and on-board rectification are mutually exclusive!");
    // }

    lazyLoad = true;

    // Retrieve message from input
    std::shared_ptr<ImgFrame> leftIn, rightIn;
    {
        auto blockEvent = this->inputBlockEvent();
        leftIn = left.get<dai::ImgFrame>();
        rightIn = right.get<dai::ImgFrame>();
    }

    leftSocket = static_cast<dai::CameraBoardSocket>(leftIn->getInstanceNum());
    rightSocket = static_cast<dai::CameraBoardSocket>(rightIn->getInstanceNum());

    // Frame Size
    uint32_t inFrameWidth = leftIn->getWidth();
    uint32_t inFrameHeight = leftIn->getHeight();

    // Some sanity checks
    if((leftIn->getWidth() != rightIn->getWidth()) || (leftIn->getHeight() != rightIn->getHeight())) {
        throw std::runtime_error(fmt::format("Left and right input frames have different sizes! Left: {}x{}, Right: {}x{}",
                                             leftIn->getWidth(),
                                             leftIn->getHeight(),
                                             rightIn->getWidth(),
                                             rightIn->getHeight()));
    }

    auto supportedTypes = {dai::ImgFrame::Type::RAW8, dai::ImgFrame::Type::GRAY8, dai::ImgFrame::Type::NV12, dai::ImgFrame::Type::YUV420p};
    for(auto& input : {leftIn, rightIn}) {
        // Check if input type is supported
        if(std::find(supportedTypes.begin(), supportedTypes.end(), input->getType()) == supportedTypes.end()) {
            throw std::runtime_error("Input type is not supported! Please use GRAY8 or NV12 input types");
        }

        // Strided inputs not supported yet
        if(input->getStride() != input->getWidth()) {
            throw std::runtime_error(
                fmt::format("Input stride '{}' not being equal to width '{}' is not supported! Please use widths divisible by 128 (1280, 640, ...)",
                            input->getStride(),
                            input->getWidth()));
        }
    }

    uint32_t inputBpp = 1;

    _leftIn = leftIn;
    _rightIn = rightIn;
    leftIn = nullptr;
    rightIn = nullptr;

    if(properties.width && properties.height) {
        inFrameWidth = *properties.width;
        inFrameHeight = *properties.height;
    }

    inWarpWidth = inFrameWidth;
    inWarpHeight = inFrameHeight;
    outFrameWidth = inFrameWidth;
    outFrameHeight = inFrameHeight;

    if(properties.outWidth && properties.outHeight) {
        outFrameWidth = *properties.outWidth;
        outFrameHeight = *properties.outHeight;
    }

    initCalib(_leftIn, _rightIn);

    // if( outFrameWidth > 1280 || outFrameWidth < 64 ||
    //     outFrameHeight < 64 || outFrameHeight > 720) {
    //     throw std::runtime_error(fmt::format("Stereo input resolution {}x{} not supported!", outFrameWidth, outFrameHeight));
    // }

    StereoInit stereoInit = {
        .bConfOutputEnable = true,
        .bSlicOutputEnable = false,
        .nActualFps = 30,
        .nOperationalFps = 30,
        .nWidth = outFrameWidth,
        .nHeight = outFrameHeight,
        .colorFormat = dai::ImgFrame::Type::GRAY8,
    };
    int stereoDepthErr = stereoDepth.Init(stereoInit);
    if(stereoDepthErr) {
        throw std::runtime_error(fmt::format("Failed initializing stereo depth node: {}!", stereoDepthErr));
    }

    outDisparityFrameSize = stereoDepth.getMinDisparityImgSize();
    outConfidenceFrameSize = stereoDepth.getMinConfImgSize();

    latestConfig = std::make_shared<StereoDepthConfig>(properties.initialConfig);

    auto queueSize = left.getMaxSize();
    auto blocking = left.getBlocking();

    if(left.getMaxSize() != right.getMaxSize()) {
        throw std::runtime_error(
            fmt::format("Stereo left and right input must have the same queue size! Left: {} vs right: {}!", left.getMaxSize(), right.getMaxSize()));
    }
    if(left.getBlocking() != right.getBlocking()) {
        throw std::runtime_error(fmt::format(
            "Stereo left and right input must have the same queue blocking behavior! Left: {} vs right: {}!", left.getBlocking(), right.getBlocking()));
    }

    // Initialize the second depth node & pyramid image DS
    // if Extended mode is is enabled
    auto rawCfg = *latestConfig;

    extendedMode = rawCfg.algorithmControl.enableExtended;

    logger->debug("Extended mode is - {}", extendedMode ? "on" : "off");

    if(extendedMode) {
        StereoInit init = {
            .bConfOutputEnable = true,
            .bSlicOutputEnable = false,
            .nActualFps = 30,
            .nOperationalFps = 30,
            .nWidth = outFrameWidth / 2,
            .nHeight = outFrameHeight / 2,
            .colorFormat = dai::ImgFrame::Type::GRAY8,
        };

        int stereoDepthErr = stereoDepthDs.Init(init);
        if(stereoDepthErr) {
            throw std::runtime_error(fmt::format("Failed initializing stereo depth node: {}!", stereoDepthErr));
        }

        PyrImgInit pyrInit = {.bOutBaseimage = false,
                              .nActualFps = 30,
                              .nOperationalFps = 30,
                              .nWidth = outFrameWidth,
                              .nHeight = outFrameHeight,
                              .colorFormat = dai::ImgFrame::Type::GRAY8,
                              .nOctaves = 2,
                              .nScalesPerOctave = 1};

        int pyrImgErr = pyrImg.Init(pyrInit);
        if(pyrImgErr) {
            throw std::runtime_error(fmt::format("Failed initializing pyramid image node: {}!", pyrImgErr));
        }
    }

    warpOut.setMaxSize(queueSize);
    warpOut.setBlocking(blocking);
    descriptorOut.setMaxSize(queueSize);
    descriptorOut.setBlocking(blocking);
    stereoOut.setMaxSize(queueSize);
    stereoOut.setBlocking(blocking);

    auto runAndCatchExceptions = [this](auto func) {
        try {
            (this->*func)();
        } catch(const dai::MessageQueue::QueueException& e) {
            logger->debug("Queue closed - {}", e.what());
            this->threadsRunning = false;
            this->warpOut.destruct();
            this->descriptorOut.destruct();
            this->stereoOut.destruct();
        } catch(const std::exception& e) {
            logger->error("Node threw exception {}", e.what());
            this->threadsRunning = false;
        }
    };

    std::thread threadWarp(runAndCatchExceptions, &StereoDepth::_ThreadWarp);
    std::thread threadStereo(runAndCatchExceptions, &StereoDepth::_ThreadStereo);
    std::thread threadPostProcessing(runAndCatchExceptions, &StereoDepth::_ThreadPostProcessing);

    if(threadWarp.joinable()) threadWarp.join();
    if(threadStereo.joinable()) threadStereo.join();

    while(isRunning()) {
        std::this_thread::sleep_for(1s);
    }

    stereoDepth.Close();

    stereoDepthDs.Close();
    pyrImg.Close();

    warp.Close();
}
void StereoDepth::stop() {
    ThreadedNode::stop();
}

}  // namespace gate
}  // namespace dai
