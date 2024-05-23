#include "depthai/basalt/BasaltVIO.hpp"

namespace dai {
namespace node {
void BasaltVIO::build() {
    vioConfig.load(configPath);
    inputImu.setMaxSize(0);
    inputStereo.setMaxSize(0);
    inputImu.setBlocking(false);
    inputStereo.setBlocking(false);
    inputImu.addCallback(std::bind(&BasaltVIO::imuCB, this, std::placeholders::_1));
    inputStereo.addCallback(std::bind(&BasaltVIO::stereoCB, this, std::placeholders::_1));
}

void BasaltVIO::run() {
    basalt::PoseVelBiasState<double>::Ptr data;
    while(isRunning()) {
        if(!calibrated) continue;
        outStateQueue.pop(data);

        if(!data.get()) continue;
        basalt::PoseState<double>::SE3 finalPose = (data->T_w_i * calib->T_i_c[0]);

        auto trans = finalPose.translation();
        auto rot = finalPose.unit_quaternion();
        auto out = std::make_shared<dai::TransformData>(trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z(), rot.w());
        transform.send(out);
        passthrough.send(leftImg);
    }
}

void BasaltVIO::stereoCB(std::shared_ptr<dai::ADatatype> images) {
    auto group = std::dynamic_pointer_cast<dai::MessageGroup>(images);
    if(!calibrated) {
        std::vector<std::shared_ptr<dai::ImgFrame>> imgFrames;
        for(auto& msg : *group) {
                imgFrames.emplace_back(std::dynamic_pointer_cast<dai::ImgFrame>(msg.second));
        }

        initialize(imgFrames);
    }
    int i = 0;
    basalt::OpticalFlowInput::Ptr data(new basalt::OpticalFlowInput(2));
    for(auto& msg : *group) {
        std::shared_ptr<dai::ImgFrame> imgFrame = std::dynamic_pointer_cast<dai::ImgFrame>(msg.second);
        if(i == 0) {
            leftImg = imgFrame;
        };
        auto t = imgFrame->getTimestamp();
        int64_t t_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();
        auto exposure = imgFrame->getExposureTime();

        int exposure_ms = std::chrono::duration_cast<std::chrono::milliseconds>(exposure).count();
        data->img_data[i].img.reset(new basalt::ManagedImage<uint16_t>(imgFrame->getWidth(), imgFrame->getHeight()));
        data->t_ns = t_ns;
        data->img_data[i].exposure = exposure_ms;
        size_t fullSize = imgFrame->getWidth() * imgFrame->getHeight();
        const uint8_t* dataIN = imgFrame->getData().data();
        uint16_t* data_out = data->img_data[i].img->ptr;
        for(size_t j = 0; j < fullSize; j++) {
            int val = dataIN[j];
            val = val << 8;
            data_out[j] = val;
        }
        i++;
    }
    lastImgData = data;
    if(imageDataQueue) {
        imageDataQueue->push(data);
    }
};

void BasaltVIO::imuCB(std::shared_ptr<dai::ADatatype> imuData) {
    auto imuPackets = std::dynamic_pointer_cast<dai::IMUData>(imuData);

    for(auto& imuPacket : imuPackets->packets) {
        basalt::ImuData<double>::Ptr data;
        data.reset(new basalt::ImuData<double>);
        auto t = imuPacket.acceleroMeter.getTimestamp();
        int64_t t_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(t).time_since_epoch().count();

        data->t_ns = t_ns;
        data->accel = Eigen::Vector3d(imuPacket.acceleroMeter.x, imuPacket.acceleroMeter.y, imuPacket.acceleroMeter.z);
        data->gyro = Eigen::Vector3d(imuPacket.gyroscope.x, imuPacket.gyroscope.y, imuPacket.gyroscope.z);
        if(imuDataQueue) imuDataQueue->push(data);
    }
};
void BasaltVIO::initialize(std::vector<std::shared_ptr<dai::ImgFrame>> frames) {
    auto pipeline = getParentPipeline();
    using Scalar = double;
    calib.reset(new basalt::Calibration<Scalar>);
    calib->imu_update_rate = imuUpdateRate;

    auto calibHandler = pipeline.getDefaultDevice()->readCalibration();

    for(const auto& frame : frames) {
        Eigen::Vector2i resolution;
        resolution << frame->getWidth(), frame->getHeight();
        calib->resolution.push_back(resolution);
        auto camID = static_cast<dai::CameraBoardSocket>(frame->getInstanceNum());
        // imu extrinsics
        std::vector<std::vector<float>> imuExtr = calibHandler.getImuToCameraExtrinsics(camID, true);

        // print out extrinsics
        for(auto& row : imuExtr) {
            for(auto& val : row) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
        Eigen::Matrix<Scalar, 3, 3> R;
        R << imuExtr[0][0], imuExtr[0][1], imuExtr[0][2],
        imuExtr[1][0], imuExtr[1][1], imuExtr[1][2],
        imuExtr[2][0], imuExtr[2][1], imuExtr[2][2];
        Eigen::Quaterniond q(R);

        Eigen::Vector3d trans(imuExtr[0][3]*0.01, imuExtr[1][3]*0.01, imuExtr[2][3]*0.01); // y -x z
        basalt::Calibration<Scalar>::SE3 T_i_c(q,trans);
        calib->T_i_c.push_back(T_i_c);

        // camera intrinsics
        auto intrinsics = calibHandler.getCameraIntrinsics(camID, frame->getWidth(), frame->getHeight());
        auto model = calibHandler.getDistortionModel(camID);
        auto distCoeffs = calibHandler.getDistortionCoefficients(camID);
        basalt::GenericCamera<Scalar> camera;
        if(model == CameraModel::Perspective) {
            basalt::PinholeRadtan8Camera<Scalar>::VecN params;
            // fx, fy, cx, cy
            double fx = intrinsics[0][0];
            double fy = intrinsics[1][1];
            double cx = intrinsics[0][2];
            double cy = intrinsics[1][2];
            double k1 = distCoeffs[0];
            double k2 = distCoeffs[1];
            double p1 = distCoeffs[2];
            double p2 = distCoeffs[3];
            double k3 = distCoeffs[4];
            double k4 = distCoeffs[5];
            double k5 = distCoeffs[6];
            double k6 = distCoeffs[7];
            params << fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6;
            basalt::PinholeRadtan8Camera<Scalar> pinhole(params);
            camera.variant = pinhole;
        } else if(model == CameraModel::Fisheye) {
            // fx, fy, cx, cy
            double fx = intrinsics[0][0];
            double fy = intrinsics[1][1];
            double cx = intrinsics[0][2];
            double cy = intrinsics[1][2];
            double k1 = distCoeffs[0];
            double k2 = distCoeffs[1];
            double k3 = distCoeffs[2];
            double k4 = distCoeffs[3];
            basalt::KannalaBrandtCamera4<Scalar>::VecN params;
            params << fx, fy, cx, cy, k1, k2, k3, k4;
            basalt::KannalaBrandtCamera4<Scalar> kannala(params);
            camera.variant = kannala;
        } else {
            throw std::runtime_error("Unknown distortion model");
        }
        calib->intrinsics.push_back(camera);
    }
    optFlowPtr = basalt::OpticalFlowFactory::getOpticalFlow(vioConfig, *calib);
    optFlowPtr->start();
    imageDataQueue = &optFlowPtr->input_img_queue;
    vio = basalt::VioEstimatorFactory::getVioEstimator(vioConfig, *calib, basalt::constants::g, true, true);
    vio->initialize(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    imuDataQueue = &vio->imu_data_queue;
    optFlowPtr->output_queue = &vio->vision_data_queue;
    vio->out_state_queue = &outStateQueue;
    vio->opt_flow_depth_guess_queue = &optFlowPtr->input_depth_queue;
    vio->opt_flow_state_queue = &optFlowPtr->input_state_queue;
    calibrated = true;
}
}  // namespace node
}  // namespace dai