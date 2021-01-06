#include "depthai/depthai_wrapper.hpp"
#include "depthai/device.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <opencv2/core/eigen.hpp>
#include <unordered_map>
#include <chrono>

using CV_mat_ptr = std::shared_ptr<cv::Mat>;

class DepthAlignment {

public:
    DepthAlignment(std::vector<std::vector<float>> src_intrinsics,
        std::vector<std::vector<float>> dest_intrinsics,
        std::vector<std::vector<float>> rectified_homo,
        std::vector<std::vector<float>> rot,
        std::vector<float> trans,
        int width, int height)
        : height_(height)
        , width_(width)
    {
        Eigen::Matrix3f temp, rotation, mat_scale;
        float scale_width = (float)1280/1920;
        // float array_val[] = {scale_width, 0., 0., 0., scale_width, 0., 0., 0., 1};
        mat_scale << scale_width,   0.,     0.,
                     0.,        scale_width, 0.,
                     0.,            0.,      1;
        // mat_scale(Eigen::Map<Eigen::Matrix3f>(array_val));
        
        // suggest using map from this link on device side (https://eigen.tuxfamily.org/dox/classEigen_1_1Map.html) src_intrinsics_(Eigen::Map<Eigen::Matrix3f>(src_intrinsics.data()))
        for(int i = 0; i < 3; ++i){
            temp.row(i) = Eigen::Vector3f::Map(&rectified_homo[i][0],rectified_homo[i].size());
            rotation.row(i) = Eigen::Vector3f::Map(&rot[i][0],rot[i].size());
            src_intrinsics_.row(i) = Eigen::Vector3f::Map(&src_intrinsics[i][0],src_intrinsics[i].size());
            dest_intrinsics_.row(i) = Eigen::Vector3f::Map(&dest_intrinsics[i][0],dest_intrinsics[i].size());
        }
        
        std::cout << "mat_scale ->" << mat_scale << std::endl;
        std::cout << "dest_intrinsics_ ->" << dest_intrinsics_ << std::endl;
        std::cout << "src_intrinsics_ ->" << src_intrinsics_ << std::endl;
        
        dest_intrinsics_ = mat_scale * dest_intrinsics_;
        inter_homo_ = src_intrinsics_.inverse() * temp.inverse(); 
        transformation_matrix_ = Eigen::Matrix4f::Identity();
        transformation_matrix_.block<3, 3>(0, 0) = rotation.inverse();
        transformation_matrix_.block<3, 1>(0, 3) << -trans[0], -trans[1], -trans[2];

        // Initializing image pointers
        int depthmap_col = width_;
        int depthmap_row = height_;
        int total_pixels = depthmap_col * depthmap_row;

        // creating column index vector : resulting in a row vector (1,total_pixels)
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> col_idx_flat_row_vec = Eigen::RowVectorXf::LinSpaced(depthmap_col, 0, depthmap_col - 1).replicate(depthmap_row, 1);
        col_idx_flat_row_vec.resize(1, total_pixels);

        // creating row index vector : resulting in a row vector (1,total_pixels)
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_idx_flat_row_vec = Eigen::VectorXf::LinSpaced(depthmap_row, 0, depthmap_row - 1).replicate(1, depthmap_col);
        row_idx_flat_row_vec.resize(1, total_pixels);

        // creating row matrix filled with ones : resulting in a row vector (1,total_pixels)
        auto one_flat_row_vec = Eigen::MatrixXf::Ones(1, total_pixels);


        points_.resize(3, total_pixels);
        std::cout << "Jereee" <<std::endl;
        points_.row(0) = col_idx_flat_row_vec;
        points_.row(1) = row_idx_flat_row_vec;
        points_.row(2) = one_flat_row_vec;

        std::cout << "There" <<std::endl;
        points_ = inter_homo_ * points_;

    }

    Eigen::Matrix3Xf DepthmapToPointCloud(const cv::Mat& depthmap)
    {
        // getting depth value inside a 2D depth map as a row vector (1,total_pixels)
        Eigen::MatrixXf depth_flat_row_vec = ConvertDepthmapToEigenRowMatrix(depthmap);

        // TODO(sachin): since we can do multiply only the block. may be I can exploit this to have part of this computation only once
        return points_ * depth_flat_row_vec.asDiagonal();
    }

    Eigen::Matrix3Xf TransformPointCloud(const Eigen::Matrix3Xf& pointcloud)
    {
        // convert (3, total_pixel)[x, y, z] -> (4, total_pixel)[x, y, z, w]
        Eigen::Matrix4Xf pcl_dest = transformation_matrix_ * pointcloud.colwise().homogeneous();
        return pcl_dest.block(0, 0, 3, pcl_dest.cols()); 
        // Transform the point cloud in right camera frame to rgb camera frame
    }

    Eigen::Matrix3Xf PointCloudToDepthmap(const Eigen::Matrix3Xf& pointcloud)
    {
  
        Eigen::Matrix3Xf pixel_idxs = dest_intrinsics_ * pointcloud;
      
        // std::cout << "IsRowMajor?: " << pixel_idxs.IsRowMajor << std::endl;
        pixel_idxs.block(0, 0, 2, pixel_idxs.cols()) = pixel_idxs.colwise().hnormalized();        
        // Transform the point cloud in right camera frame to rgb camera frame
        pixel_idxs.block(2, 0, 1, pixel_idxs.cols()) = pointcloud.block(2, 0, 1, pointcloud.cols());
        
        return pixel_idxs;
    }

    cv::Mat DepthmapPtsToCVImage(const Eigen::Matrix3Xf& pixel_pts)
    {
        cv::Mat depthmap(height_, width_, CV_16UC1, cvScalar(std::numeric_limits<uint16_t>::max()));
        for (int i = 0; i < pixel_pts.cols(); i++) {
            Eigen::Vector3f item = pixel_pts.col(i);
            int val = item(2) * 10;
            if ((0 <= item(0) && item(0) < width_) && (0 <= item(1) && item(1) < height_) && val < 40000) {
                depthmap.at<ushort>(item(1), item(0)) = val;
            }
        }
        return depthmap;
    }

    Eigen::MatrixXf ConvertDepthmapToEigenRowMatrix(const cv::Mat& depthmap)
    {
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mtrx;
        cv::cv2eigen(depthmap, mtrx);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_mtrx = mtrx;

        row_mtrx.resize(1, depthmap.rows * depthmap.cols);

        // mm to cm conversion
        row_mtrx /= 10.0;

        return row_mtrx;
    }

private:
    // camera's intrinsics
    Eigen::Matrix3f src_intrinsics_, dest_intrinsics_, inter_homo_;
    Eigen::Matrix4f transformation_matrix_;
    int height_, width_;
    Eigen::Matrix3Xf points_;

};

int
main()
{
    Eigen::setNbThreads(8);
    std::unordered_map<std::string, CV_mat_ptr> output_streams_;
    std::list<std::shared_ptr<NNetPacket>> op_NNet_detections;
    std::string config_file_path_("/home/sachin/Desktop/luxonis/rgb_alignment/depthai-core/example/config_1.json");
    DepthAI::DepthAI oak("", config_file_path_, false);
    // cv::Mat depth, color;

    DepthAlignment rgb_align(oak.get_right_intrinsic(), 
                             oak.get_intrinsic(CameraControl::CamId::RGB),
                             oak.get_right_homography(), 
                             oak.get_rgb_rotation(), 
                             oak.get_rgb_translation(),
                             1280,720);

    while (true) {
        std::cout << "Req frames" << std::endl;
        oak.get_streams(output_streams_);

        std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();

        Eigen::Matrix3Xf pcl_right = rgb_align.DepthmapToPointCloud(*output_streams_["depth"]);

        // std::cout << "transforming pcl to rgb frame" << std::endl;
        Eigen::Matrix3Xf pcl_rgb = rgb_align.TransformPointCloud(pcl_right);


        // std::cout << "neext -> pcl to depth map" << std::endl;
        Eigen::Matrix3Xf depth_pts = rgb_align.PointCloudToDepthmap(pcl_rgb);

        // std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
        std::cout << "Size of pcl" << pcl_rgb.rows() << " x " << pcl_rgb.cols() <<   std::endl;

        cv::Mat depth_rgb = rgb_align.DepthmapPtsToCVImage(depth_pts);
        // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
        
        
        std::cout << "Converting to color map " << depth_rgb.rows << " x " << depth_rgb.cols << std::endl;
        for (int i = 0; i < depth_rgb.rows; i++)
        {
            for (int j = 0; j < depth_rgb.cols; j++)
            {
                depth_rgb.at<uint16_t>(i, j) = static_cast<uint16_t>(
                    std::numeric_limits<uint16_t>::max() / depth_rgb.at<uint16_t>(i, j));
            }
        }
        depth_rgb.convertTo(depth_rgb, CV_8UC1);
        cv::Mat im_color;
        cv::applyColorMap(depth_rgb, im_color, cv::COLORMAP_HOT);
        
        cv::Mat overlay_dest;
        // cv::addWeighted(*output_streams_["color"], 0.6, im_color, 0.3, 0.0, overlay_dest);
        
        cv::imshow("colored overlap view", im_color);
        cv::imshow("right view", *output_streams_["right"]);
        cv::imshow("depth view", *output_streams_["depth"]);
        // cv::imshow("color view", *output_streams_["color"]);

        cv::waitKey(1);
        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::cout << "Time difference PointCloudToDepthmap= " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
      
    }


    return 0;
}