#include "depthai/pipeline/datatype/Landmarks.hpp"

namespace dai {

#ifdef DEPTHAI_HAVE_OPENCV_SUPPORT
// cv::Mat Landmarks::getCovarianceAsCVMat(Landmark landmark) {
//     std::vector<double> data;
//
//     for(std::array<double, 6>& i : landmark.covariance) {
//         for(double& j : i) {
//             data.push_back(i);
//         }
//     }
//
//
//
//     cv::Mat mat(6, 6, CV_64FC1, (double*)data);
//
//
//
//     return mat;
//
// }
#endif
}
