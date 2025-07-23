#include "depthai/pipeline/datatype/Landmarks.hpp"
#include "iostream"

namespace dai {
cv::Mat Landmark::getCovarianceAsCvMat() {
    //The matrix will always be 6x6 since that is what rtabmap expects.
    cv::Mat cvMat(6, 6, CV_64F);

    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
            cvMat.at<double>(i, j) = covariance.at(i).at(j);
        }
    }

    return cvMat;
}
}
