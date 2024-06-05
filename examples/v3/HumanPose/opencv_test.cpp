// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <opencv2/opencv.hpp>

#include "xtensor/xarray.hpp"
#include "xtensor/xio.hpp"
#include "xtensor/xview.hpp"

void showFrame(const cv::Mat& frame) {
    cv::imshow("frame", frame);
    cv::waitKey(0);
}

int main() {
    // xt::xarray of shape 32, 57
    xt::xarray<float> temp = xt::zeros<float>({2, 3});
    // set corner values to 1
    temp(0, 0) = 1;

    // display it
    std::cout << temp << std::endl;

    //  turn it into a cv::Mat
    cv::Mat temp_cv = cv::Mat(temp.shape()[0], temp.shape()[1], CV_32F, temp.data());

    // resize it
    cv::Mat temp_resized;
    cv::resize(temp_cv, temp_resized, cv::Size(4, 6));

    // display it
    std::cout << temp_resized << std::endl;





    // cout the version of OpenCV
    std::cout << "OpenCV version : " << cv::getVersionString() << std::endl;

    // Create a cv::Mat than is all zeros but one circle in the middle
    cv::Mat frame = cv::Mat::zeros(32, 56, CV_8UC1);
    cv::circle(frame, cv::Point(56/4, 32/2), 5, cv::Scalar(200), -1);
    cv::circle(frame, cv::Point(2*56/4, 32/2), 5, cv::Scalar(200), -1);
    cv::circle(frame, cv::Point(3*56/8, 32/4), 3, cv::Scalar(200), -1);
    cv::circle(frame, cv::Point(3*56/8, 3*32/4), 3, cv::Scalar(200), -1);

    cv::circle(frame, cv::Point(3*56/4, 32/2), 5, cv::Scalar(40), -1); 

    // add a bit of gaussian noise
    cv::Mat noise = cv::Mat(frame.size(), CV_8UC1);
    cv::randn(noise, 0, 10);
    cv::add(frame, noise, frame);

    // make values float between 0 and 1
    frame.convertTo(frame, CV_32F, 1.0/255.0);



    // Show the frame
    showFrame(frame);

    // Resize the frame, without interpolation
    // ! I am not sure this is the default behavior in the python code
    cv::Mat frame_resized = cv::Mat::zeros(256, 456, CV_32F);
    cv::resize(frame, frame_resized, frame_resized.size(), 0, 0, cv::INTER_NEAREST); 

    // Show the frame
    showFrame(frame_resized);

    // Smooth using GaussianBlur
    // ! This works even though it is not very visible
    cv::Mat frame_smoothed = cv::Mat::zeros(frame.size(), CV_32F);
    cv::GaussianBlur(frame_resized, frame_smoothed, cv::Size(5, 5), 0, 0);

    // Show the frame
    showFrame(frame_smoothed);

    // Threshold the frame
    // The less visible one goes away and the other one stays and becomes filled in
    float threshold = 0.3;
    cv::Mat frame_thresholded = cv::Mat::zeros(frame_smoothed.size(), CV_32F);
    cv::threshold(frame_smoothed, frame_thresholded, threshold, 1.0, cv::THRESH_BINARY);

    // Show the frame
    showFrame(frame_thresholded);

    // Find contours
    // turn into CV_8UC1
    frame_thresholded.convertTo(frame_thresholded, CV_8UC1, 255);
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(frame_thresholded, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Draw contours in red
    cv::Mat frame_contours = cv::Mat::zeros(frame_thresholded.size(), CV_32F);
    cv::drawContours(frame_contours, contours, -1, cv::Scalar(1.0), 1);
    showFrame(frame_contours);

    // loop through contours 
    for (int i = 0; i < contours.size(); i++) {
        cv::Mat blobMask = cv::Mat::zeros(frame_resized.size(), CV_32F);
        cv::fillPoly(blobMask, contours[i], cv::Scalar(1.0));
        cv::Mat maskedFrame = frame_resized.mul(blobMask);
        // find the highest value in the masked frame
        double minVal, maxVal;
        cv::Point minLoc, maxLoc;
        cv::minMaxLoc(maskedFrame, &minVal, &maxVal, &minLoc, &maxLoc);
        // draw a red circle around the highest value RED
        cv::circle(maskedFrame, maxLoc, 5, cv::Scalar(0.5), -1);

        // Show the frame
        showFrame(maskedFrame);
        
    }
}