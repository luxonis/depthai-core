#pragma once

#include "opencv2/opencv.hpp"

cv::Mat toMat(const std::vector<uint8_t>& data, int w, int h , int numPlanes, int bpp);
void toPlanar(cv::Mat& bgr, std::vector<std::uint8_t>& data);
cv::Mat resizeKeepAspectRatio(const cv::Mat &input, const cv::Size &dstSize, const cv::Scalar &bgcolor);
int createDirectory(std::string directory);
