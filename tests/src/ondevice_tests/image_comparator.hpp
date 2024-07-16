#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>

class ImageComparator {
   private:
    bool verticalCut = true;
    cv::Mat bigImg;

   public:
    ImageComparator() = default;

    void showImages(cv::Mat& img1, cv::Mat& img2, double alpha) {
        if(img1.empty() || img2.empty()) return;

        alpha = std::clamp(alpha, 0.0, 1.0);

        assert(img1.rows == img2.rows);
        assert(img1.cols == img2.cols);

        if(alpha > 0.0 && alpha < 1.0) {
            if(verticalCut) {
                int colcut = static_cast<int>(img1.cols * alpha);
                cv::Rect r1(0, 0, colcut, img1.rows);
                cv::Mat ap = img1(r1);
                ap.copyTo(bigImg(r1));

                cv::Rect r2(colcut, 0, img1.cols - colcut, img1.rows);
                cv::Mat bp = img2(r2);
                bp.copyTo(bigImg(r2));
                cv::line(bigImg, {colcut, 0}, {colcut, img1.rows}, cv::Scalar(255, 255, 255), 2, cv::LINE_4);
            } else {
                int rowcut = static_cast<int>(img1.rows * alpha);
                cv::Rect r1(0, 0, img1.cols, rowcut);
                cv::Mat ap = img1(r1);
                ap.copyTo(bigImg(r1));

                cv::Rect r2(0, rowcut, img1.cols, img1.rows - rowcut);
                cv::Mat bp = img2(r2);
                bp.copyTo(bigImg(r2));
                cv::line(bigImg, {0, rowcut}, {img1.cols, rowcut}, cv::Scalar(255, 255, 255), 2, cv::LINE_4);
            }
            cv::imshow("ImageCompare", bigImg);
        }

        if(alpha == 0.0) {
            imshow("ImageCompare", img2);
        }

        if(alpha == 1.0) {
            imshow("ImageCompare", img1);
        }
    }

    // void run(const std::string& path1, const std::string& path2) {
    void run(cv::Mat img1, cv::Mat img2) {
        std::cout << "Key k : Increase clipping value" << std::endl;
        std::cout << "Key j : Decrease clipping value" << std::endl;
        std::cout << "Key d : Change direction of clipping" << std::endl;
        std::cout << "Key l : Continue with next picture" << std::endl;

        /*
        cv::Mat img1 = cv::imread(path1);
        cv::Mat img2 = cv::imread(path2);
        */

        std::string winname = "ImageCompare";
        cv::namedWindow(winname, cv::WINDOW_AUTOSIZE);
        bigImg = cv::Mat::zeros(img1.rows, img1.cols, CV_8UC3);

        double dl = 1.0 / 100.0;
        double alpha = 0.5;
        while(true) {
            showImages(img1, img2, alpha);
            int key = cv::waitKey(0);
            if('d' == key) {
                verticalCut = !verticalCut;
            }
            if('k' == key) {
                alpha += dl;
            }
            if('j' == key) {
                alpha -= dl;
            }

            if('l' == key) {
                break;
            }

            alpha = std::clamp(alpha, 0.0, 1.0);
        }

        cv::destroyWindow(winname);
    }
};
