//
// Created by dinhnambkhn on 30/08/2023.
//

#include <iostream>
#include "PoseLib/poselib.h"

#include <Eigen/Dense>
#include <vector>
#include <chrono>

//opencv
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
//calid3d
#include <opencv2/calib3d.hpp>

int main()
{
    //read the image VF6_front_.png with original size
    cv::Mat img = cv::imread("../front_cam.jpg", cv::IMREAD_COLOR);
    //VF6_rear_.png
    cv::Mat img_rear = cv::imread("../rear_cam.jpg", cv::IMREAD_COLOR);
    //success
    if (img.empty() || img_rear.empty())
    {
        std::cout << "Could not read the image: " << std::endl;
        return 1;
    }
    //img size
    std::cout << "img size = " << img.size() << "\n";

    //cv::Matx33d K(432.7390364738057, 0, 476.0614994349778, 0, 431.2395555913084, 288.7602152621297, 0, 0, 1);
    //std::vector<double> dist_coeff = { -0.2852754904152874, 0.1016466459919075, -0.0004420196146339175, 0.0001149909868437517, -0.01803978785585194 };

    //intrinsic matrix opencv
    //distortion_parameters:
//   k1: 0.138281
//   k2: 0.025172
//   p1: -0.030963
//   p2: 0.005019
//projection_parameters:
//   fx: 304.007121
//   fy: 304.078429
//   cx: 638.469054
//   cy: 399.956311

    cv::Matx33d K(304.007121, 0, 638.469054, 0, 304.078429, 399.956311, 0, 0, 1);
    //print K and distCoeffs
    std::cout << "K = " << K << "\n";
    //cofficient of distortion k1, k2, p1, p2
    std::vector<double> distCoeffs = { 0.138281, 0.025172, -0.030963, 0.005019 };
    //undistort
    cv::Mat img_undistort;
    cv::Mat img_rear_undistor;
    //new camera matrix
    cv::Mat K_new;
    //estimateNewCameraMatrixForUndistortRectify
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(K, distCoeffs, img.size(), cv::Matx33d::eye(), K_new, 1);
    //undistort
    cv::fisheye::undistortImage(img, img_undistort, K, distCoeffs, K_new);
    cv::fisheye::undistortImage(img_rear, img_rear_undistor, K, distCoeffs, K_new);
    //save img_undistort
    cv::imwrite("../front_un.png", img_undistort);
    cv::imwrite("../rear_un.png", img_rear_undistor);
    //print K_new
    std::cout << "K_new = " << K_new << "\n";
    //show img and img_undistort on same window
    cv::Mat img_show;
    cv::hconcat(img, img_undistort, img_show);
    cv::imshow("img and img_undistort", img_show);
    //wait for key press
    cv::waitKey(0);


}
