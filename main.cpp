#include <iostream>
#include "PoseLib/poselib.h"

#include <Eigen/Dense>
#include <string>
#include <vector>

//for the p1p2ll function the 2D lines need to be in the homogeneous representation,
// instead of point + direction. See the code below

int main(int argc, char** argv) {
    // the point correspondence
    Eigen::Vector3d xp(0.0, -0.1, 1.0);
    Eigen::Vector3d Xp(10.0, 50.0, 100.0);

// bearing vector for point needs to be normalized!
    xp.normalize();

// and two line 2D - 3D correspondence
    Eigen::Vector3d x1(0.0, -0.1, 1.0);
    Eigen::Vector3d v1(1.0, 0.0, 0.0);
    Eigen::Vector3d x2(0.0, 0.1, 1.0);
    Eigen::Vector3d v2(1.0, 0.0, 0.0);


// we convert the 2d lines to homogeneous representation
    Eigen::Vector3d l1 = x1.cross(v1);
    Eigen::Vector3d l2 = x2.cross(v2);
    //print l1 and l2
    std::cout << "l1 = " << l1.transpose() << "\n";
    std::cout << "l2 = " << l2.transpose() << "\n";

// 3D lines
    Eigen::Vector3d X1(0.0, 10.0, 100.0);
    Eigen::Vector3d V1(1.0, 0.0, 0.0);
    Eigen::Vector3d X2(0.0, 30.0, 110.0);
    Eigen::Vector3d V2(1.0, 0.02, 0.01);

    std::vector<poselib::CameraPose> poses;

    int n_sols = poselib::p1p2ll({ xp }, { Xp }, {l1, l2}, { X1, X2 }, { V1, V2 }, &poses);

    // print the results
    std::cout << "Found " << n_sols << " solutions\n";

    for (poselib::CameraPose& pose : poses) {
        std::cout << "R = \n" << pose.R() << "\n t = " << pose.t.transpose() << "\n";

        double err_p = (xp.hnormalized() - (pose.R() * Xp + pose.t).hnormalized()).norm();

        double err_l1_p = std::abs(l1.dot(pose.R() * X1 + pose.t));
        double err_l1_v = std::abs(l1.dot(pose.R() * V1));
        double err_l2_p = std::abs(l2.dot(pose.R() * X2 + pose.t));
        double err_l2_v = std::abs(l2.dot(pose.R() * V2));

        std::cout << "err = " << err_p << " "
                  << err_l1_p << " " << err_l1_v << " "
                  << err_l2_p << " " << err_l2_v << "\n";

        //recover the solution

    }

    //define three 3D lines
    Eigen::Vector3d X3(0.5, 1.0, 10.0);
    Eigen::Vector3d V3(1.0, 0.4, 0.0);
    Eigen::Vector3d X4(0.7, 3.0, 11.0);
    Eigen::Vector3d V4(1.0, 0.2, 0.01);
    Eigen::Vector3d X5(0.4, 3.2, 12.0);
    Eigen::Vector3d V5(1.0, 0.02, 0.01);
    //camera intrinsics
    double focal_length = 1000.0;
    double width = 640.0;
    double height = 480.0;
    double cx = width / 2.0;
    double cy = height / 2.0;
    //projection 3D lines to camera image plane
    //define 2d lines on normalized image plane
    Eigen::Vector3d x3(0.0, 0.1, 1.0);
    Eigen::Vector3d v3(1.0, 0.2, 0.0);
    Eigen::Vector3d x4(0.0, 0.2, 1.0);
    Eigen::Vector3d v4(1.0, 0.3, 0.0);
    Eigen::Vector3d x5(0.0, 0.3, 1.0);
    Eigen::Vector3d v5(1.0, 0.5, 0.0);
    //convert 2d lines to homogeneous representation
    Eigen::Vector3d l3 = x3.cross(v3);
    Eigen::Vector3d l4 = x4.cross(v4);
    Eigen::Vector3d l5 = x5.cross(v5);
    //print l3, l4, l5
    std::cout << "l3 = " << l3.transpose() << "\n";
    std::cout << "l4 = " << l4.transpose() << "\n";
    std::cout << "l5 = " << l5.transpose() << "\n";
    //solve for the camera pose poses3
    std::vector<poselib::CameraPose> poses3;
    int n_sols3 =poselib::p3ll({l3, l4, l5}, {X3, X4, X5}, {V3, V4, V5}, &poses3);
    //print the results
    std::cout << "Found " << n_sols3 << " solutions\n";
    //find the best solution
    double best_err = std::numeric_limits<double>::max();
    poselib::CameraPose best_pose;
    for (poselib::CameraPose& pose : poses3) {
        std::cout << "R = \n" << pose.R() << "\n t = " << pose.t.transpose() << "\n";
        double err_l3_p = std::abs(l3.dot(pose.R() * X3 + pose.t));
        double err_l3_v = std::abs(l3.dot(pose.R() * V3));
        double err_l4_p = std::abs(l4.dot(pose.R() * X4 + pose.t));
        double err_l4_v = std::abs(l4.dot(pose.R() * V4));
        double err_l5_p = std::abs(l5.dot(pose.R() * X5 + pose.t));
        double err_l5_v = std::abs(l5.dot(pose.R() * V5));
        double err = err_l3_p + err_l3_v + err_l4_p + err_l4_v + err_l5_p + err_l5_v;
        std::cout << "err = " << err << "\n";
        if (err < best_err) {
            best_err = err;
            best_pose = pose;
        }
    }
    //print the best solution
    std::cout << "best R = \n" << best_pose.R() << "\n t = " << best_pose.t.transpose() << "\n";

    return 0;
}
