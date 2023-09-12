//
// Created by dinhnambkhn on 12/09/2023.
//
#include <iostream>
#include "PoseLib/poselib.h"

#include <Eigen/Dense>
#include <vector>
#include <chrono>

//camera OpenCVCameraModel: fx = , fy, cx, cy, k1, k2, p1, p2
int main()
{
    //list of points in 3D
    std::vector<poselib::Point3D> points3D;
    points3D.emplace_back(2.0, -2.0, 0.0);//A
    points3D.emplace_back(2.0, -1.0, 0.0);//B
    points3D.emplace_back(2.0, 1.0, 0.0);//C
    points3D.emplace_back(2.0, 2.0, 0.0);//D

    points3D.emplace_back(-2.0,2.0, 0.0);//E
    points3D.emplace_back(-2.0,1.0, 0.0);//F
    points3D.emplace_back(-2.0,-1.0, 0.0);//G
    points3D.emplace_back(-2.0,-2.0, 0.0);//H

    //camera model
    poselib::Camera camera;
    camera.model_id = 1;
    camera.width = 1280;
    camera.height = 800;
    // set params to 304.007121, 304.078429, 638.469054, 399.956311, 0.138281, 0.025172, -0.030963, 0.005019;
    camera.params = {304.007121, 304.078429, 638.469054, 399.956311, 0.138281, 0.025172, -0.030963, 0.005019};

    //Rotation matrix from roll, pitch, yaw
    Eigen::Vector3d rpy;
    rpy << M_PI_2+0.2, 0.05, M_PI_2-0.1; //roll, pitch, yaw from world to camera
    //convert to quaternion
    Eigen::Quaterniond q = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());
    //print roll, pitch, yaw from q
    Eigen::Vector3d rpy_q = q.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "rpy_q = " << rpy_q.transpose() << "\n";
    //print quaternion
    std::cout << "q = " << q.coeffs().transpose() << "\n";
    //translation vector
    Eigen::Vector3d t;
    t << -4.0, -0.0, 0.5;
    //camera pose
    poselib::CameraPose camera_pose{q.toRotationMatrix(), t};
    //print camera pose
    std::cout << "world to cam: quat = " << camera_pose.q.transpose() << "\n t = " << camera_pose.t.transpose() << "\n";

    //projection of points to 2D image
    std::vector<poselib::Point2D> points2D;
    for (auto &point3D : points3D) {
        poselib::Point2D point2D;
        //project point3D to unit plane
        Eigen::Vector2d point2D_unit;
        //transform point3D from world to camera coordinate system: T_c = R_cw*T_w + t_cw
        Eigen::Vector3d point3D_camera = camera_pose.R()*point3D + camera_pose.t;

        point2D_unit << point3D_camera[0]/point3D_camera[2], point3D_camera[1]/point3D_camera[2];
        //project point2D_unit to image plane
        camera.project(point2D_unit, &point2D);
        //add noise to point2D randomly from 1-20
        auto random = (double)rand() / RAND_MAX;
        point2D[0] += 5*random;
        point2D[1] += 5*random;
        points2D.push_back(point2D);
        //print point2D
        std::cout << "point2D = " << point2D.transpose() << "\n";
    }
    //make three 3D lines from three 3D points
    std::vector<poselib::Line3D> lines3D;
    //line from H to A
    Eigen::Vector3d V_HA = points3D[0] - points3D[7];
    Eigen::Vector3d X_HA = points3D[7];
    //G to B
    Eigen::Vector3d V_GB = points3D[1] - points3D[6];
    Eigen::Vector3d X_GB = points3D[6];
    //F to C
    Eigen::Vector3d V_FC = points3D[2] - points3D[5];
    Eigen::Vector3d X_FC = points3D[5];
    //E to D
    Eigen::Vector3d V_ED = points3D[3] - points3D[4];
    Eigen::Vector3d X_ED = points3D[4];
    //push back to lines3D
    lines3D.emplace_back(X_HA, V_HA);
    lines3D.emplace_back(X_GB, V_GB);
    lines3D.emplace_back(X_FC, V_FC);
    lines3D.emplace_back(X_ED, V_ED);
    //make three 2D lines from three 2D points
    std::vector<poselib::Line2D> lines2D;
    //line from H to A
    lines2D.emplace_back(points2D[0], points2D[7]);
    //G to B
    lines2D.emplace_back(points2D[1], points2D[6]);
    //F to C
    lines2D.emplace_back(points2D[2], points2D[5]);
    //E to D
    lines2D.emplace_back(points2D[3], points2D[4]);


    //estimate absolute pose using LO-RANSAC followed by non-linear refinement
    poselib::RansacOptions ransac_opt;
    poselib::BundleOptions bundle_opt;

    ransac_opt.max_reproj_error = 10.0;
    ransac_opt.max_iterations = 300.0;
    bundle_opt.verbose = true;
    //show ransac_opt
    std::cout << "ransac_opt:\n";
    std::cout << "max_iterations = " << ransac_opt.max_iterations << "\n";
    std::cout << "min_iterations = " << ransac_opt.min_iterations << "\n";
    std::cout << "max_reproj_error = " << ransac_opt.max_reproj_error << "\n";
    std::cout << "max_epipolar_error = " << ransac_opt.max_epipolar_error << "\n";
    std::cout << "success_prob = " << ransac_opt.success_prob << "\n";
    std::cout << "progressive_sampling = " << ransac_opt.progressive_sampling << "\n";

    poselib::CameraPose pose;
    std::vector<char> inliers;
    //start time
    //only take use point B, C
    std::vector<poselib::Point3D> points3D_new;
    points3D_new.push_back(points3D[0]);
    points3D_new.push_back(points3D[1]);

    //point2D B, C
    std::vector<poselib::Point2D> points2D_new;
    points2D_new.push_back(points2D[0]);
    points2D_new.push_back(points2D[1]);
    //print points3D and points2D
    std::cout << "points3D:\n";
    for (auto &point3D : points3D) {
        std::cout << point3D.transpose() << "\n";
    }
    std::cout << "points2D:\n";
    for (auto &point2D : points2D) {
        std::cout << point2D.transpose() << "\n";
    }
    //start time

    auto start = std::chrono::high_resolution_clock::now();
    poselib::RansacStats stats = poselib::estimate_absolute_pose_pnpl(points2D_new, points3D_new, lines2D, lines3D, camera, ransac_opt, bundle_opt, &pose, &inliers, &inliers);
    //stop time
    auto stop = std::chrono::high_resolution_clock::now();
    //duration in us
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "duration = " << duration.count() << " us\n";

    //--------------------------------
    //pose ground truth
    std::cout << "pose ground truth:\n";
    std::cout << "pose t = " << camera_pose.t.transpose() << "\n";
    //pose R to quat
    Eigen::Quaterniond q_posegt(camera_pose.R()); // world to camera
    std::cout << "pose q = " << q_posegt.coeffs().transpose() << "\n";
    //----------------------------------
    std::cout << "--------------SOLVER---------------\n";
    //print the results
    std::cout << "iterations: " << stats.iterations << " \n";
    std::cout << "inlier_ratio: " << stats.inlier_ratio << " \n";
    std::cout << "model_score: " << stats.model_score << " \n";
    //time
    std::cout << "duration: " << duration.count() << " us\n";
    //pose t
    std::cout << "pose t = " << pose.t.transpose() << "\n";
    //pose R to quat
    Eigen::Quaterniond q_pose(pose.R()); // world to camera
    std::cout << "pose q = " << q_pose.coeffs().transpose() << "\n";
    //convert camera pose to world pose
    Eigen::Quaterniond q_world = q_pose.conjugate();
    Eigen::Vector3d t_world = -q_world.toRotationMatrix()*pose.t;
    //print world pose
    std::cout << "world to pose: quat = " << q_world<< "\n t = " << t_world << "\n";
    //----------------------------------
    std::cout << "-----------------------------\n";
    //compute the translation error
    double err_t = (camera_pose.t - pose.t).norm();
    std::cout << "err_t = " << err_t << "\n";
    //compute the rotation error
    double err_q = std::acos(std::abs(q_posegt.dot(q_pose)));
    std::cout << "err_q = " << err_q << "\n";
    //err_q to degree
    double err_q_deg = err_q*180.0/M_PI;
    std::cout << "err_q_deg = " << err_q_deg << "\n";

    return 0;

}

