/*
Benchmark to compare tag pose estimation directly from apriltag library and using solvePnP from opencv.
The criterion is the average reprojection error of corners of each tag.
SolvePnP is shown to work

To use:
cd bin/
./pose_estimation_benchmark <your.png> <list.jpg> <of.jpg> <files.png>


TODO:
- add parameters
- Add a 3rd estimation using solvePnPRansac y fusing several tag detections
- 2D viz with Rviz
*/


#include <stdio.h>
#include <iostream>
#include <ctime>
#include <stdint.h>
#include <inttypes.h>
#include <ctype.h>
#include <unistd.h>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry> 
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgcodecs.hpp"


#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h10.h"
#include <tag36artoolkit.h>
#include "tag25h9.h"
#include "tag25h7.h"
#include "common/image_u8.h"
#include "common/image_u8x4.h"
#include "common/pjpeg.h"
#include "common/homography.h"
#include "common/zarray.h"

#include "ippe.h"

#include "pose_estimation_benchmark.hpp"


int main(int argc, char *argv[]){
    // Define cam intrisics cx, cy, fx, fy
    double cx = 323;
    double cy = 232;
    double fx = 711;
    double fy = 711;
    std::vector<double> k_vec = {cx, cy, fx, fy};
    double tag_width = 0.055;
    int scale = 2;  // to visualize

    // Corners from apriltag are in the following order (anti clockwise, looking at the tag)
    double s = tag_width/2;
    Eigen::Vector3d p1; p1 << -s,  s, 0; // bottom left
    Eigen::Vector3d p2; p2 <<  s,  s, 0; // bottom right
    Eigen::Vector3d p3; p3 <<  s, -s, 0; // top right
    Eigen::Vector3d p4; p4 << -s, -s, 0; // top left
    std::vector<Eigen::Vector3d> tag_X_vec = {p1, p2, p3, p4};

    apriltag_detector_t *detector_at = create_detector();
    for (int i=1; i < argc; i++) {
        // Get Image with opencv
        std::string path = "../images/" + std::string(argv[i]);
        cv::Mat img;
        cv::Mat grayscale_image;
        img = cv::imread(path, CV_LOAD_IMAGE_COLOR);
        if (img.data){std::cout << "\nImage read successfully: " << path << std::endl;}
        else {std::cout << "!! Could not be read: " << path << std::endl; continue;}
        cv::cvtColor(img, grayscale_image, cv::COLOR_BGR2GRAY);
        // Make an image_u8_t header for the Mat data
        image_u8_t imu8gray = {   .width  = grayscale_image.cols,
                                  .height = grayscale_image.rows,
                                  .stride = grayscale_image.cols,
                                  .buf    = grayscale_image.data
                              };       
        
        int nb_mean = 5;

        // std::clock_t t1 = std::clock();
        // for (unsigned int i=0; i < nb_mean; i++){
        //     std::cout << "YO" << std::endl;
        //     apriltag_detector_detect(detector_at, &imu8gray);
        // }
        // std::clock_t dt1 = std::clock() - t1;
        // std::cout << "Mean time for apriltag_detector_detect: " << ((float)dt1/CLOCKS_PER_SEC)/nb_mean << " s" << std::endl;
        // std::cout << "WESH" << std::endl;

        zarray_t *detections = apriltag_detector_detect(detector_at, &imu8gray);
        // std::cout << "YOGZ " << zarray_size(detections) << std::endl;

        for (int i = 0; i < zarray_size(detections); i++) {
            // std::cout << "YE" << std::endl;

            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            std::cout << "Tag id: " << det->id << std::endl; 
            // std::clock_t t2 = std::clock();
            // for (unsigned int i=0; i < nb_mean; i++){
            //     std::cout << "PO" << std::endl;
            //     umich_pose_estimation(det, k_vec);
            // }
            // std::clock_t dt2 = std::clock() - t2;
            // std::cout << "Mean time for apriltag_detector_detect: " << ((float)dt2/CLOCKS_PER_SEC)/nb_mean << " s" << std::endl;

            // std::clock_t t3 = std::clock();
            // for (unsigned int i=0; i < nb_mean; i++){
            //     std::cout << "POOOO" << std::endl;
            //     opencv_pose_estimation(det, k_vec);           
            // }
            // std::clock_t dt3 = std::clock() - t3;
            // std::cout << "Mean time for apriltag_detector_detect: " << ((float)dt3/CLOCKS_PER_SEC)/nb_mean << " s" << std::endl;


            Eigen::Affine3d M_umich = umich_pose_estimation(det, k_vec, tag_width);
            Eigen::Affine3d M_opencv = opencv_pose_estimation(det, k_vec, tag_width); 
            Eigen::Affine3d M_ippe1, M_ippe2;
            float rep_error1, rep_error2;
            ippe_pose_estimation(det, k_vec, tag_width, M_ippe1, rep_error1, M_ippe2, rep_error2); 
                      
            // std::cout << std::endl;            
            // std::cout << "Tag id: " << det->id << std::endl; 
            // std::cout << "M_umich" << std::endl;            
            // std::cout << M_umich.matrix() << std::endl;
            // std::cout << "M_opencv" << std::endl;            
            // std::cout << M_opencv.matrix() << std::endl;
            // std::cout << "M_ippe1" << std::endl;            
            // std::cout << M_ippe1.matrix() << std::endl;
            // std::cout << "M_ippe2" << std::endl;            
            // std::cout << M_ippe2.matrix() << std::endl;

            std::vector<cv::Point2d> reproj_corners_umich  = pinholePix(tag_X_vec, M_umich, k_vec);
            std::vector<cv::Point2d> reproj_corners_opencv = pinholePix(tag_X_vec, M_opencv, k_vec);
            std::vector<cv::Point2d> reproj_corners_ippe1  = pinholePix(tag_X_vec, M_ippe1, k_vec);
            std::vector<cv::Point2d> reproj_corners_ippe2  = pinholePix(tag_X_vec, M_ippe2, k_vec);
            
            // std::cout << std::endl;            
            std::cout << "Tag id: " << det->id << std::endl; 
            // std::cout << "reproj_corners_umich" << std::endl;            
            // std::cout << reproj_corners_umich << std::endl;
            // std::cout << "reproj_corners_opencv" << std::endl;            
            // std::cout << reproj_corners_opencv << std::endl;
            std::cout << "Reprojection error umich : " << reprojection_error(get_corners(det), reproj_corners_umich) << std::endl;
            std::cout << "Reprojection error opencv: " << reprojection_error(get_corners(det), reproj_corners_opencv) << std::endl;
            std::cout << "Reprojection error ippe1: " << reprojection_error(get_corners(det), reproj_corners_ippe1) << std::endl;
            std::cout << "Reprojection error ippe2: " << reprojection_error(get_corners(det), reproj_corners_ippe2) << std::endl;
            std::cout << "rep_error1: " << rep_error1 << std::endl;
            std::cout << "rep_error2: " << rep_error2 << std::endl;

            print_points(img, get_corners(det), cv::viz::Color::blue(), 1, 1);
            // print_points(img, reproj_corners_umich, cv::viz::Color::red(), 1, 1);
            print_points(img, reproj_corners_opencv, cv::viz::Color::yellow(), 1, 1);
            print_points(img, reproj_corners_ippe1, cv::viz::Color::green(), 1, 1);
            // print_points(img, reproj_corners_ippe2, cv::viz::Color::violet(), 1, 1);

        }   
            cv::Size s = img.size()*scale;
            cv::resize(img, img, s, 0, 0, cv::INTER_CUBIC);
            cv::imshow(argv[i], img);
            cv::waitKey(0);
    }

    // Should also free the tag family used...
    apriltag_detector_destroy(detector_at);
}



Eigen::Affine3d umich_pose_estimation(apriltag_detection_t *det, std::vector<double> kvec, double tag_width){
    // To put in the usual camera frame with Z looking in front (RDF)
    Eigen::Affine3d c_M_ac;
    c_M_ac.matrix() = (Eigen::Vector4d() << 1, -1, -1, 1).finished().asDiagonal();

    Eigen::Affine3d M_april_raw;
    matd_t *pose_matrix = homography_to_pose(det->H, -kvec[2], kvec[3], kvec[0], kvec[1]); // !! fx Negative sign advised by apriltag library commentary
    // write it in Eigen form
    Eigen::Affine3d ac_M_t;
    for(int r=0; r<4; r++)
        for(int c=0; c<4; c++)
            ac_M_t.matrix()(r,c) = matd_get(pose_matrix, r, c);
    
    Eigen::Affine3d c_M_t = c_M_ac * ac_M_t;

    // Normalize transform
    c_M_t.matrix().block(0,3,3,1) *= tag_width/2;

    return c_M_t;
}

Eigen::Affine3d opencv_pose_estimation(apriltag_detection_t *det, std::vector<double> kvec, double tag_width){
    std::vector<cv::Point2d> corners_pix = get_corners(det);

    std::vector<cv::Point3d> obj_pts;
    // Same order as the 2D corners (anti clockwise, looking at the tag).
    // Looking at the tag, the reference frame is
    // X = Right, Y = Down, Z = Inside the plane    
    double s = tag_width/2;
    obj_pts.emplace_back(-s,  s, 0); // bottom left
    obj_pts.emplace_back( s,  s, 0); // bottom right
    obj_pts.emplace_back( s, -s, 0); // top right
    obj_pts.emplace_back(-s, -s, 0); // top left

    // Solve for pose
    // The estimated r and t brings points from tag frame to camera frame
    cv::Mat rvec, tvec;
    cv::Mat K = (cv::Mat_<double>(3,3) << kvec[2], 0, kvec[0],
                                          0, kvec[3], kvec[1],
                                          0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming corrected images

    cv::solvePnP(obj_pts, corners_pix, K, dist_coeffs, rvec, tvec);

    // Puts the result in a Eigen affine Transform   
    cv::Matx33d rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_eigen; cv2eigen(rmat, R_eigen);
    Eigen::Vector3d t_eigen; cv2eigen(tvec, t_eigen);
    Eigen::Affine3d M;
    M.matrix().block(0,0,3,3) = R_eigen;
    M.matrix().block(0,3,3,1) = t_eigen;

    return M;
}

void ippe_pose_estimation(apriltag_detection_t *det, std::vector<double> kvec, double tag_width, 
                            Eigen::Affine3d &M1, 
                            float &rep_error1, 
                            Eigen::Affine3d &M2, 
                            float &rep_error2){

    cv::Mat K = (cv::Mat_<double>(3,3) << kvec[2], 0, kvec[0],
                                        0, kvec[3], kvec[1],
                                        0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // Assuming corrected images


    std::vector<cv::Point2d> corners_pix = get_corners(det);

    std::vector<cv::Point3d> obj_pts;
    // Same order as the 2D corners (anti clockwise, looking at the tag).
    // Looking at the tag, the reference frame is
    // X = Right, Y = Down, Z = Inside the plane
    double s = tag_width/2;
    obj_pts.emplace_back(-s,  s, 0); // bottom left
    obj_pts.emplace_back( s,  s, 0); // bottom right
    obj_pts.emplace_back( s, -s, 0); // top right
    obj_pts.emplace_back(-s, -s, 0); // top left

    cv::Mat rvec1, tvec1, rvec2, tvec2;
    IPPE::PoseSolver pose_solver;

        /**
     * @brief                Finds the two possible poses of a planar object given a set of correspondences and their respective reprojection errors. The poses are sorted with the first having the lowest reprojection error.
     * @param _objectPoints  Array of 4 or more coplanar object points defined in object coordinates. 1xN/Nx1 3-channel (float or double) where N is the number of points
     * @param _imagePoints   Array of corresponding image points, 1xN/Nx1 2-channel. This can either be in pixel coordinates or normalized pixel coordinates.
     * @param _cameraMatrix  Intrinsic camera matrix (same definition as OpenCV). If _imagePoints is in normalized pixel coordinates you must set  _cameraMatrix = cv::noArray()
     * @param _distCoeffs    Intrinsic camera distortion vector (same definition as OpenCV). If _imagePoints is in normalized pixel coordinates you must set  _cameraMatrix = cv::noArray()
     * @param _rvec1         First rotation solution (3x1 rotation vector)
     * @param _tvec1         First translation solution (3x1 vector)
     * @param reprojErr1     Reprojection error of first solution
     * @param _rvec2         Second rotation solution (3x1 rotation vector)
     * @param _tvec2         Second translation solution (3x1 vector)
     * @param reprojErr2     Reprojection error of second solution
     */
    pose_solver.solveGeneric(obj_pts, corners_pix, K, dist_coeffs,
                            rvec1, tvec1, rep_error1, 
                            rvec2, tvec2, rep_error2);

    M1 = opencv_pose_to_eigen(rvec1, tvec1);
    M2 = opencv_pose_to_eigen(rvec2, tvec2);
}


apriltag_detector_t *create_detector(){
    // TODO: params in a struct
    apriltag_family_t *tag_family;
    // configure apriltag detector
    std::string famname("tag36h11");
    if (famname == "tag36h11")
        tag_family = tag36h11_create();
    else if (famname == "tag36h10")
        tag_family = tag36h10_create();
    else if (famname == "tag36artoolkit")
        tag_family = tag36artoolkit_create();
    else if (famname == "tag25h9")
        tag_family = tag25h9_create();
    else if (famname == "tag25h7")
        tag_family = tag25h7_create();
    else {
        std::cout << "Argh" << std::endl;
        exit(-1);
    }

    tag_family->black_border    = 1;

    apriltag_detector_t *detector = apriltag_detector_create();
    apriltag_detector_add_family(detector, tag_family);

    detector->quad_decimate     = 0.0;
    detector->quad_sigma        = 2.0;
    detector->nthreads          = 4;
    detector->debug             = false;
    detector->refine_edges      = true;
    detector->refine_decode     = true;
    detector->refine_pose       = true;

    return detector;
}

void normalize_transform(Eigen::Affine3d &M, double tag_width){
    M.matrix().block(0,3,3,1) *= tag_width/2;
}


std::vector<cv::Point2d> pinholePix(const std::vector<Eigen::Vector3d> &t_X_vec, const Eigen::Affine3d &c_M_t, const std::vector<double> &kvec){
    Eigen::Matrix3d K;
    K << kvec[2], 0, kvec[0],
         0, kvec[3], kvec[1],
         0, 0, 1;

    std::vector<cv::Point2d> imgpix_vec;
    for (unsigned int i=0; i < t_X_vec.size(); i++){
        Eigen::Vector3d projected = K * c_M_t * t_X_vec[i];
        imgpix_vec.push_back(projectedToPix(projected));
    }

    return imgpix_vec;
}



cv::Point2d projectedToPix(const Eigen::Vector3d &projected){
    cv::Point2d imgpix = cv::Point2d(projected(0)/projected(2), projected(1)/projected(2));
    return imgpix;
}

void print_points(cv::Mat im_inout, const std::vector<cv::Point2d> &points, const cv::Scalar &color, double radius, double thickness){
    double scale_radius = 1;
    for (unsigned int i=0; i < points.size(); i++){
        cv::circle(im_inout, points[i], radius*scale_radius, color, thickness);
        scale_radius += 2;
    }
}

std::vector<cv::Point2d> get_corners(apriltag_detection_t *det){
    std::vector<cv::Point2d> corners_pix(4);
    for (int i = 0; i < 4; i++)
    {   
        corners_pix[i].x = det->p[i][0];
        corners_pix[i].y = det->p[i][1];
    }

    return corners_pix;
}

double reprojection_error(const std::vector<cv::Point2d> &pvec, const std::vector<cv::Point2d> &rep_pvec){
    float dx, dy;
    float err = 0;
    for (unsigned int i = 0; i < pvec.size(); i++) {
        dx = rep_pvec[i].x - pvec[i].x;
        dy = rep_pvec[i].y - pvec[i].y;
        err += dx * dx + dy * dy;
    }
    err = sqrt(err / (2.0f * pvec.size()));

    return err;
}

Eigen::Affine3d opencv_pose_to_eigen(const cv::Mat &rvec, const cv::Mat &tvec){
    // Puts the result in a Eigen affine Transform   
    cv::Matx33d rmat;
    cv::Rodrigues(rvec, rmat);
    Eigen::Matrix3d R_eigen; cv2eigen(rmat, R_eigen);
    Eigen::Vector3d t_eigen; cv2eigen(tvec, t_eigen);
    Eigen::Affine3d M;
    M.matrix().block(0,0,3,3) = R_eigen;
    M.matrix().block(0,3,3,1) = t_eigen;

    return M;
}