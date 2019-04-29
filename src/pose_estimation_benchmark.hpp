#include "apriltag.h"
#include <Eigen/Core>
#include <Eigen/Geometry> 


// struct {
//     double cx;
//     double cy;
//     double fx;
//     double fy;
//     double tag_width;
//     int scale;
// } misc_params;

/* 
Create an apriltag detector pointer with default parameters (TODO) 
\return detection pointer
*/
apriltag_detector_t *create_detector();

void ippe_pose_estimation(apriltag_detection_t *det, std::vector<double> kvec, double tag_width, 
                            Eigen::Affine3d &M1, 
                            float &rep_error1, 
                            Eigen::Affine3d &M2, 
                            float &rep_error2);    

/*
Pose estimation using the librapriltag pose retrieval algorithm from the homography matrix (obtained through DLT)
\param det: detection of a tag (from which is extracted the homography matrix)
\param kvec: camera intrinsics vector [cx, cy, fx, fy]
\return estimated pose c_M_t
*/
Eigen::Affine3d umich_pose_estimation(apriltag_detection_t *det, std::vector<double> kvec, double tag_width);

/*
Pose estimation using the opencv solvePnP pose retrieval algorithm from the corners of a tag 
\param det: detection of a tag (from which are extracted the 4 corners)
\param kvec: camera intrinsics vector [cx, cy, fx, fy]
\return estimated pose c_M_t
*/
Eigen::Affine3d opencv_pose_estimation(apriltag_detection_t *det, std::vector<double> kvec, double tag_width);

/*
Normalize the translation part of a transformation (if given in "tag units")
\return normalized transform
*/
void normalize_transform(Eigen::Affine3d &M, double tag_width);

/*
Project points in the target frame to 2D pixels using the pinhole model
\param t_X_vec vector of 3D points in the target reference frame
\param c_M_t transformation mapping points in the target frame to points in the camera frame
\param kvec camera intrinsics vector [cx, cy, fx, fy]
\param tag_width: width of the tag in meters
\return vector of pixels as 2D points 
*/
std::vector<cv::Point2d> pinholePix(const std::vector<Eigen::Vector3d> &t_X_vec, const Eigen::Affine3d &c_M_t, const std::vector<double> &kvec);

/*
Bring homogeneous projected points to the pixel space (euclidianisation)
\param projected point in homogeneous coordinates
\return image point in pixels
*/
cv::Point2d projectedToPix(const Eigen::Vector3d &projected);

/*
Print points as circles on an image for visualisation
\param im_inout image on which will be printed the points
\param points vector of 2D pixels to print
\param color 
\param radius 
\param thickness 
\return image with printed points
*/
void print_points(cv::Mat im_inout, const std::vector<cv::Point2d> &points, const cv::Scalar &color, double radius, double thickness);

/*
Compute mean of the 2D points distance
\param pvec vector of 2D pixels
\param rep_pvec vector of 2D pixels
\return error
*/
double reprojection_error(const std::vector<cv::Point2d> &pvec, const std::vector<cv::Point2d> &rep_pvec);

/*
Extract corners from detection
\param det apriltag detection of a tag
\return vector of 2D pixels
*/
std::vector<cv::Point2d> get_corners(apriltag_detection_t *det);


Eigen::Affine3d opencv_pose_to_eigen(const cv::Mat &rvec, const cv::Mat &tvec);

void rot2vec(cv::InputArray _R, cv::OutputArray _r);
