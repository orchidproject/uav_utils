#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

/**
 * Projects pixel coordinates to a given plane in world coordinates.
 * @param[in] inPixel 2-D vector containing (x,y) pixel coordinates.
 * @param[in] normPlane 3-D vector normal to the projection plane.
 * @param[in] offsetPlane offset of plane from origin (of world frame) in direction of normPlane
 * @param[in] toWorld transformation from camera frame to world frame.
 * @param[in] intristic matrix of the camera
 * @param[in] distortionCoeffs distortion coefficients of camera (e.g. as supplied by ROS calibration package).
 * @returns 3-D vector specifying position of pixel coordinates projected onto the plane.
 */
Eigen::Vector3d projectPixel(const Eigen::Vector2d& inPixel,
                             const Eigen::Vector3d& normPlane,
                             const double offsetPlane,
                             const Eigen::Transform<double,3,Eigen::Affine>& toWorld,
                             const Eigen::Matrix3d& intristic,
                             const Eigen::Matrix<double,5,1>& distortionCoeffs){

    //**************************************************************************
    //   Convert pixel coordinates in to camera frame coordinates
    //**************************************************************************
    double xCoordInCameraFrame = (inPixel(0) - intristic(0,2)) / intristic(0,0);
    double yCoordInCameraFrame = (inPixel(1) - intristic(1,2)) / intristic(1,1);
    Eigen::Vector2d oldPixel(xCoordInCameraFrame,yCoordInCameraFrame);

    //**************************************************************************
    //   Here we implement an iterative method for removing camera distortion
    //   from the camera frame coordinates.
    //   See following URL for explanation:
    //   http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    //**************************************************************************
    Eigen::Vector2d delta;
    Eigen::Vector2d newPixel;
    double r2, k_radial, error = 1;
    while(error > 0.001){
        r2 = oldPixel.dot(oldPixel);
        k_radial = 1 + distortionCoeffs(0) * r2 + distortionCoeffs(1) * r2 * r2 + distortionCoeffs(4) * r2 * r2 * r2;
        delta << 2 * distortionCoeffs(2) * oldPixel(0) * oldPixel(1) + distortionCoeffs(3) * (r2 + 2 * oldPixel(0) * oldPixel(0)),
                2 * distortionCoeffs(3) * oldPixel(0) * oldPixel(1) + distortionCoeffs(2) * (r2 + 2 * oldPixel(1) * oldPixel(1));
        newPixel = (oldPixel - delta) * k_radial;
        error = (newPixel - oldPixel).dot(newPixel - oldPixel);
        oldPixel = newPixel;
    }

    //**************************************************************************
    //   Project pixel coordinates and camera frame origin to world frame 
    //**************************************************************************
    Eigen::Vector4d center_in_world = toWorld * Eigen::Vector4d(0,0,0,1);
    Eigen::Vector4d pixel_in_world = toWorld * Eigen::Vector4d(newPixel(0),newPixel(1),1,1);
    std::cout<< "Center:" << std::endl << center_in_world << std::endl;
    std::cout<< "Pixel:" << std::endl << pixel_in_world << std::endl;

    //**************************************************************************
    //   Intersect Ray with projection plane, and return result
    //**************************************************************************
    Eigen::Vector4d normPlane4d(normPlane(0),normPlane(1),normPlane(2),0);
    double t = - (offsetPlane + normPlane4d.dot(center_in_world)) / normPlane4d.dot(pixel_in_world - center_in_world);
    std::cout<< "t:" << t << std::endl;
    Eigen::Vector4d result(center_in_world + t * (pixel_in_world - center_in_world));
    return result.head<3>();

} // projectPixel


int main() {
    Eigen::Vector2d pixel(0,0);
    Eigen::Vector3d normPlane(0,0,1);
    double offsetPlane = 0;
    Eigen::Matrix3d intristic;
    intristic << 700, 0, 320,
                 0, 700, 220,
                 0, 0, 1;
    Eigen::Matrix<double,5,1> distortionCoeffs;
    distortionCoeffs << 0.0182389759532889, 0.0520276742502367, 0.00651075732801101, 0.000183496184521575, 0;
    Eigen::Transform<double,3,Eigen::Affine> toWorld(Eigen::Translation3d(1,1,1));
    std::cout<< projectPixel(pixel, normPlane, offsetPlane, toWorld, intristic, distortionCoeffs) << std::endl;
}
