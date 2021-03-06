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
Eigen::Vector3d projectPixel(const Eigen::Matrix<double,2,Eigen::Dynamic>& inPixel,
                             const Eigen::Vector3d& normPlane,
                             const double offsetPlane,
                             const Eigen::Transform<double,3,Eigen::Affine>& toWorld,
                             const Eigen::Matrix3d& intristic,
                             const Eigen::Matrix<double,5,1>& distortionCoeffs){

    //**************************************************************************
    //   Convert pixel coordinates in to camera frame coordinates
    //**************************************************************************
    double xCoordInCameraFrame = (inPixel(0,0) - intristic(0,2)) / intristic(0,0);
    double yCoordInCameraFrame = (inPixel(1,0) - intristic(1,2)) / intristic(1,1);
    Eigen::Vector2d oldPixel(xCoordInCameraFrame,yCoordInCameraFrame);

    //std::cout<<"Start:"<<std::endl<<oldPixel<<std::endl;

    //**************************************************************************
    //   Here we implement an iterative method for removing camera distortion
    //   from the camera frame coordinates.
    //   See following URL for explanation:
    //   http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    //**************************************************************************
    Eigen::Vector2d delta;
    Eigen::Vector2d newPixel;
    double r2, k_radial, error = 1;
    do {
        r2 = oldPixel.dot(oldPixel);
        k_radial = 1 + distortionCoeffs(0) * r2 + distortionCoeffs(1) * r2 * r2 + distortionCoeffs(4) * r2 * r2 * r2;
        delta << 2 * distortionCoeffs(2) * oldPixel(0) * oldPixel(1) + distortionCoeffs(3) * (r2 + 2 * oldPixel(0) * oldPixel(0)),
                2 * distortionCoeffs(3) * oldPixel(0) * oldPixel(1) + distortionCoeffs(2) * (r2 + 2 * oldPixel(1) * oldPixel(1));
        newPixel = (oldPixel - delta) * k_radial;
        error = (newPixel - oldPixel).dot(newPixel - oldPixel);
        oldPixel = newPixel;

        //std::cout<<"Radial:"<<std::endl<<k_radial<<std::endl;
        //std::cout<<"tangental:"<<std::endl<<delta<<std::endl;
        //std::cout<<"New:"<<std::endl<<newPixel<<std::endl;
        //std::cout<<"Errors:"<<std::endl<<error<<std::endl;

    } while(error > 0.001);

    //std::cout<<"Final:"<<std::endl<<newPixel<<std::endl;

    //**************************************************************************
    //   Project pixel coordinates and camera frame origin to world frame 
    //**************************************************************************
    Eigen::Vector3d center_in_world = (toWorld * Eigen::Vector4d(0,0,0,1)).head<3>();
    Eigen::Vector3d pixel_in_world = (toWorld * Eigen::Vector4d(newPixel(0),newPixel(1),1,1)).head<3>();
    //**************************************************************************
    //   Intersect Ray with projection plane, and return result
    //**************************************************************************
    double t = - (offsetPlane + normPlane.dot(center_in_world)) / normPlane.dot(pixel_in_world - center_in_world);

    //std::cout<< "Center:" << std::endl << center_in_world << std::endl;
    //std::cout<< "Pixel:" << std::endl << pixel_in_world << std::endl;
    //std::cout<< "t:" << t << std::endl;
    //std::cout<<" Result: " << center_in_world + t * (pixel_in_world - center_in_world) << std::endl;

    return center_in_world + t * (pixel_in_world - center_in_world);
} // projectPixel

Eigen::Matrix<double,3,Eigen::Dynamic> projectPixels(const Eigen::Matrix<double,2,Eigen::Dynamic>& inPixel,
                             const Eigen::Vector3d& normPlane,
                             const double offsetPlane,
                             const Eigen::Transform<double,3,Eigen::Affine>& toWorld,
                             const Eigen::Matrix3d& intristic,
                             const Eigen::Matrix<double,5,1>& distortionCoeffs){
    //**************************************************************************
    //   Convert pixel coordinates in to camera frame coordinates
    //**************************************************************************
    int n = inPixel.cols();
    Eigen::Array<double,2,Eigen::Dynamic> coordInCameraFrame = (inPixel.colwise() - intristic.block<2,1>(0,2)).array().colwise() / intristic.diagonal().head<2>().array();

    //std::cout<<"Start:"<<std::endl<<coordInCameraFrame<<std::endl;

    //**************************************************************************
    //   Here we implement an iterative method for removing camera distortion
    //   from the camera frame coordinates.
    //   See following URL for explanation:
    //   http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html
    //**************************************************************************
    Eigen::Array<double,2,Eigen::Dynamic> tangentalDistortion(2,n);
    Eigen::Array<double,2,Eigen::Dynamic> newCoordinates(2,n);
    Eigen::Array<double,1,Eigen::Dynamic> squaredR(1,n);
    Eigen::Array<double,1,Eigen::Dynamic> radialDistortion(1,n);
    Eigen::Array<double,1,Eigen::Dynamic> errors(1,n);
    do{
        squaredR = coordInCameraFrame.square().colwise().sum();
        radialDistortion = (distortionCoeffs(0) * squaredR + distortionCoeffs(1) * squaredR * squaredR +
                distortionCoeffs(4) * squaredR * squaredR * squaredR) + 1;
        tangentalDistortion << 2 * distortionCoeffs(2) * coordInCameraFrame.row(0) * coordInCameraFrame.row(1) +
                 distortionCoeffs(3) * (squaredR + 2 * coordInCameraFrame.row(0).square()),
                             2 * distortionCoeffs(3) * coordInCameraFrame.row(0) * coordInCameraFrame.row(1) +
                 distortionCoeffs(2) * (squaredR + 2 * coordInCameraFrame.row(1).square());
        newCoordinates = (coordInCameraFrame - tangentalDistortion).rowwise() * radialDistortion;
        errors = (newCoordinates - coordInCameraFrame).square().colwise().sum();
        coordInCameraFrame = newCoordinates;

        //std::cout<<"Radial:"<<std::endl<<radial_distort<<std::endl;
        //std::cout<<"Tangental:"<<std::endl<<tangental_distort<<std::endl;
        //std::cout<<"New:"<<std::endl<<newCoordinates<<std::endl;
        //std::cout<<"Errors:"<<std::endl<<errors<<std::endl;

    } while(errors.maxCoeff() > 0.001);

    //std::cout<<"Final:"<<std::endl<<newCoordinates<<std::endl;

    //**************************************************************************
    //   Project pixel coordinates and camera frame origin to world frame
    //   NOTE:
    //   The pixels calculated from the above procedure give us an (x,y)
    //   coordiante in the camera frame, positioned on the plane with z=1
    //   thus to apply the transform, which works with 4-D vectors, we need
    //   to add two additional rows of 1's to the newCoordiantes matrix
    //   pixelsExtended does exactly this
    //**************************************************************************
    Eigen::Vector3d centerInWorld = (toWorld * Eigen::Vector4d(0,0,0,1)).head<3>();
    Eigen::Matrix<double,4,Eigen::Dynamic> pixelsExtended = Eigen::MatrixXd::Constant(4,n,1);
    pixelsExtended.topRows<2>() = newCoordinates;
    Eigen::Matrix<double,3,Eigen::Dynamic> pixelsInWorld = (toWorld.matrix() * pixelsExtended).topRows<3>();
    //**************************************************************************
    //   Intersect Rays with the projection plane, and return result
    //**************************************************************************
    Eigen::Array<double,1,Eigen::Dynamic> ts = -(offsetPlane + normPlane.dot(centerInWorld)) / (normPlane.transpose()*(pixelsInWorld.colwise() - centerInWorld)).array();

    //std::cout<<((pixelsInWorld.colwise() - centerInWorld).array().colwise() * normPlane.array()).colwise().sum().array()<<std::endl;
    //std::cout<<normPlane.transpose()*(pixelsInWorld.colwise() - centerInWorld)<<std::endl;
    //std::cout<<-(offsetPlane + normPlane.dot(centerInWorld)) * Eigen::MatrixXd::Constant(1,n,1).array() / (normPlane.transpose()*(pixelsInWorld.colwise() - centerInWorld)).array()<<std::endl;
    //std::cout<< "Center:" << std::endl << centerInWorld << std::endl;
    //std::cout<< "Pixels:" << std::endl << pixelsInWorld << std::endl;
    //std::cout<< "t:" << std::endl << ts << std::endl;
    //std::cout<<" Result: " << ((pixelsInWorld.colwise() - centerInWorld).array().rowwise() * ts.array()).colwise() + centerInWorld.array()<< std::endl;

    return ((pixelsInWorld.colwise() - centerInWorld).array().rowwise() * ts).colwise() + centerInWorld.array();
} //projectPixels

int main() {
    Eigen::Array<double,2,Eigen::Dynamic> a(2,3);
    a << 1, 2, 3, 3 ,4, 5;
    std::cout<< a.square().colwise().sum() << std::endl;
    std::cout<< a*a <<std::endl;
    Eigen::Vector2d pixel(0,0);
    Eigen::Vector2d pixel2(638,439);
    Eigen::MatrixXd pixels(2,2);
    pixels << pixel,pixel2;
    Eigen::Vector3d normPlane(0,0,1);
    double offsetPlane = 0;
    Eigen::Matrix3d intristic;
    intristic << 700, 0, 320,
                 0, 700, 220,
                 0, 0, 1;
    Eigen::Matrix<double,5,1> distortionCoeffs;
    distortionCoeffs << 0.0182389759532889, 0.0520276742502367, 0.00651075732801101, 0.000183496184521575, 0;
    Eigen::Transform<double,3,Eigen::Affine> toWorld(Eigen::Translation3d(2,2,2)*Eigen::AngleAxisd(3*M_PI/4, Eigen::Vector3d::UnitX()));
    std::cout<<"Pixel1:"<<std::endl<< projectPixel(pixel, normPlane, offsetPlane, toWorld, intristic, distortionCoeffs) << std::endl;
    std::cout<<"Pixel2:"<<std::endl<< projectPixel(pixel2, normPlane, offsetPlane, toWorld, intristic, distortionCoeffs) << std::endl;
    std::cout<<"Both:"<<std::endl<< projectPixels(pixels, normPlane, offsetPlane, toWorld, intristic, distortionCoeffs) << std::endl;
}
