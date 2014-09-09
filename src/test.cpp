#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>


Eigen::Vector3d projectPixel(Eigen::Vector2d pixel, Eigen::Vector3d normPlane, double offsetPlane,
                             Eigen::Transform<double,3,Eigen::Affine> toWorld,
                             Eigen::Matrix3d intristic,
                             Eigen::Matrix<double,5,1> cameraCalibration){

    Eigen::Vector2d prediction((pixel(0) - intristic(0,2)) / intristic(0,0),
            (pixel(1) - intristic(1,2)) / intristic(1,1));
    Eigen::Vector2d delta;
    double r2, k_radial, error = 1;
    while(error > 0.001){
        r2 = prediction.dot(prediction);
        k_radial = 1 + cameraCalibration(0) * r2 + cameraCalibration(1) * r2 * r2 + cameraCalibration(4) * r2 * r2 * r2;
        delta << 2 * cameraCalibration(2) * prediction(0) * prediction(1) + cameraCalibration(3) * (r2 + 2 * prediction(0) * prediction(0)),
                2 * cameraCalibration(3) * prediction(0) * prediction(1) + cameraCalibration(2) * (r2 + 2 * prediction(1) * prediction(1));
        pixel = (prediction - delta) * k_radial;
        error = (pixel - prediction).dot(pixel - prediction);
        prediction = pixel;
    }

    Eigen::Vector4d center_in_world = toWorld * Eigen::Vector4d(0,0,0,1);
    Eigen::Vector4d pixel_in_world = toWorld * Eigen::Vector4d(pixel(0),pixel(1),1,1);
    std::cout<< "Center:" << std::endl << center_in_world << std::endl;
    std::cout<< "Pixel:" << std::endl << pixel_in_world << std::endl;
    Eigen::Vector4d normPlane4d(normPlane(0),normPlane(1),normPlane(2),0);
    double t = - (offsetPlane + normPlane4d.dot(center_in_world)) / normPlane4d.dot(pixel_in_world - center_in_world);
    std::cout<< "t:" << t << std::endl;
    Eigen::Vector4d result(center_in_world + t * (pixel_in_world - center_in_world));
    return Eigen::Vector3d(result(0),result(1),result(2));
}


int main() {
    Eigen::Vector2d pixel(0,0);
    Eigen::Vector3d normPlane(0,0,1);
    double offsetPlane = 0;
    Eigen::Matrix3d intristic;
    intristic << 700, 0, 320,
                 0, 700, 220,
                 0, 0, 1;
    Eigen::Matrix<double,5,1> cameraCalibration;
    cameraCalibration << 0.0182389759532889, 0.0520276742502367, 0.00651075732801101, 0.000183496184521575, 0;
    Eigen::Transform<double,3,Eigen::Affine> toWorld(Eigen::Translation3d(1,1,1));
    std::cout<< projectPixel(pixel, normPlane, offsetPlane, toWorld, intristic, cameraCalibration) << std::endl;
}
