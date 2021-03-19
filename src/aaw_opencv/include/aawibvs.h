#ifndef AAWIBVS_H
#define AAWIBVS_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace CameraSN11818179 {
    const float   fx_left = 1395.23,
                  fy_left = 1395.17,
                  cx_left = 1139.14,
                  cy_left = 669.406;
    const float  fx_right = 1399.12,
                 fy_right = 1399.31,
                 cx_right = 1095.57,
                 cy_right = 660.776;
    const float focalLengthInPixel = (fx_left + fx_right) / 2;
    const double baseLine = 0.0630536;
    const cv::Point2d desiredCoord_LeftView_NormalizedPlane_0(-0.339764,-0.314982);
    const cv::Point2d desiredCoord_LeftView_NormalizedPlane_1(-0.327863,0.203417);
    const cv::Point2d desiredCoord_LeftView_NormalizedPlane_2(0.725336,0.190165);
    const cv::Point2d desiredCoord_LeftView_NormalizedPlane_3(0.721327,-0.34093);
}

namespace CameraSN16988350 {
    const float   fx_left = 1400.29,
                  fy_left = 1400.15,
                  cx_left = 1154.29,
                  cy_left = 662.777;
    const float  fx_right = 1400.2,
                 fy_right = 1400.19,
                 cx_right = 1087.37,
                 cy_right = 626.596;
    const float focalLengthInPixel = (fx_left + fx_right) / 2;
    const double baseLine = 0.0632291;
}

class AawIBVS
{
public:
    //*************************不同的控制法则尚未实现************
    enum FeatureJacobianType {DESIRED, CURRENT, MEAN};
    enum CameraSerialNumber {SN11818179, SN16988350};
    AawIBVS();
    AawIBVS(CameraSerialNumber SN);
    void updateVertexesCoordinates(std::vector<cv::Point2f> vertexesLeft, std::vector<cv::Point2f> vertexesRight);
    void updateControlLaw();
    Eigen::VectorXd getControlVel() const;

private:
    static const double lambda_;
    static const unsigned int pointsNumber_;
    Eigen::Matrix<float, 3, 3> cameraIntrinsicsMatrix_LeftView, cameraIntrinsicsMatrix_RightView;
    double baseLine_;
    float focalLengthInPixel_;
    std::vector<cv::Point2f> currentVertexes_LeftView_Pixel_,
                             currentVertexes_RightView_Pixel_;
    std::vector<cv::Point2d> desiredPointsCoordinates_LeftView_NormalizedPlane_;
    Eigen::Matrix<double, 6, 1> controlVel_;

    void setCameraIntrinsics(CameraSerialNumber SN);
    void setDesiredCoordinatesOnNormalizedPlane(CameraSerialNumber SN);
    double estimateDepthInCameraFrame(cv::Point2f pointCoordInLeftView, cv::Point2f pointCoordInRightView);
    cv::Point2d getCurrentCoordinatesOnNormalizedPlane(cv::Point2f pointCoordInLeftView);
    Eigen::VectorXd calcErrorVector(std::vector<cv::Point2d> currentPointsCoordinates_LeftView_NormalizedPlane);
    Eigen::MatrixXd calcFeatureJacobianMatrix(std::vector<cv::Point2d> coords_xyOnNP, std::vector<double> depth_ZInCF);
    void calcControlVel(Eigen::MatrixXd featureJacobianMatrix, Eigen::VectorXd errorVector);
};

#endif // AAWIBVS_H
