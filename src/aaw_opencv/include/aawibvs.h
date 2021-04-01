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
    const float baseLine = 0.0630536;

    //----------------------Need to be measured!!!----------------------------------------
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_0(-0.339764,-0.314982);
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_1(-0.327863,0.203417);
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_2(0.725336,0.190165);
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_3(0.721327,-0.34093);
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
    const float baseLine = 0.0632291;
}

class AAWIBVS
{
public:
    //*************************不同的控制法则尚未实现************
    enum FeatureJacobianType {DESIRED, CURRENT, MEAN};
    enum CameraSerialNumber {SN11818179, SN16988350};
    AAWIBVS();
    AAWIBVS(CameraSerialNumber SN);
    void updateVertexesCoordinates(std::vector<cv::Point2f> vertexesLeft, std::vector<cv::Point2f> vertexesRight);
    Eigen::Matrix<float, 6, 1> getCamCtrlVel();
    void measureDesiredCoordsOnNP();

private:
    static const float lambda_;
    static const unsigned int pointsNumber_;
    static const unsigned int desiredCoordsAccumMaxTimes_;

    Eigen::Matrix<float, 3, 3> cameraIntrinsicsMatrix_LeftView, cameraIntrinsicsMatrix_RightView;
    float baseLine_;
    float focalLengthInPixel_;
    std::vector<cv::Point2f> currentVertexes_LeftView_Pixel_,
                             currentVertexes_RightView_Pixel_;
    std::vector<cv::Point2f> desiredPointsCoordinates_LeftView_NormalizedPlane_;
    Eigen::Matrix<float, 6, 1> camCtrlVel_;
    std::vector<cv::Point2f> desiredCoordsOnNP_sum_;
    unsigned int desiredCoordsAccumCount_;

    void initIBVS(CameraSerialNumber SN);
    void setCameraIntrinsics(CameraSerialNumber SN);
    void setDesiredCoordinatesOnNormalizedPlane(CameraSerialNumber SN);
    void updateControlLaw();
    float estimateDepthInCameraFrame(cv::Point2f pointCoordInLeftView, cv::Point2f pointCoordInRightView);
    cv::Point2f getCurrentCoordinatesOnNormalizedPlane(cv::Point2f& pointCoordInLeftView);
    Eigen::VectorXf calcErrorVector(std::vector<cv::Point2f> currentPointsCoordinates_LeftView_NormalizedPlane);
    Eigen::MatrixXf calcFeatureJacobianMatrix(std::vector<cv::Point2f> coords_xyOnNP, std::vector<float> depth_ZInCF);
    void calcCamCtrlVel(Eigen::MatrixXf& featureJacobianMatrix, Eigen::VectorXf& errorVector);
};

#endif // AAWIBVS_H
