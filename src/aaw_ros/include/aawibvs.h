#ifndef AAWIBVS_H
#define AAWIBVS_H

#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <list>
#include <iterator>
#include <algorithm>
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
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_0(-0.274239, -0.286026);
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_1(-0.274708, 0.197363);
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_2(0.721141, 0.205359);
    const cv::Point2f desiredCoord_LeftView_NormalizedPlane_3(0.715898, -0.294417);
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
    void updateControlLaw();
    Eigen::Matrix<float, 6, 1> getCamCtrlVel();
    void measureDesiredCoordsOnNP();
    bool isDesiredPosArrived();
    bool isDesiredPosNear();

private:
    typedef std::list<float> LIST;

    static const float lambda_;
    static const float varianceOfEVEA_Threshold_Near_;
    static const float varianceOfEVEA_Threshold_Arrived_;
    static const unsigned int pointsNumber_;
    static const unsigned int desiredCoordsAccumMaxTimes_;
    static const unsigned int maxListSize_;

    Eigen::Matrix<float, 3, 3> cameraIntrinsicsMatrix_LeftView, cameraIntrinsicsMatrix_RightView;
    float baseLine_;
    float focalLengthInPixel_;
    std::vector<cv::Point2f> currentVertexes_LeftView_Pixel_,
                             currentVertexes_RightView_Pixel_;
    std::vector<cv::Point2f> desiredPointsCoordinates_LeftView_NormalizedPlane_;
    Eigen::Matrix<float, 6, 1> camCtrlVel_;
    std::vector<cv::Point2f> desiredCoordsOnNP_sum_;
    unsigned int desiredCoordsAccumCount_;
    float sumOfErrorVecElementsAbs_;
    LIST list_sumOfEVEA_;
    LIST::iterator listIter_;
    unsigned int currentListSize_;
    float varianceOfEVEA_;

    void initIBVS(CameraSerialNumber SN);
    void setCameraIntrinsics(CameraSerialNumber SN);
    void setDesiredCoordinatesOnNormalizedPlane(CameraSerialNumber SN);

    float estimateDepthInCameraFrame(cv::Point2f pointCoordInLeftView, cv::Point2f pointCoordInRightView);
    cv::Point2f getCurrentCoordinatesOnNormalizedPlane(cv::Point2f& pointCoordInLeftView);
    Eigen::VectorXf calcErrorVector(std::vector<cv::Point2f> currentPointsCoordinates_LeftView_NormalizedPlane);
    Eigen::MatrixXf calcFeatureJacobianMatrix(std::vector<cv::Point2f> coords_xyOnNP, std::vector<float> depth_ZInCF);
    void calcCamCtrlVel(Eigen::MatrixXf& featureJacobianMatrix, Eigen::VectorXf& errorVector);
    void updateEVEAList();
    void calcVarianceOfEVEA();
};

#endif // AAWIBVS_H
