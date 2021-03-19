#include "aawibvs.h"

const double AawIBVS::lambda_ = 0.5;
const unsigned int AawIBVS::pointsNumber_ = 4;

//public member functions

AawIBVS::AawIBVS()
{
    setCameraIntrinsics(SN11818179);
    setDesiredCoordinatesOnNormalizedPlane(SN11818179);
}

AawIBVS::AawIBVS(CameraSerialNumber SN)
{
    setCameraIntrinsics(SN);
    setDesiredCoordinatesOnNormalizedPlane(SN);
}

/**
 * @brief IBVS::updateVertexesCoordinates 更新当前顶点坐标，需要在使用该类的地方主动调用该成员函数，以实现坐标的实时更新。
 * @param inputVertexes 存有4个顶点的像素坐标，由AawVertexesGainer的成员函数获得。
 */
void AawIBVS::updateVertexesCoordinates(std::vector<cv::Point2f> vertexesLeft, std::vector<cv::Point2f> vertexesRight)
{
    currentVertexes_LeftView_Pixel_.clear();
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        currentVertexes_LeftView_Pixel_.push_back(vertexesLeft[i]);
    }

    currentVertexes_RightView_Pixel_.clear();
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        currentVertexes_RightView_Pixel_.push_back(vertexesRight[i]);
    }
}

/**
 * @brief IBVS::updateControlLaw 更新控制法则，即根据新输入的特征点计算新的控制速度。
 * @note 用户调用updateVertexesCoordinates更新特征点像素坐标后，需要手动调用该函数以更新Camera frame的速度控制量，然后调用getControlVel()获取控制速度。
 */
void AawIBVS::updateControlLaw()
{
    std::vector<cv::Point2d> currentCoordsOnNP;
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        currentCoordsOnNP.push_back(getCurrentCoordinatesOnNormalizedPlane(currentVertexes_LeftView_Pixel_[i]));
    }
    Eigen::VectorXd errorVector = calcErrorVector(currentCoordsOnNP);
    std::vector<double> pointsDepthInCF;
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        pointsDepthInCF.push_back(estimateDepthInCameraFrame(currentVertexes_LeftView_Pixel_[i], currentVertexes_RightView_Pixel_[i]));
    }
    Eigen::MatrixXd featureJacobianMatrix = calcFeatureJacobianMatrix(currentCoordsOnNP, pointsDepthInCF);
    calcControlVel(featureJacobianMatrix, errorVector);
}

/**
 * @brief IBVS::getControlVel 返回camera frame的速度控制量，需要经坐标变换等才可由并联机构使用。
 * @return
 */
Eigen::VectorXd AawIBVS::getControlVel() const
{
    return controlVel_;
}

//private member functions

/**
 * @brief IBVS::setCameraIntrinsics 设置当前使用的相机的内参以及两镜头间的基线长度。
 * @param SN 当前相机的序列号。
 */
void AawIBVS::setCameraIntrinsics(CameraSerialNumber SN)
{
    switch (SN) {
    case SN11818179:
        cameraIntrinsicsMatrix_LeftView <<  CameraSN11818179::fx_left,                          0,  CameraSN11818179::cx_left,
                                                                    0,  CameraSN11818179::fy_left,  CameraSN11818179::cy_left,
                                                                    0,                          0,                          1;
        cameraIntrinsicsMatrix_RightView<< CameraSN11818179::fx_right,                          0, CameraSN11818179::cx_right,
                                                                    0, CameraSN11818179::fy_right, CameraSN11818179::cy_right,
                                                                    0,                          0,                          1;
        baseLine_ = CameraSN11818179::baseLine;
        focalLengthInPixel_ = CameraSN11818179::focalLengthInPixel;
        break;
    case SN16988350:
        cameraIntrinsicsMatrix_LeftView <<  CameraSN16988350::fx_left,                          0,  CameraSN16988350::cx_left,
                                                                    0,  CameraSN16988350::fy_left,  CameraSN16988350::cy_left,
                                                                    0,                          0,                          1;
        cameraIntrinsicsMatrix_RightView<< CameraSN16988350::fx_right,                          0, CameraSN16988350::cx_right,
                                                                    0, CameraSN16988350::fy_right, CameraSN16988350::cy_right,
                                                                    0,                          0,                          1;
        baseLine_ = CameraSN16988350::baseLine;
        focalLengthInPixel_ = CameraSN16988350::focalLengthInPixel;
        break;
    default:
        std::cerr<<"No intrinsic parameters for current camera, initialization failed!"<<std::endl;
        break;
    }
}

/**
 * @brief IBVS::setDesiredCoordinatesOnNormalizedPlane 设置理想位置（伺服目标位置）处，成像点在归一化平面上的坐标。当前只考虑一台相机其左视野，后续有需求再拓展。
 * @param SN 当前相机的序列号。
 */
void AawIBVS::setDesiredCoordinatesOnNormalizedPlane(CameraSerialNumber SN)
{
    switch (SN) {
    case SN11818179:
        desiredPointsCoordinates_LeftView_NormalizedPlane_.push_back(CameraSN11818179::desiredCoord_LeftView_NormalizedPlane_0);
        desiredPointsCoordinates_LeftView_NormalizedPlane_.push_back(CameraSN11818179::desiredCoord_LeftView_NormalizedPlane_1);
        desiredPointsCoordinates_LeftView_NormalizedPlane_.push_back(CameraSN11818179::desiredCoord_LeftView_NormalizedPlane_2);
        desiredPointsCoordinates_LeftView_NormalizedPlane_.push_back(CameraSN11818179::desiredCoord_LeftView_NormalizedPlane_3);
        break;
    default:
        std::cerr<<"No desired coordinates available for current camera, initialization failed!"<<std::endl;
        break;
    }
}

/**
 * @brief IBVS::estimateDepthInCameraFrame 由双目相机观察点的视差估计其深度。
 * @param pointCoordInLeftView 特征点在相机左侧视野中的像素坐标。
 * @param pointCoordInRightView 特征点在相机右侧视野中的像素坐标。
 * @return 返回特征点的深度信息，即在相机左视野相机坐标系（Camera frame）中的Z坐标值。
 */
double AawIBVS::estimateDepthInCameraFrame(cv::Point2f pointCoordInLeftView, cv::Point2f pointCoordInRightView)
{
    float parallax = abs(pointCoordInLeftView.x - pointCoordInRightView.x);
    float depth = focalLengthInPixel_ * baseLine_ / parallax;   //Z=bf/parallax
    return depth;
}

/**
 * @brief IBVS::getCurrentCoordinatesOnNormalizedPlane 计算相机左视野中某特征点在归一化平面上的x，y坐标。
 * @param pointCoordInLeftView 特征点在左视野中的像素平面坐标u，v。
 * @return 返回特征点在归一化平面上的x，y坐标分量。
 */
cv::Point2d AawIBVS::getCurrentCoordinatesOnNormalizedPlane(cv::Point2f pointCoordInLeftView)
{
    cv::Point2d pointCoordOnNP;
    Eigen::Vector3f homoCoordInPixel;
    homoCoordInPixel << pointCoordInLeftView.x, pointCoordInLeftView.y, 1;
    Eigen::Vector3f homoCoordOnNP = cameraIntrinsicsMatrix_LeftView.colPivHouseholderQr().solve(homoCoordInPixel);
    pointCoordOnNP.x = homoCoordOnNP(0);
    pointCoordOnNP.y = homoCoordOnNP(1);
    return pointCoordOnNP;
}

/**
 * @brief IBVS::calcErrorVector 计算误差向量 e = x - x*。
 * @param currentPointsCoordinates_LeftView_NormalizedPlane 当前左视野中在归一化平面上的点坐标。
 * @return 返回误差向量e。
 */
Eigen::VectorXd AawIBVS::calcErrorVector(std::vector<cv::Point2d> currentPointsCoordinates_LeftView_NormalizedPlane)
{
    Eigen::VectorXd errorVector(pointsNumber_*2);
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        errorVector(2*i) = currentPointsCoordinates_LeftView_NormalizedPlane[i].x - desiredPointsCoordinates_LeftView_NormalizedPlane_[i].x;
        errorVector(2*i+1) = currentPointsCoordinates_LeftView_NormalizedPlane[i].y - desiredPointsCoordinates_LeftView_NormalizedPlane_[i].y;
    }
    return errorVector;
}

/**
 * @brief IBVS::calcFeatureJacobianMatrix 计算特征雅可比矩阵Le。
 * @param coords_xyOnNP 存放特征点在左视野归一化平面上的x，y坐标
 * @param depth_ZInCF 存放特征点在左视野相机坐标系中的Z坐标。
 * @return 返回特征雅可比矩阵Le。
 */
Eigen::MatrixXd AawIBVS::calcFeatureJacobianMatrix(std::vector<cv::Point2d> coords_xyOnNP, std::vector<double> depth_ZInCF)
{
    Eigen::MatrixXd featureJacobianMatrix(pointsNumber_*2, 6);
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        featureJacobianMatrix(2*i, 0) = (-1) / depth_ZInCF[i];
        featureJacobianMatrix(2*i, 1) = 0;
        featureJacobianMatrix(2*i, 2) = coords_xyOnNP[i].x / depth_ZInCF[i];
        featureJacobianMatrix(2*i, 3) = coords_xyOnNP[i].x * coords_xyOnNP[i].y;
        featureJacobianMatrix(2*i, 4) = (-1) * (1 + pow(coords_xyOnNP[i].x, 2));
        featureJacobianMatrix(2*i, 5) = coords_xyOnNP[i].y;
        featureJacobianMatrix(2*i+1, 0) = 0;
        featureJacobianMatrix(2*i+1, 1) = (-1) / depth_ZInCF[i];
        featureJacobianMatrix(2*i+1, 2) = coords_xyOnNP[i].y / depth_ZInCF[i];
        featureJacobianMatrix(2*i+1, 3) = 1 + pow(coords_xyOnNP[i].y, 2);
        featureJacobianMatrix(2*i+1, 4) = (-1) * coords_xyOnNP[i].x * coords_xyOnNP[i].y;
        featureJacobianMatrix(2*i+1, 5) = (-1) * coords_xyOnNP[i].x;
    }
    return featureJacobianMatrix;
}

/**
 * @brief IBVS::calcControlVel 计算控制速度，该速度为Camera frame的速度，需经过坐标变换才能成为并联机构的速度和位置控制量。
 * @param featureJacobianMatrix 特征雅可比矩阵Le。
 * @param errorVector 误差向量e。
 */
void AawIBVS::calcControlVel(Eigen::MatrixXd featureJacobianMatrix, Eigen::VectorXd errorVector)
{
    controlVel_ = featureJacobianMatrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve((-1) * lambda_ * errorVector);
}
