#include "aawibvs.h"

const float AAWIBVS::lambda_ = 0.5;
const float AAWIBVS::varianceOfEVEA_Threshold_Near_ = 1.0e-2;//1.0e-3;
const float AAWIBVS::varianceOfEVEA_Threshold_Arrived_ = 1.0e-3; //5.0e-5;
const unsigned int AAWIBVS::pointsNumber_ = 4;
const unsigned int AAWIBVS::desiredCoordsAccumMaxTimes_ = 50; //获取期望位姿的样本量（取这些样本的平均值作为期望位姿）
const unsigned int AAWIBVS::maxListSize_ = 5;

//public member functions

AAWIBVS::AAWIBVS() : desiredCoordsAccumCount_(0)
{
    initIBVS(SN11818179);
}

AAWIBVS::AAWIBVS(CameraSerialNumber SN) : desiredCoordsAccumCount_(0)
{
    initIBVS(SN);
}

/**
 * @brief IBVS::updateVertexesCoordinates 更新当前顶点坐标，需要在使用该类的地方主动调用该成员函数，以实现坐标的实时更新。
 * @param inputVertexes 存有4个顶点的像素坐标，由AawVertexesGainer的成员函数获得。
 */
void AAWIBVS::updateVertexesCoordinates(std::vector<cv::Point2f> vertexesLeft, std::vector<cv::Point2f> vertexesRight)
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
 * @brief IBVS::updateControlLaw 更新控制法则，即根据新输入的特征点的像素坐标计算新的控制速度。
 * @note 用户在更新特征点坐标后，需要手动调用该函数才能调用别的公有函数，否则值不会更新。
 */
void AAWIBVS::updateControlLaw()
{
    std::vector<cv::Point2f> currentCoordsOnNP;
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        currentCoordsOnNP.push_back(getCurrentCoordinatesOnNormalizedPlane(currentVertexes_LeftView_Pixel_[i]));
    }
    Eigen::VectorXf errorVector = calcErrorVector(currentCoordsOnNP);
    std::vector<float> pointsDepthInCF;
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        pointsDepthInCF.push_back(estimateDepthInCameraFrame(currentVertexes_LeftView_Pixel_[i], currentVertexes_RightView_Pixel_[i]));
    }
    Eigen::MatrixXf featureJacobianMatrix = calcFeatureJacobianMatrix(currentCoordsOnNP, pointsDepthInCF);
    calcCamCtrlVel(featureJacobianMatrix, errorVector);
}

/**
 * @brief IBVS::getCamCtrlVel 返回相机的速度控制量。
 * @return 相机的控制速度。
 */
Eigen::Matrix<float, 6, 1> AAWIBVS::getCamCtrlVel()
{
    return camCtrlVel_;
}

/**
 * @brief 循环调用此函数一定次数，获取期望位姿的特征点在归一化平面坐标数据，用以后续的视觉伺服计算误差向量。
 */
void AAWIBVS::measureDesiredCoordsOnNP() {
    if (desiredCoordsAccumCount_ >= desiredCoordsAccumMaxTimes_)
        return;
    else {
        for (unsigned int i = 0; i < pointsNumber_; ++i) {
            cv::Point2f currentCoordsOnNP;
            currentCoordsOnNP = getCurrentCoordinatesOnNormalizedPlane(currentVertexes_LeftView_Pixel_[i]);
            desiredCoordsOnNP_sum_[i].x += currentCoordsOnNP.x;
            desiredCoordsOnNP_sum_[i].y += currentCoordsOnNP.y;
        }
        ++desiredCoordsAccumCount_;
        std::cout<<"Count "<<desiredCoordsAccumCount_<<"\n";
        if (desiredCoordsAccumCount_ == desiredCoordsAccumMaxTimes_) {
            std::cout<<"Desired coords of feature points on normalized plane have been accumulated "<<desiredCoordsAccumCount_<<" times, "
                       "the mean values of these coords are listed below:\n";
            for (unsigned int i = 0; i < pointsNumber_; ++i) {
                std::cout<<"Point "<<i<<":\n";
                std::cout<<desiredCoordsOnNP_sum_[i].x / float(desiredCoordsAccumCount_)<<", "
                        <<desiredCoordsOnNP_sum_[i].y / float(desiredCoordsAccumCount_)<<"\n";
            }
            std::cout<<"Please update these values to the aawibvs.h file.\n";
        }
    }
}

/**
 * 返回是否伺服至期望位姿的判断结果，通过errorVector的元素绝对值之和的方差小于阈值来判断（方差越小，数据越稳定）。
 * @return 是否到达期望位姿的判定结果。
 */
bool AAWIBVS::isDesiredPosArrived()
{
    std::cout<<"sum of abs error vector elements: "<<sumOfErrorVecElementsAbs_<<"\n";
    if (varianceOfEVEA_ < varianceOfEVEA_Threshold_Arrived_)
        return true;
    return false;
}

/**
 * 返回是否伺服接近期望位姿的判断结果，用来更改速度积分量的判断标志。
 * @return　是否伺服接近期望位姿的判断结果。
 */
bool AAWIBVS::isDesiredPosNear() {
    if (varianceOfEVEA_ < varianceOfEVEA_Threshold_Near_)
        return true;
    return false;
}

//private member functions

void AAWIBVS::initIBVS(CameraSerialNumber SN)
{
    setCameraIntrinsics(SN);
    setDesiredCoordinatesOnNormalizedPlane(SN);
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        desiredCoordsOnNP_sum_.push_back(cv::Point2f(0, 0));
    }
    sumOfErrorVecElementsAbs_ = 0;
    currentListSize_ = 0;
    varianceOfEVEA_ = 0;
}

/**
 * @brief IBVS::setCameraIntrinsics 设置当前使用的相机的内参以及两镜头间的基线长度。
 * @param SN 当前相机的序列号。
 */
void AAWIBVS::setCameraIntrinsics(CameraSerialNumber SN)
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
        exit(1);
    }
}

/**
 * @brief IBVS::setDesiredCoordinatesOnNormalizedPlane 设置理想位置（伺服目标位置）处，成像点在归一化平面上的坐标。当前只考虑一台相机其左视野，后续有需求再拓展。
 * @param SN 当前相机的序列号。
 */
void AAWIBVS::setDesiredCoordinatesOnNormalizedPlane(CameraSerialNumber SN)
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
        exit(1);
    }
}

/**
 * @brief IBVS::estimateDepthInCameraFrame 由双目相机观察点的视差估计其深度。
 * @param pointCoordInLeftView 特征点在相机左侧视野中的像素坐标。
 * @param pointCoordInRightView 特征点在相机右侧视野中的像素坐标。
 * @return 返回特征点的深度信息，即在相机左视野相机坐标系（Camera frame）中的Z坐标值。
 */
float AAWIBVS::estimateDepthInCameraFrame(cv::Point2f pointCoordInLeftView, cv::Point2f pointCoordInRightView)
{
    float parallax = std::abs(pointCoordInLeftView.x - pointCoordInRightView.x);
    float depth = focalLengthInPixel_ * baseLine_ / parallax;   //Z=bf/parallax
    return depth;
}

/**
 * @brief IBVS::getCurrentCoordinatesOnNormalizedPlane 计算相机左视野中某特征点在归一化平面上的x，y坐标。
 * @param pointCoordInLeftView 特征点在左视野中的像素平面坐标u，v。
 * @return 返回特征点在归一化平面上的x，y坐标分量。
 */
cv::Point2f AAWIBVS::getCurrentCoordinatesOnNormalizedPlane(cv::Point2f& pointCoordInLeftView)
{
    cv::Point2f pointCoordOnNP;
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
Eigen::VectorXf AAWIBVS::calcErrorVector(std::vector<cv::Point2f> currentPointsCoordinates_LeftView_NormalizedPlane)
{
    Eigen::VectorXf errorVector(pointsNumber_*2);
    for (unsigned int i = 0; i < pointsNumber_; ++i) {
        errorVector(2*i) = currentPointsCoordinates_LeftView_NormalizedPlane[i].x - desiredPointsCoordinates_LeftView_NormalizedPlane_[i].x;
        errorVector(2*i+1) = currentPointsCoordinates_LeftView_NormalizedPlane[i].y - desiredPointsCoordinates_LeftView_NormalizedPlane_[i].y;
    }

    sumOfErrorVecElementsAbs_ = 0;
    for (unsigned int i = 0; i < errorVector.size(); ++i) {
        sumOfErrorVecElementsAbs_ += std::abs(errorVector(i));
    }
    updateEVEAList();
    calcVarianceOfEVEA();

    return errorVector;
}

/**
 * @brief IBVS::calcFeatureJacobianMatrix 计算特征雅可比矩阵Le。
 * @param coords_xyOnNP 存放特征点在左视野归一化平面上的x，y坐标
 * @param depth_ZInCF 存放特征点在左视野相机坐标系中的Z坐标。
 * @return 返回特征雅可比矩阵Le。
 */
Eigen::MatrixXf AAWIBVS::calcFeatureJacobianMatrix(std::vector<cv::Point2f> coords_xyOnNP, std::vector<float> depth_ZInCF)
{
    Eigen::MatrixXf featureJacobianMatrix(pointsNumber_*2, 6);
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
 * @brief IBVS::calcCameraVel 计算相机的飞行速度，该速度为Camera frame在空间中相对其自身的瞬时速度。
 * @param featureJacobianMatrix 特征雅可比矩阵Le。
 * @param errorVector 误差向量e。
 */
void AAWIBVS::calcCamCtrlVel(Eigen::MatrixXf& featureJacobianMatrix, Eigen::VectorXf& errorVector)
{
    camCtrlVel_ = featureJacobianMatrix.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve((-1) * lambda_ * errorVector);
}

/**
 * 更新EVEA链表，包括从头部删除旧元素、从尾部添加新元素。
 */
void AAWIBVS::updateEVEAList()
{
    if (currentListSize_ < maxListSize_) {
        list_sumOfEVEA_.push_back(sumOfErrorVecElementsAbs_);
        ++currentListSize_;
    }
    else {
        list_sumOfEVEA_.pop_front();
        list_sumOfEVEA_.push_back(sumOfErrorVecElementsAbs_);
    }
}

/**
 * 计算误差向量元素绝对值之和的方差。
 * @return　误差向量元素绝对值之和的方差。
 */
void AAWIBVS::calcVarianceOfEVEA() {
    float varianceOfEVEA_sum = 0;
    float sum, mean;
    sum = std::accumulate(list_sumOfEVEA_.begin(), list_sumOfEVEA_.end(), 0);
    mean = sum / (float)currentListSize_;
    for (listIter_ = list_sumOfEVEA_.begin(); listIter_ != list_sumOfEVEA_.end(); listIter_++) {
        varianceOfEVEA_sum += std::pow(*listIter_ - mean, 2);
    }
    varianceOfEVEA_ = varianceOfEVEA_sum / (float) currentListSize_;

    std::cout<<"Variance of EVEA = "<<varianceOfEVEA_<<"\n";
}
