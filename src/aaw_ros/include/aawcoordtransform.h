#ifndef AAWCOORDTRANSFORM_H
#define AAWCOORDTRANSFORM_H

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include <vector>

/**
 * @brief The AAWCoordTransform class 实现视觉伺服输出（相机速度）到并联机构运动控制量的转换。
 * @note 该类对象在伺服过程中需要一直存在，因为要记录伺服过程中的并联机构控制量，用来计算坐标变换矩阵。
 * 新建对象时需要传入并联机构当前的位置，由于并联机构没有位置反馈，所以伺服开始前需要回零点，然后将零点位置用来初始化该类对象，此后一边伺服，一边更新控制量。
 * 不允许使用不带参数的默认构造函数！
 * @warning 每次进行getCtrlVal()操作之后，该类默认机器人已经达到ctrlVal_指定的位姿。因为需要当前位姿信息进行下一次的控制量计算，而当前位姿只能由该类上一步计算的坐标变换矩阵获取！
 * 如果实际上机器人并没有走到指定位置，那么该类后续的计算将会失效（因为上一步保存的坐标变化矩阵已经不对了），其累加效应会让控制量越偏越远。
 */
class AAWCoordTransform
{
public:
    AAWCoordTransform(std::vector<float> &originalCtrlVal);
    std::vector<float> getCtrlVal(Eigen::Matrix<float, 6, 1> cameraVel);

private:
    static const float offset_CFvsRF_X;
    static const float offset_CFvsRF_Y;
    static const float offset_CFvsRF_Z;
    static const float timeIntegration_;

    Eigen::Matrix<float, 6, 6> velTransformMatrix_;
    Eigen::Matrix<float, 6, 1> movingPlatformVel_;
    std::vector<float> currentCtrlVal_;
    std::vector<float> ctrlVal_; //final output control value
    Eigen::Matrix<float, 4, 4> transformMatrix_B2A_;
    Eigen::Matrix<float, 4, 4> transformMatrix_C2B_;
    Eigen::Matrix<float, 4, 4> transformMatrix_C2A_;

    void convertRobotPos2rad(std::vector<float> &robotPos);
    void setVelTransformMatrix();
    void calcCtrlVal(Eigen::Matrix<float, 6, 1> &cameraVel);
    Eigen::Matrix<float, 6, 1> convertCamVel2mm(Eigen::Matrix<float, 6, 1> &cameraVel);
    void calcTransformMatrix_B2A();
    void calcTransformMatrix_C2B();
    void calcTransformMatrix_C2A();
    void extractOriginDisp();
    void extractEulerAngles();
};

#endif // AAWCOORDTRANSFORM_H
