#include "aawcoordtransform.h"

/**
 * @brief 以下3个值为相机左视野坐标系相对于并联机构动平台坐标系的3个偏置量，定义见手稿。
 * 为保持与并联机构控制量的单位一致，其单位取为mm。
 */

//------------------------Need to be measured!!!-------------------------
//前3个值不影响视觉伺服的最终结果精度，但是会影响伺服过程的平滑度，与实际值越接近，伺服轨迹越流畅。
const float AAWCoordTransform::offset_CFvsRF_X = 10;
const float AAWCoordTransform::offset_CFvsRF_Y = 258;
const float AAWCoordTransform::offset_CFvsRF_Z = 435;

//单位为秒，速度转换成位移用到的时间间隔，太小会导致伺服很慢，太大会导致伺服终点精度不高。不满足需求时可考虑改成动态的。
float AAWCoordTransform::timeIntegration_ = 0.8;

//public member functions

/**
 * @brief AAWCoordTransform::AAWCoordTransform 构造函数要求输入并联机构当前的位置控制量，其线性单位为mm，角度单位为°，需要将°转换为rad使用。
 * @param originalCtrlVal 输入的并联机构当前位置控制量。
 */
AAWCoordTransform::AAWCoordTransform(std::vector<float> &originalCtrlVal)
{
    convertRobotPos2rad(originalCtrlVal);
    calcTransformMatrix_B2A();  //只在初始化的时候计算一次，之后就用上一次的C2A来替代本次的B2A了。
    setVelTransformMatrix();
}

/**
 * @brief AAWCoordTransform::getCtrlVal 输入相机的视觉伺服控制速度，获得并联机构的位置、转角控制量输出。
 * @param cameraVel 当前时刻相机的视觉伺服控制速度，其单位为m/s和rad/s。
 * @return 并联机构的下一位置控制量。
 */
std::vector<float> AAWCoordTransform::getCtrlVal(Eigen::Matrix<float, 6, 1> cameraVel)
{
    calcCtrlVal(cameraVel);
    return ctrlVal_;
}

/**
 * @brief 更改坐标变换用到的时间积分量大小。
 * @param timeIntegration 新的时间积分量。
 */
void AAWCoordTransform::changeTimeIntegration(float timeIntegration)
{
    timeIntegration_ = timeIntegration;
    std::cout<<"Time integration for velocity changed to "<<timeIntegration_<<" seconds.\n";
}

//private member functions

/**
 * @brief AAWCoordTransform::convertRobotPos2rad 转换并存储并联机构的初始控制量，主要是把角度换成弧度。
 * @param robotPos 输入的并联机构控制量。
 * @note 转换之后存储在currentCtrlVal_中的控制量单位为mm和rad。
 */
void AAWCoordTransform::convertRobotPos2rad(std::vector<float> &robotPos)
{
    currentCtrlVal_.clear();
    for (int i = 0; i < 3; ++i)
        currentCtrlVal_.push_back(robotPos[i]);
    for (int i = 3; i < 6; ++i)
        currentCtrlVal_.push_back(robotPos[i] / 180 * M_PI);
}

/**
 * @brief AAWCoordTransform::calcTransformMatrix_B2A 计算并联机构动平台B相对于并联机构基坐标系A的坐标变换矩阵。
 * @note 这一步除了首次启用该对象是需要的，其他时候都是多余的，因为下一次的B2A就是上一次的C2A。
 */
void AAWCoordTransform::calcTransformMatrix_B2A()
{
    //a b c分别对应z y x欧拉角，右乘
    float x = currentCtrlVal_[0];
    float y = currentCtrlVal_[1];
    float z = currentCtrlVal_[2];
    float a = currentCtrlVal_[3];
    float b = currentCtrlVal_[4];
    float c = currentCtrlVal_[5];
    transformMatrix_B2A_ << cos(a)*cos(b), cos(a)*sin(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c), x,
                            sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), y,
                                 0-sin(b),                      cos(b)*sin(c),                      cos(b)*cos(c), z,
                                        0,                                  0,                                  0, 1;
}

/**
 * @brief AAWCoordTransform::setVelTransformMatrix 设置速度变换矩阵，由Camera frame的速度变换为动平台坐标系的运动速度。
 * @note 定义见手稿，由于两个坐标系固连才写成这样的形式，其中的分量需要精确标定测量获得。
 */
void AAWCoordTransform::setVelTransformMatrix()
{
    velTransformMatrix_ << 1, 0, 0, 0, offset_CFvsRF_Y, (-1)*offset_CFvsRF_Z,
                           0, 0, 1, offset_CFvsRF_Z, offset_CFvsRF_X, 0,
                           0,-1, 0, offset_CFvsRF_Y, 0, offset_CFvsRF_X,
                           0, 0, 0, 1, 0, 0,
                           0, 0, 0, 0, 0, 1,
                           0, 0, 0, 0,-1, 0;
}

/**
 * @brief AAWCoordTransform::calcCtrlVal 封装公有函数getCtrlVal()的功能，实现相机速度到控制量的转换。
 * @param cameraVel 输入的当前相机视觉伺服控制速度。
 */
void AAWCoordTransform::calcCtrlVal(Eigen::Matrix<float, 6, 1> &cameraVel)
{
    //获得控制速度、角速度的单位分别为mm/s和rad/s
    movingPlatformVel_ = velTransformMatrix_ * convertCamVel2mm(cameraVel);
    calcTransformMatrix_C2B();
    calcTransformMatrix_C2A();
    extractOriginDisp();
    extractEulerAngles();
}

/**
 * @brief AAWCoordTransform::convertCamVel2mm 将视觉伺服输入的标准单位的线速度转为mm/s，角速度单位保持rad/s不变，后续计算都保持这个单位。
 * @param cameraVel 输入的需要进行单位转换的速度。
 * @return 返回转换后线速度为mm/s，角速度为rad/s的速度。
 */
Eigen::Matrix<float, 6, 1> AAWCoordTransform::convertCamVel2mm(Eigen::Matrix<float, 6, 1> &cameraVel)
{
    Eigen::Matrix<float, 6, 1> cameraVelINmm;
    for (int i = 0; i < 3; ++i) {
        cameraVelINmm(i) = cameraVel(i)*1000;
    }
    //注意，视觉伺服传入的角速度单位是弧度每秒，rad/s，不需要转换，直接用于计算
    for (int i = 3; i < 6; ++i) {
        cameraVelINmm(i) = cameraVel(i);
    }
    return cameraVelINmm;
}

/**
 * @brief AAWCoordTransform::calcTransformMatrix_C2B 计算并联机构动平台的下个期望位姿坐标系C相对于当前坐标系B的坐标变换矩阵。
 */
void AAWCoordTransform::calcTransformMatrix_C2B()
{
    //c b a分别对应绕x y z定轴的RPY角，左乘
    float x = movingPlatformVel_(0)*timeIntegration_;
    float y = movingPlatformVel_(1)*timeIntegration_;
    float z = movingPlatformVel_(2)*timeIntegration_;
    float c = movingPlatformVel_(3)*timeIntegration_;
    float b = movingPlatformVel_(4)*timeIntegration_;
    float a = movingPlatformVel_(5)*timeIntegration_;
    transformMatrix_C2B_ << cos(a)*cos(b), cos(a)*sin(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c), x,
                            sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c), y,
                                 0-sin(b),                      cos(b)*sin(c),                      cos(b)*cos(c), z,
                                        0,                                  0,                                  0, 1;
}

/**
 * @brief AAWCoordTransform::calcTransformMatrix_C2A 计算并联机构动平台的下个期望位姿坐标系C相对于并联机构基坐标系A的坐标变换矩阵。
 */
void AAWCoordTransform::calcTransformMatrix_C2A()
{
    transformMatrix_C2A_ = transformMatrix_B2A_ * transformMatrix_C2B_;
    transformMatrix_B2A_ = transformMatrix_C2A_;    //这一步的C2A就是下一步的B2A，直接赋值存储。
}

/**
 * @brief AAWCoordTransform::extractOriginDisp 从transformMatrix_C2A_变换矩阵中提取并联机构控制量中的x, y, z分量。
 */
void AAWCoordTransform::extractOriginDisp()
{
    ctrlVal_.clear();
    ctrlVal_.push_back(transformMatrix_C2A_(0,3));
    ctrlVal_.push_back(transformMatrix_C2A_(1,3));
    ctrlVal_.push_back(transformMatrix_C2A_(2,3));
}

/**
 * @brief AAWCoordTransform::extractEulerAngles 从transformMatrix_C2A_变换矩阵进行逆向运动学求解，获得zyx欧拉角，作为并联机构控制量中的a, b, c分量。
 */
void AAWCoordTransform::extractEulerAngles()
{
    //由于并联机构工作空间不存在b等于±90°的情况，因此就不作考虑了
    float r11 = transformMatrix_C2A_(0, 0);
    float r21 = transformMatrix_C2A_(1, 0);
    float r31 = transformMatrix_C2A_(2, 0);
    float r32 = transformMatrix_C2A_(2, 1);
    float r33 = transformMatrix_C2A_(2, 2);
    float a = atan2(r21, r11);
    float b = atan2(0-r31, sqrt(r11*r11 + r21*r21));
    float c = atan2(r32, r33);

    //用于输出的控制量角度单位为角度。
    ctrlVal_.push_back(a / M_PI * 180);
    ctrlVal_.push_back(b / M_PI * 180);
    ctrlVal_.push_back(c / M_PI * 180);
}
