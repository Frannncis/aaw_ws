#ifndef AAW_VISUAL_SERVO_H_
#define AAW_VISUAL_SERVO_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen/Dense>
#include <Eigen/Core>
// #include "aawvertexesgainer.h"
#include "aawvertexesgainer2.h"
#include "aawibvs.h"
#include <aaw_ros/MoveRobot.h>
#include <aaw_ros/MoveRobot_DistanceZ.h>
#include <aaw_ros/MoveRobot_CtrlVal.h>
#include <aaw_ros/ChangeRobotStatus.h>
#include <aaw_ros/ChangeTimeIntegration.h>
#include <aaw_ros/BoltMotionCtrl.h>
#include <aaw_ros/LDSMotionCtrl.h>
#include <aaw_ros/UpdateCoordTransformer.h>
#include <aaw_ros/WeightSensorData.h>
#include <aaw_ros/CurrentRobotCtrlVal.h>
#include <interaction/RestartRobotMotion.h>
#include <interaction/MoveCar.h>
#include <interaction/AdjustCarPos.h>
#include <boost/bind.hpp>
#include "aaw_originalCtrlVal.h"

namespace visualServo
{
    static const std::string Left_View = "Left View";
    static const std::string Right_View = "Right View";

    bool toDock_ = false; //决定此次动作是对接还是分离,true为对接,false为分离，初始值设为false，是因为wakeUpActionCallback中会进行一次反转。
    bool readyToGoHome_ = false;    //上方的插销动作完成后，将此标志设为true，进行回撤和回零点动作。
    bool taskFinished_ = true; //正式程序中，这个的初始值应该为true，等收到小车的信号再设为false.
    bool timeIntegrationChanged_ = false;
    const float moveUpDistance_ = 90.44059;
    const float moveDownDistance_ = moveUpDistance_;
    const unsigned int communicationRetryingTimes_ = 5;
    const float lowVelTimeIntegration_ = 0.4;   //senconds
    std::vector<float> newCtrlVal_(originalCtrlVal_); //当物体与相机靠得太近时，用来处理并联机构侧移的控制量
    float cameraStepBackLength_ = 20;   //物体与相机靠太近时，相机每次尝试后退的距离，单位为mm
    const int outaSecurityWeight_ = 2200;
    std::vector<float> currentCtrlVal_(originalCtrlVal_);

    ros::ServiceClient *moveClientPtr;
    ros::ServiceClient *moveClientPtr_DistanceZ;
    ros::ServiceClient *moveClientPtr_CtrlVal;
    ros::ServiceClient *changeRobotStatusClientPtr;
    ros::ServiceClient *changeTimeIntegrationClientPtr;
    ros::ServiceClient *boltMotionCtrlClientPtr;
    ros::ServiceClient *LDSMotionCtrlClientPtr;
    ros::ServiceClient *updateCoordTransClientPtr;

    ros::ServiceClient *moveCarClientPtr;
    ros::ServiceClient *adjustCarPosClientPtr;
    ros::ServiceServer restartRobotMotion_;

    ros::Subscriber weightSensorSub_;
    ros::Subscriber robotCtrlValSub_;

    AAWIBVS *ibvsPtr;

    void showMsg(const char * msg);
    void errorMsg(const char * msg);

    void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage);

    //并联机构控制相关函数
    int visualServoRobot(Eigen::Matrix<float, 6, 1> & camCtrlVel);
    void ensureVisualServoRobot(Eigen::Matrix<float, 6, 1> & camCtrlVel);
    int dock();
    void ensureDock();
    void disableRobot();
    void enableRobot();
    void withdrawAndGoHome();
    int withdraw(float distance); //只确保ROS Service成功通信
    void ensureWithdraw(float distance); //确保并联机构的反馈为成功执行指令
    int move2Pos(std::vector<float> & robotCtrlVal); //只确保ROS Service成功通信
    void ensureMove2Pos(std::vector<float> & robotCtrlVal); //确保并联机构的反馈为成功执行指令
    void checkAndChangeTimeIntegration();

    //上方直线运动模组控制相关函数
    int insertBolt();
    int pullOutBolt();

    //侧方推杆控制相关函数
    int pushOutLDS();
    int pullBackLDS();

    void wakeUpAction();
    int updateCoordTrans(std::vector<float> & robotCtrlVal);
    void waitAndWakeUpAction();     //不与小车通信时用这个触发新的动作

    bool restartRobotMotionCallback(interaction::RestartRobotMotionRequest& requestMotion, interaction::RestartRobotMotionResponse& execStatus);
    void weightSensorDataCallback(const aaw_ros::WeightSensorData::ConstPtr & msg);
    void robotCtrlValCallback(const aaw_ros::CurrentRobotCtrlVal::ConstPtr & msg);

    int askCarToMove();
    int carCreepingForward();
    int carCreepingBackward();
    
    void withdrawAndRestartServo();
}

#endif