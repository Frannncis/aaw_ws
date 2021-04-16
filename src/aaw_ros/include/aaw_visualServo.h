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
#include <interaction/RestartRobotMotion.h>
#include <interaction/MoveCar.h>
#include <interaction/AdjustCarPos.h>
#include <boost/bind.hpp>
#include "aaw_originalCtrlVal.h"

namespace visualServo
{
    static const std::string Left_View = "Left View";
    static const std::string Right_View = "Right View";
    const unsigned int timeWaitBeforeDocking_ = 1;   //seconds

    bool toDock_ = false; //决定此次动作是对接还是分离,true为对接,false为分离，初始值设为false，是因为wakeUpActionCallback中会进行一次反转。
    bool readyToGoHome_ = false;    //上方的插销动作完成后，将此标志设为true，进行回撤和回零点动作。
    bool taskFinished_ = true; //正式程序中，这个的初始值应该为true，等收到小车的信号再设为false.
    bool timeIntegrationChanged_ = false;
    const float moveUpDistance_ = 94.05559;
    const float moveDownDistance_ = moveUpDistance_;
    const unsigned int keepDockingSecs_ = 20;
    const unsigned int communicationRetryingTimes_ = 5;
    const float lowVelTimeIntegration_ = 0.4;   //senconds

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

    AAWIBVS *ibvsPtr;

    void showMsg(const char * msg);
    void errorMsg(const char * msg);

    void imageCb(const sensor_msgs::ImageConstPtr& leftImage, const sensor_msgs::ImageConstPtr& rightImage);

    //并联机构控制相关函数
    int visualServoRobot(Eigen::Matrix<float, 6, 1> camCtrlVel);
    int dock();
    void disableRobot();
    void enableRobot();
    int withdrawAndGoHome();
    int withdraw();
    int move2OriginalPos();
    void checkAndChangeTimeIntegration();

    //上方直线运动模组控制相关函数
    int insertBolt();
    int pullOutBolt();

    //侧方推杆控制相关函数
    int pushOutLDS();
    int pullBackLDS();

    bool wakeUpAction();    //预留，接受小车信号进行下一次动作。
    int updateCoordTrans();
    void waitAndWakeUpAction();     //不与小车通信时用这个触发新的动作。

    bool restartRobotMotionCallback(interaction::RestartRobotMotionRequest& requestMotion, interaction::RestartRobotMotionResponse& execStatus);
    int askCarToMove();
    int moveCarForwardALittle();
    int moveCarBackwardALittle();
}

#endif