#ifndef AAW_MOTOR_DRIVER_SERVER_SIDE_H_
#define AAW_MOTOR_DRIVER_SERVER_SIDE_H_

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include "aaw_ros/LDSMotionCtrl.h"
#include "aawserver4motor.h"

class AAWMotorDriverServerSide
{
public:
    AAWMotorDriverServerSide(ros::NodeHandle* nodehandle);
    ~AAWMotorDriverServerSide();

private:
    static const unsigned int motionTime_;

    ros::NodeHandle nh_;
    ros::ServiceServer LDSMotionCtrl_;

    AAWMotorServer *sideMotorServerPtr_;

    int stickOut();
    int pullBack();
    int stop();

    void errorMsg(const char * msg);
    void showMsg(const char * msg);

    bool LDSMotionCtrlCallback(aaw_ros::LDSMotionCtrlRequest& requestDir, aaw_ros::LDSMotionCtrlResponse& execStatus);
};

#endif