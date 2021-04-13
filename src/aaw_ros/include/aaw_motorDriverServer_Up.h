#ifndef AAW_MOTOR_DRIVER_SERVER_UP_H_
#define AAW_MOTOR_DRIVER_SERVER_UP_H_

#include <ros/ros.h>
#include <iostream>
#include <cstdlib>
#include "aaw_ros/BoltMotionCtrl.h"
#include "aawserver4motor.h"

class AAWMotorDriverServerUp
{
public:
    AAWMotorDriverServerUp(ros::NodeHandle* nodehandle);
    ~AAWMotorDriverServerUp();

private:
    static const unsigned int motionTime_;

    ros::NodeHandle nh_;
    ros::ServiceServer boltMotionCtrl_;

    AAWMotorServer *upMotorServerPtr_;

    int stickOut();
    int pullBack();
    int stop();

    void errorMsg(const char * msg);
    void showMsg(const char * msg);

    bool boltMotionCtrlCallback(aaw_ros::BoltMotionCtrlRequest& requestDir, aaw_ros::BoltMotionCtrlResponse& execStatus);
};

#endif