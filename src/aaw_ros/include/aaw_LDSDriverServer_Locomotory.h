#ifndef AAW_LDS_DRIVER_SERVER_LOCOMOTORY_H_
#define AAW_LDS_DRIVER_SERVER_LOCOMOTORY_H_

#include <ros/ros.h>
#include <iostream>
#include <cstring>
#include "aawldsdriver.h"
#include <aaw_ros/LDSInteraction.h>

//完成激光开关控制、位移数据读取，并与"aaw_visualServo"节点通信
class AAWLDSServerLocomotory
{
public:
    AAWLDSServerLocomotory(ros::NodeHandle* nodehandle);
    ~AAWLDSServerLocomotory();

private:
    ros::NodeHandle nh_;
    ros::ServiceServer LDSInteractionServer_;

    AAWLDSDriverClass * LDSPtr_;

    float getDistance();
    void turnOnLaser();
    void turnOffLaser();

    bool LDSInteractionCallback(aaw_ros::LDSInteractionRequest& requestType, aaw_ros::LDSInteractionResponse& response);

    void errorMsg(const char * msg);
    void showMsg(const char * msg);
};

#endif