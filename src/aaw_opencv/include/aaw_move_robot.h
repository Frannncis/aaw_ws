#ifndef AAW_MOVE_ROBOT_CLASS_H_
#define AAW_MOVE_ROBOT_CLASS_H_

#include <ros/ros.h>
#include "aaw_opencv/MoveRobot.h"
#include <iostream>
#include <vector>
#include "aawtcpserver.h"
#include "aawcoordtransform.h"

class AAWMoveRobotClass
{
public:
    AAWMoveRobotClass(ros::NodeHandle* nodehandle);
    int AAWEnableRobot();
    int AAWDisableRobot();

private:
    ros::NodeHandle nh_;
    ros::ServiceServer moveRobotService_;
    AAWTCPServer *myTCPServerPtr_;
    AAWCoordTransform *coordTransformerPtr_;

    bool serviceCallback(aaw_opencv::MoveRobotRequest& requestPos, aaw_opencv::MoveRobotResponse& execStatus);
};

#endif