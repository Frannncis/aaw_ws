#ifndef AAW_MOVE_ROBOT_SERVER_H_
#define AAW_MOVE_ROBOT_SERVER_H_

#include <ros/ros.h>
#include "aaw_ros/MoveRobot.h"
#include <iostream>
#include <vector>
#include "aawtcpserver.h"
#include "aawcoordtransform.h"

class AAWMoveRobotServer
{
public:
    AAWMoveRobotServer(ros::NodeHandle* nodehandle);
    int AAWEnableRobot();
    int AAWDisableRobot();

private:
    ros::NodeHandle nh_;
    ros::ServiceServer moveRobotService_;
    AAWTCPServer *myTCPServerPtr_;
    AAWCoordTransform *coordTransformerPtr_;

    bool serviceCallback(aaw_ros::MoveRobotRequest& requestPos, aaw_ros::MoveRobotResponse& execStatus);
};

#endif