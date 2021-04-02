#ifndef AAW_MOVE_ROBOT_SERVER_H_
#define AAW_MOVE_ROBOT_SERVER_H_

#include <ros/ros.h>
#include "aaw_ros/MoveRobot.h"
#include "aaw_ros/MoveRobot_DistanceZ.h"
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
    ~AAWMoveRobotServer();

private:
    ros::NodeHandle nh_;
    ros::ServiceServer moveRobot_camVel_;
    ros::ServiceServer moveRobot_distanceZ_;
    AAWTCPServer *myTCPServerPtr_;
    AAWCoordTransform *coordTransformerPtr_;

    std::vector<float> ctrlVal_;

    bool camVelInputCallback(aaw_ros::MoveRobotRequest& requestCamVel, aaw_ros::MoveRobotResponse& execStatus);
    bool distanceZInputCallback(aaw_ros::MoveRobot_DistanceZRequest& requestDistanceZ, aaw_ros::MoveRobot_DistanceZResponse& execStatus);
};

#endif