#ifndef AAW_MOVE_ROBOT_SERVER_H_
#define AAW_MOVE_ROBOT_SERVER_H_

#include <ros/ros.h>
#include "aaw_ros/MoveRobot.h"
#include "aaw_ros/MoveRobot_DistanceZ.h"
#include "aaw_ros/MoveRobot_CtrlVal.h"
#include "aaw_ros/ChangeRobotStatus.h"
#include "aaw_ros/ChangeTimeIntegration.h"
#include "aaw_ros/UpdateCoordTransformer.h"
#include <iostream>
#include <vector>
#include "aawtcpserver.h"
#include "aawcoordtransform.h"
#include "aaw_originalCtrlVal.h"

class AAWMoveRobotServer
{
public:
    AAWMoveRobotServer(ros::NodeHandle* nodehandle);
    ~AAWMoveRobotServer();

private:
    ros::NodeHandle nh_;
    ros::ServiceServer moveRobot_camVel_;
    ros::ServiceServer moveRobot_distanceZ_;
    ros::ServiceServer moveRobot_ctrlVal_;
    ros::ServiceServer changeRobotStatus_;
    ros::ServiceServer changeTimeIntegration_;
    ros::ServiceServer updateCoordTransformer_;

    AAWTCPServer *myTCPServerPtr_;
    AAWCoordTransform *coordTransformerPtr_;

    std::vector<float> ctrlVal_;

    void AAWEnableRobot();
    void AAWDisableRobot();
    bool camVelInputCallback(aaw_ros::MoveRobotRequest& requestCamVel, aaw_ros::MoveRobotResponse& execStatus);
    bool distanceZInputCallback(aaw_ros::MoveRobot_DistanceZRequest& requestDistanceZ, aaw_ros::MoveRobot_DistanceZResponse& execStatus);
    bool ctrlValInputCallback(aaw_ros::MoveRobot_CtrlValRequest& requestCtrlVal, aaw_ros::MoveRobot_CtrlValResponse& execStatus);
    bool changeRobotStatusCallback(aaw_ros::ChangeRobotStatusRequest& requestStatus, aaw_ros::ChangeRobotStatusResponse& execStatus);
    bool changeTimeIntegCallback(aaw_ros::ChangeTimeIntegrationRequest& reqTimeInteg, aaw_ros::ChangeTimeIntegrationResponse& execStatus);
    bool updateCoordTransCallback(aaw_ros::UpdateCoordTransformerRequest& req, aaw_ros::UpdateCoordTransformerResponse& res);
};

#endif