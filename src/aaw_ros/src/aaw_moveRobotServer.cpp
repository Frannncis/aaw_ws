#include "aaw_moveRobotServer.h"

AAWMoveRobotServer::AAWMoveRobotServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    moveRobot_camVel_ = nh_.advertiseService("move_robot_input_camVel", &AAWMoveRobotServer::camVelInputCallback, this);
    moveRobot_distanceZ_ = nh_.advertiseService("move_robot_input_distanceZ", &AAWMoveRobotServer::distanceZInputCallback, this);
    moveRobot_ctrlVal_ = nh_.advertiseService("move_robot_input_ctrlVal", &AAWMoveRobotServer::ctrlValInputCallback, this);
    disableRobot_ = nh_.advertiseService("disable_robot_service", &AAWMoveRobotServer::disableRobotCallback, this);

    myTCPServerPtr_ = new AAWTCPServer(3000);
    std::vector<float> velAcc{3, 5, 5, 10};
    myTCPServerPtr_->setVelAcc(velAcc);
    myTCPServerPtr_->waitUntilConnected();
    sleep(3);
    AAWEnableRobot();
    std::cout<<"Robot enabled!\n";

    while(!(myTCPServerPtr_->move(originalCtrlVal_)))
        sleep(1);
    std::cout<<"Moved to original pos, ready to accept visual servo control!\n";

    coordTransformerPtr_ = new AAWCoordTransform(originalCtrlVal_);
}

AAWMoveRobotServer::~AAWMoveRobotServer()
{
    delete myTCPServerPtr_;
    delete coordTransformerPtr_;
}

bool AAWMoveRobotServer::camVelInputCallback(aaw_ros::MoveRobotRequest& requestCamVel, aaw_ros::MoveRobotResponse& execStatus)
{
    Eigen::Matrix<float, 6, 1> camVel;
    camVel(0) = requestCamVel.vx;
    camVel(1) = requestCamVel.vy;
    camVel(2) = requestCamVel.vz;
    camVel(3) = requestCamVel.wx;
    camVel(4) = requestCamVel.wy;
    camVel(5) = requestCamVel.wz;

    ctrlVal_.clear();
    ctrlVal_ = coordTransformerPtr_->getCtrlVal(camVel);
    execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
    
    return true;
}

/* 要注意在调用这个服务的地方，不能重复发送指令，因为接收的是增量，重复发送就出错了。
 * 或者调用之前就把这个增量拆分开，这样允许多次调用。  
 */
bool AAWMoveRobotServer::distanceZInputCallback(aaw_ros::MoveRobot_DistanceZRequest& requestDistanceZ, aaw_ros::MoveRobot_DistanceZResponse& execStatus)
{
    if (requestDistanceZ.isUp)
    {
        ctrlVal_[2] += requestDistanceZ.goUpDistance;
        execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
        return true;
    }
    else {
        ctrlVal_[2] -= requestDistanceZ.goDownDistance;
        execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
        return true;
    }
}

bool AAWMoveRobotServer::ctrlValInputCallback(aaw_ros::MoveRobot_CtrlValRequest& requestCtrlVal, aaw_ros::MoveRobot_CtrlValResponse& execStatus)
{
    ctrlVal_.clear();
    ctrlVal_[0] = requestCtrlVal.x;
    ctrlVal_[1] = requestCtrlVal.y;
    ctrlVal_[2] = requestCtrlVal.z;
    ctrlVal_[3] = requestCtrlVal.a;
    ctrlVal_[4] = requestCtrlVal.b;
    ctrlVal_[5] = requestCtrlVal.c;
    execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
    return true;
}

bool AAWMoveRobotServer::disableRobotCallback(aaw_ros::DisableRobotRequest& req, aaw_ros::DisableRobotResponse& execStatus)
{
    execStatus.ExecStatus = req.req;
    AAWDisableRobot();
    return true;
}

int AAWMoveRobotServer::AAWEnableRobot()
{
    while (!(myTCPServerPtr_->enableRobot()))
        sleep(1);
    std::cout<<"Robot enabled!\n";
}

int AAWMoveRobotServer::AAWDisableRobot()
{
    while (!(myTCPServerPtr_->disableRobot()))
        sleep(1);
    std::cout<<"Robot disabled!\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_moveRobotServer");
    ros::NodeHandle nh;

    // ros::ServiceServer service = n.advertiseService("move_robot_to_pos", callback);
    // ROS_INFO("Ready to move robot.");

    AAWMoveRobotServer amr(&nh);
    
    ros::spin();

    return 0;
}