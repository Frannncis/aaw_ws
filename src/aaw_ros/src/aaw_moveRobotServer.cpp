#include "aaw_moveRobotServer.h"

AAWMoveRobotServer::AAWMoveRobotServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    moveRobot_camVel_ = nh_.advertiseService("move_robot_input_camVel", &AAWMoveRobotServer::camVelInputCallback, this);
    moveRobot_distanceZ_ = nh_.advertiseService("move_robot_input_distanceZ", &AAWMoveRobotServer::distanceZInputCallback, this);

    myTCPServerPtr_ = new AAWTCPServer(3000);
    std::vector<float> velAcc{3, 5, 5, 10};
    myTCPServerPtr_->setVelAcc(velAcc);
    myTCPServerPtr_->waitUntilConnected();
    AAWEnableRobot();
    std::cout<<"Robot enabled!\n";

    std::vector<float> originalCtrlVal{-11.41949, -10.42905, 681.59697, 5.97, 0.38830, 0.10055};
    while(!(myTCPServerPtr_->move(originalCtrlVal)))
        sleep(1);
    std::cout<<"Moved to original pos, ready to accept visual servo control!\n";

    coordTransformerPtr_ = new AAWCoordTransform(originalCtrlVal);
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
    ctrlVal_[2] += requestDistanceZ.goUpDistance;
    execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal_);
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