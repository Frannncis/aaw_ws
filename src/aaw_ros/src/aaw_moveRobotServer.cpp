#include "aaw_moveRobotServer.h"

AAWMoveRobotServer::AAWMoveRobotServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    moveRobotService_ = nh_.advertiseService("move_robot_to_pos", &AAWMoveRobotServer::serviceCallback, this);
    ROS_INFO("Ready to move robot.");
    myTCPServerPtr_ = new AAWTCPServer(3000);
    std::vector<float> velAcc{3, 5, 5, 10};
    myTCPServerPtr_->setVelAcc(velAcc);
    myTCPServerPtr_->waitUntilConnected();
    AAWEnableRobot();
    std::cout<<"Robot enabled!\n";
    std::vector<float> originalCtrlVal{-11.41949, -10.42905, 681.59697, 5.97, 0.38830, 0.10055};
    while(!(myTCPServerPtr_->move(originalCtrlVal)))
        sleep(1);
    std::cout<<"Moved to original pos!\n";
    coordTransformerPtr_ = new AAWCoordTransform(originalCtrlVal);
}

AAWMoveRobotServer::~AAWMoveRobotServer()
{
    delete myTCPServerPtr_;
    delete coordTransformerPtr_;
}

bool AAWMoveRobotServer::serviceCallback(aaw_ros::MoveRobotRequest& requestCamVel, aaw_ros::MoveRobotResponse& execStatus)
{
    // ROS_INFO("Move robot service callback activated");
    Eigen::Matrix<float, 6, 1> camVel;
    camVel(0) = requestCamVel.vx;
    camVel(1) = requestCamVel.vy;
    camVel(2) = requestCamVel.vz;
    camVel(3) = requestCamVel.wx;
    camVel(4) = requestCamVel.wy;
    camVel(5) = requestCamVel.wz;

    std::vector<float> ctrlVal;
    ctrlVal = coordTransformerPtr_->getCtrlVal(camVel);
    execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal);
    // std::cout<<"Moved to ctrlVal:\n";
    // for (size_t i = 0; i < 6; ++i)
    //     std::cout<<ctrlVal[i]<<"\n";
    
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