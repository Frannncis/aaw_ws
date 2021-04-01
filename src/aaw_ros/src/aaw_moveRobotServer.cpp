#include "aaw_moveRobotServer.h"

AAWMoveRobotServer::AAWMoveRobotServer(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("In class constructor of AAWMoveRobotServer");
    moveRobotService_ = nh_.advertiseService("move_robot_to_pos", &AAWMoveRobotServer::serviceCallback, this);
    ROS_INFO("Ready to move robot.");
    myTCPServerPtr_ = new AAWTCPServer(3000);   //没有写delete,暂时不知道放在哪里,反正只有一个对象,问题不大,后续再考虑
    std::vector<float> velAcc{3, 5, 5, 10};
    myTCPServerPtr_->setVelAcc(velAcc);
    myTCPServerPtr_->waitUntilConnected();
    while (!(myTCPServerPtr_->enableRobot()))
        sleep(1);
    std::cout<<"Robot enabled!\n";
    std::vector<float> originalCtrlVal{-29, -4.45, 680, -0.5, 1.2, 0.4};
    while(!(myTCPServerPtr_->move(originalCtrlVal)))
        sleep(1);
    std::cout<<"Moved to original pos!\n";
    coordTransformerPtr_ = new AAWCoordTransform(originalCtrlVal);
}

bool AAWMoveRobotServer::serviceCallback(aaw_ros::MoveRobotRequest& requestCamVel, aaw_ros::MoveRobotResponse& execStatus)
{
    ROS_INFO("Move robot service callback activated");
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
    std::cout<<"Moved to ctrlVal:\n";
    for (size_t i = 0; i < 6; ++i)
        std::cout<<ctrlVal[i]<<"\n";
    
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