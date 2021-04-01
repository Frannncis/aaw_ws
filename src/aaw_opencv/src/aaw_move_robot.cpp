// #include <ros/ros.h>
// #include "aaw_opencv/MoveRobot.h"
// #include <iostream>
// #include <vector>
// #include "aawtcpserver.h"

// class AAWMoveRobotClass
// {
// public:
//     AAWMoveRobotClass(ros::NodeHandle* nodehandle);
//     ~AAWMoveRobotClass();

// private:
//     ros::NodeHandle nh_;
//     ros::ServiceServer moveRobotService_;
//     AAWTCPServer myTCPServer_;

//     bool serviceCallback(aaw_opencv::MoveRobotRequest& requestCamVel, aaw_opencv::MoveRobotResponse& execStatus);
// };

#include "aaw_move_robot.h"

AAWMoveRobotClass::AAWMoveRobotClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("in class constructor of AAWMoveRobotClass");
    moveRobotService_ = nh_.advertiseService("move_robot_to_pos", &AAWMoveRobotClass::serviceCallback, this);
    ROS_INFO("Ready to move robot.");
    myTCPServerPtr_ = new AAWTCPServer(3000);   //没有写delete,暂时不知道放在哪里,反正只有一个对象,问题不大,后续再考虑
    std::vector<float> velAcc{3, 5, 5, 10};
    myTCPServerPtr_->setVelAcc(velAcc);
    myTCPServerPtr_->waitUntilConnected();
    std::cout<<"Client connected!\n";
    while (!(myTCPServerPtr_->enableRobot()))
        sleep(1);
    std::cout<<"Robot enabled!\n";
    std::vector<float> originalCtrlVal{-29, -4.45, 680, -0.5, 1.2, 0.4};
    while(!(myTCPServerPtr_->move(originalCtrlVal)))
        sleep(1);
    std::cout<<"Moved to original pos!\n";
    coordTransformerPtr_ = new AAWCoordTransform(originalCtrlVal);
}

bool AAWMoveRobotClass::serviceCallback(aaw_opencv::MoveRobotRequest& requestCamVel, aaw_opencv::MoveRobotResponse& execStatus)
{
    ROS_INFO("callback activated");
    Eigen::Matrix<float, 6, 1> camVel;
    camVel(0) = requestCamVel.vx;
    camVel(1) = requestCamVel.vy;
    camVel(2) = requestCamVel.vz;
    camVel(3) = requestCamVel.wx;
    camVel(4) = requestCamVel.wy;
    camVel(5) = requestCamVel.wz;
    std::cout<<"camVel received: \n";

    std::vector<float> ctrlVal;
    ctrlVal = coordTransformerPtr_->getCtrlVal(camVel);
    execStatus.ExecStatus = myTCPServerPtr_->move(ctrlVal);
    std::cout<<"Moved to ctrlVal:\n";
    for (size_t i = 0; i < 6; ++i)
        std::cout<<ctrlVal[i]<<"\n";
    
    return true;
}

int AAWMoveRobotClass::AAWEnableRobot()
{
    while (!(myTCPServerPtr_->enableRobot()))
        sleep(1);
    std::cout<<"Robot enabled!\n";
}

int AAWMoveRobotClass::AAWDisableRobot()
{
    while (!(myTCPServerPtr_->disableRobot()))
        sleep(1);
    std::cout<<"Robot disabled!\n";
}

// bool callback(aaw_opencv::MoveRobotRequest& requestCamVel, aaw_opencv::MoveRobotResponse& execStatus)
// {
//     ROS_INFO("callback activated");
//     std::vector<float> pos;
//     pos.push_back(requestCamVel.x);
//     pos.push_back(requestCamVel.y);
//     pos.push_back(requestCamVel.z);
//     pos.push_back(requestCamVel.a);
//     pos.push_back(requestCamVel.b);
//     pos.push_back(requestCamVel.c);

//     std::cout<<"pos received: \n";
//     for (size_t i = 0; i < 6; ++i)
//     {
//         std::cout<<pos[i]<<"\n";
//     }

//     execStatus.ExecStatus = 123;
    
//     return true;
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aaw_move_robot");
    ros::NodeHandle nh;

    // ros::ServiceServer service = n.advertiseService("move_robot_to_pos", callback);
    // ROS_INFO("Ready to move robot.");

    AAWMoveRobotClass amr(&nh);


    
    ros::spin();

    return 0;
}