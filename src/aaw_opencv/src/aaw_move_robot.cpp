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

//     bool serviceCallback(aaw_opencv::MoveRobotRequest& requestPos, aaw_opencv::MoveRobotResponse& execStatus);
// };

#include "aaw_move_robot.h"



AAWMoveRobotClass::AAWMoveRobotClass(ros::NodeHandle* nodehandle):nh_(*nodehandle)
{
    ROS_INFO("in class constructor of AAWMoveRobotClass");
    moveRobotService_ = nh_.advertiseService("move_robot_to_pos", &AAWMoveRobotClass::serviceCallback, this);
    ROS_INFO("Ready to move robot.");
    myTCPServerPtr_ = new AAWTCPServer(3000);   //没有写delete,暂时不知道放在哪里,反正只有一个对象,问题不大,后续再考虑
    std::vector<float> velAcc{3, 5, 5, 10};
    (*myTCPServerPtr_).setVelAcc(velAcc);
    std::cout<<"Client connected!\n";
}

bool AAWMoveRobotClass::serviceCallback(aaw_opencv::MoveRobotRequest& requestPos, aaw_opencv::MoveRobotResponse& execStatus)
{
    ROS_INFO("callback activated");
    std::vector<float> pos;
    pos.push_back(requestPos.x);
    pos.push_back(requestPos.y);
    pos.push_back(requestPos.z);
    pos.push_back(requestPos.a);
    pos.push_back(requestPos.b);
    pos.push_back(requestPos.c);

    std::cout<<"pos received: \n";
    for (size_t i = 0; i < 6; ++i)
    {
        std::cout<<pos[i]<<"\n";
    }

    execStatus.ExecStatus = 123;
    
    return true;
}

// bool callback(aaw_opencv::MoveRobotRequest& requestPos, aaw_opencv::MoveRobotResponse& execStatus)
// {
//     ROS_INFO("callback activated");
//     std::vector<float> pos;
//     pos.push_back(requestPos.x);
//     pos.push_back(requestPos.y);
//     pos.push_back(requestPos.z);
//     pos.push_back(requestPos.a);
//     pos.push_back(requestPos.b);
//     pos.push_back(requestPos.c);

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