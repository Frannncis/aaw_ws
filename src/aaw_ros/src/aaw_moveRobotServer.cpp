#include "aaw_moveRobotServer.h"

/* 构造函数。
 * 发布４个ROS Service接受visualServo的运动控制请求。
 * 创建与并联机构通讯的TCP server并完成连接，使能机器人并将机器人移动到零点。
 * 主要完成坐标变换与TCP通讯。
 */
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

/* 析构函数，释放动态请求的空间。
 */
AAWMoveRobotServer::~AAWMoveRobotServer()
{
    delete myTCPServerPtr_;
    delete coordTransformerPtr_;
}

/* 相机伺服速度输入的运动控制回调函数。
 * 完成坐标变换并驱动控制。
 * @note 一旦开始由相机速度控制的视觉伺服过程，必须一直调用并更新坐标变换类中的当前坐标变换矩阵，即一直调用此处的coordTransformerPtr_->getCtrlVal(camVel)
 * 因为本次伺服依赖于上一次的坐标变换矩阵，不允许在伺服过程中进行其他形式的控制请求（如ctrlVal），这样会扰乱伺服过程。
 */
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

/* 输入并联机构控制量进行运动控制的回调函数，此函数不应在视觉伺服完成之前调用，主要目的是在完成undock之后控制并联机器人回零点。
 */
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

/* 控制并联机器人下使能的回调函数。
 */
bool AAWMoveRobotServer::disableRobotCallback(aaw_ros::DisableRobotRequest& req, aaw_ros::DisableRobotResponse& execStatus)
{
    execStatus.ExecStatus = req.req;
    AAWDisableRobot();
    return true;
}

/* 使能并联机器人
 */
int AAWMoveRobotServer::AAWEnableRobot()
{
    while (!(myTCPServerPtr_->enableRobot()))
        sleep(1);
    std::cout<<"Robot enabled!\n";
}

/* 下使能并联机器人
 */
int AAWMoveRobotServer::AAWDisableRobot()
{
    while (!(myTCPServerPtr_->disableRobot()))
        sleep(1);
    std::cout<<"Robot disabled!\n";
}

/* 此node接受运动控制请求，将请求转换成运动控制量，实现并联机构的远程驱动控制。
 */
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